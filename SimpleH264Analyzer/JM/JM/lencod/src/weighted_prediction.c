/**********************************************************************
 * Software Copyright Licensing Disclaimer
 *
 * This software module was originally developed by contributors to the
 * course of the development of ISO/IEC 14496-10 for reference purposes
 * and its performance may not have been optimized.  This software
 * module is an implementation of one or more tools as specified by
 * ISO/IEC 14496-10.  ISO/IEC gives users free license to this software
 * module or modifications thereof. Those intending to use this software
 * module in products are advised that its use may infringe existing
 * patents.  ISO/IEC have no liability for use of this software module
 * or modifications thereof.  The original contributors retain full
 * rights to modify and use the code for their own purposes, and to
 * assign or donate the code to third-parties.
 *
 * This copyright notice must be included in all copies or derivative
 * works.  Copyright (c) ISO/IEC 2004.
 **********************************************************************/

/*!
*************************************************************************************
* \file weighted_prediction.c
*
* \brief
*    Estimate weights for WP
*
* \author
*    Main contributors (see contributors.h for copyright, address and affiliation details)
*     - Alexis Michael Tourapis         <alexismt@ieee.org>
*************************************************************************************
*/
#include "contributors.h"

#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "image.h"


#define Clip(min,max,val) (((val)<(min))?(min):(((val)>(max))?(max):(val)))

/*!
************************************************************************
* \brief
*    Estimates reference picture weighting factors
************************************************************************
*/

void estimate_weighting_factor_P_slice()
{
  int i, j, n;
  
  int dc_org = 0;
  int index;
  int comp;
  double dc_ref[MAX_REFERENCE_PICTURES];
  
  pel_t*  ref_pic;   
  pel_t*  ref_pic_w;   
  int default_weight;
  int default_weight_chroma;
  int list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;
  int weight[2][MAX_REFERENCE_PICTURES][3]; 
  int offset[2][MAX_REFERENCE_PICTURES][3];       
  int clist;
  
  
  
  luma_log_weight_denom = 5;
  chroma_log_weight_denom = 5;
  wp_luma_round = 1 << (luma_log_weight_denom - 1);
  wp_chroma_round = 1 << (chroma_log_weight_denom - 1);
  default_weight = 1<<luma_log_weight_denom;
  default_weight_chroma = 1<<chroma_log_weight_denom;
  
  /* set all values to defaults */
  for (i = 0; i < 2 + list_offset; i++)
  {
    for (j = 0; j < listXsize[i]; j++)
    {
      for (n = 0; n < 3; n++)
      {
        wp_weight[i][j][n] = default_weight;
        wp_offset[i][j][n] = 0;
        offset[i][j][n] = 0;
      }
    }
  }
  
  for (i = 0; i < img->height; i++)
    for (j = 0; j < img->width; j++)
    {
      dc_org += imgY_org[i][j];
    }
    
    
    for (clist=0; clist<2 + list_offset; clist++)
    {
      for (n = 0; n < listXsize[clist]; n++)
      {
        dc_ref[n] = 0.0;
        
        ref_pic       = listX[clist][n]->imgY_11;
        ref_pic_w     = listX[clist][n]->imgY_11_w;
        
        // Y
        for (i = 0; i < img->height * img->width; i++)
        {
          dc_ref[n] += (double) ref_pic[i];
        }
        
        if (dc_ref[n] != 0)
          weight[clist][n][0] = (int) (default_weight * dc_org / dc_ref[n] + 0.5);
        else
          weight[clist][n][0] = default_weight;  // only used when reference picture is black
        if (weight[clist][n][0] < -64 || weight[clist][n][0] >127)
          weight[clist][n][0] = 32;
        
        
        
        
        
        
        /* for now always use default weight for chroma weight */
        weight[clist][n][1] = default_weight_chroma;
        weight[clist][n][2] = default_weight_chroma;
        
        
        
        /* store weighted reference pic for motion estimation */
        for (i = 0; i < img->height * img->width; i++)
        {          
          ref_pic_w[i] = Clip (0, 255, (((int) ref_pic[i] * weight[clist][n][0] + wp_luma_round) >> luma_log_weight_denom) + offset[clist][n][0]);
        }
        for (i = 0; i < 4*(img->height + 2*IMG_PAD_SIZE) ; i++)
        {
          for (j = 0; j< 4*(img->width + 2*IMG_PAD_SIZE); j++)
          {
            listX[LIST_0][n]->imgY_ups_w[i][j] =   Clip (0, 255, (((int) listX[LIST_0 ][n]->imgY_ups[i][j] * weight[clist][n][0] + wp_luma_round) >> luma_log_weight_denom) + offset[clist][n][0]);
          }
        }
      }
    }
    
    
    for (index = 0; index < listXsize[0]; index++)
    {
      for (comp=0; comp < 3; comp ++)
      {
        wp_weight[0][index][comp] = weight[0][index][comp];
        wp_offset[0][index][comp] = offset[0][index][comp];
        // printf("index %d component %d weight %d offset %d\n",index,comp,weight[0][index][comp],offset[0][index][comp]);
      }
    }
    
    
}

/*!
************************************************************************
* \brief
*    Estimates reference picture weighting factors
************************************************************************
*/
void estimate_weighting_factor_B_slice()
{
  int i, j, n, l;
  
  int x,z;
  double dc_org = 0.0;
  int index;
  int comp;
  double dc_ref[6][MAX_REFERENCE_PICTURES];
  
  int log_weight_denom;
  
  pel_t*  ref_pic;   
  pel_t*  ref_pic_w;   
  int default_weight;
  int default_weight_chroma;
  int list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;
  int weight[6][MAX_REFERENCE_PICTURES][3]; 
  int offset[6][MAX_REFERENCE_PICTURES][3];       
  int im_weight[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES][3]; 
  int im_offset[6][MAX_REFERENCE_PICTURES][MAX_REFERENCE_PICTURES][3]; 
  int clist;
  int wf_weight, wf_offset;
  
  luma_log_weight_denom = 5;
  chroma_log_weight_denom = 5;
  wp_luma_round = 1 << (luma_log_weight_denom - 1);
  wp_chroma_round = 1 << (chroma_log_weight_denom - 1);
  default_weight = 1<<luma_log_weight_denom;
  default_weight_chroma = 1<<chroma_log_weight_denom;
  
  /* set all values to defaults */
  for (i = 0; i < 2 + list_offset; i++)
  {
    for (j = 0; j < listXsize[i]; j++)
    {
      for (n = 0; n < 3; n++)
      {
        wp_weight[i][j][n] = default_weight;
        wp_offset[i][j][n] = 0;
        offset   [i][j][n] = 0;
        weight   [i][j][n] = default_weight;
      }
    }
  }
  
  for (i = 0; i < listXsize[LIST_0]; i++)
  {
    for (j = 0; j < listXsize[LIST_1]; j++)
    {
      int pt,p0;
      pt = (listX[LIST_1][j]->poc - listX[LIST_0][i]->poc);
      p0 = (enc_picture->poc - listX[LIST_0][i]->poc);
      for (comp = 0; comp < 3; comp++)
      {
        { // implicit weights
          
          if (pt == 0)
          {
            im_weight[1][i][j][comp] = 32 ;
            im_weight[0][i][j][comp] = 32;
            im_offset[1][i][j][comp] = 0;
            im_offset[0][i][j][comp] = 0;
          }
          else
          {            
            x = (16384 + (pt>>1))/pt;
            z = Clip(-1024, 1023, (x*p0 + 32 )>>6);
            im_weight[1][i][j][comp] = z>>2;
            if (im_weight[1][i][j][comp] < -64 || im_weight[1][i][j][comp] >128)
              im_weight[1][i][j][comp] = 32;
            im_weight[0][i][j][comp] = 64 - im_weight[1][i][j][comp];            
            im_offset[1][i][j][comp] = 0;
            im_offset[0][i][j][comp] = 0;
          }
        }
      }
      /*
      printf ("%d imp weight[%d][%d] = %d  , %d (%d %d %d) (%d %d) (%d %d)\n",enc_picture->poc, i, j,  im_weight[0][i][j][0], im_weight[1][i][j][0],
      enc_picture->poc,listX[LIST_0][i]->poc, listX[LIST_1][j]->poc,
      z,x,pt,p0);
      */
    }
  }
  
  
  
  
  if (input->WeightedBiprediction == 2) //! implicit mode
  {
    
    for (i = 0; i < listXsize[LIST_0]; i++)
    {
      for (j = 0; j < listXsize[LIST_1]; j++)
      {
        for (comp = 0; comp < 3; comp++)
        {
          log_weight_denom = (comp == 0) ? luma_log_weight_denom : chroma_log_weight_denom;         
          wbp_weight[1][i][j][comp] = im_weight[1][i][j][comp] ;
          wbp_weight[0][i][j][comp] = im_weight[0][i][j][comp];
        }
      }
    }
    
    for (clist=0; clist<2 + list_offset; clist++)
    {
      for (index = 0; index < listXsize[clist]; index++)
      {
        wp_weight[clist][index][0] = 1<<luma_log_weight_denom;
        wp_weight[clist][index][1] = 1<<chroma_log_weight_denom;
        wp_weight[clist][index][2] = 1<<chroma_log_weight_denom;
        wp_offset[clist][index][0] = 0;
        wp_offset[clist][index][1] = 0;
        wp_offset[clist][index][2] = 0;
      }
    }
    for (i = 0; i < listXsize[LIST_0]; i++)
    {
      for (j = 0; j < listXsize[LIST_1]; j++)
      {
        
        for (n = 0; n < img->height * img->width; n++)
        {
          listX[0][i]->imgY_11_w[n] = listX[0][i]->imgY_11[n];
          listX[1][j]->imgY_11_w[n] = listX[1][j]->imgY_11[n];
        }
        
        for (n = 0; n < 4*(img->height + 2*IMG_PAD_SIZE) ; n++)
        {
          for (l = 0; l< 4*(img->width + 2*IMG_PAD_SIZE); l++)
          {
            listX[LIST_0][i]->imgY_ups_w[n][l] =   listX[LIST_0 ][i]->imgY_ups[n][l];
            listX[LIST_1][j]->imgY_ups_w[n][l] =   listX[LIST_1 ][j]->imgY_ups[n][l];
          }
        }
      }
    }
  }
  else
  {
    for (i = 0; i < img->height; i++)
    {
      for (j = 0; j < img->width; j++)
      {
        dc_org += (double) imgY_org[i][j];
      }
    }
    
    for (clist=0; clist<2 + list_offset; clist++)
    {
      for (n = 0; n < listXsize[clist]; n++)
      {
        dc_ref[clist][n] = 0;
        
        ref_pic       = listX[clist][n]->imgY_11;
        ref_pic_w     = listX[clist][n]->imgY_11_w;
        
        // Y
        for (i = 0; i < img->height * img->width; i++)
        {
          dc_ref[clist][n] += (double) ref_pic[i];
        }
        if (dc_ref[clist][n] != 0)
          wf_weight = (int) (default_weight * dc_org / dc_ref[clist][n] + 0.5);
        else
          wf_weight = default_weight;  // only used when reference picture is black
        if ( (wf_weight<-64) || (wf_weight>127) )
        {
          wf_weight = default_weight;
        }
        wf_offset = 0;
        
        
        //    printf("dc_org = %d, dc_ref = %d, weight[%d] = %d\n",dc_org, dc_ref[n],n,weight[n][0]);        
        
        weight[clist][n][0] = wf_weight;
        weight[clist][n][1] = default_weight_chroma;
        weight[clist][n][2] = default_weight_chroma;
        offset[clist][n][0] = 0;
        offset[clist][n][1] = 0;
        offset[clist][n][2] = 0;
        
        
        /* store weighted reference pic for motion estimation */
        for (i = 0; i < img->height * img->width; i++)
        {
          ref_pic_w[i] = Clip (0, 255, (((int) ref_pic[i] * wf_weight + wp_luma_round) >> luma_log_weight_denom) + wf_offset);
        }
        for (i = 0; i < 4*(img->height + 2*IMG_PAD_SIZE) ; i++)
        {
          for (j = 0; j< 4*(img->width + 2*IMG_PAD_SIZE); j++)
          {
            listX[LIST_0][n]->imgY_ups_w[i][j] =   Clip (0, 255, (((int) listX[LIST_0][n]->imgY_ups[i][j] * wf_weight + wp_luma_round) >> (luma_log_weight_denom)) + wf_offset );      	
          }
          
        }
      }
    }
    
    if (img->type == B_SLICE && (input->WeightedBiprediction == 1))
    {
      for (clist=0; clist<2 + list_offset; clist++)
      {
        for (index = 0; index < listXsize[clist]; index++)
        {
          for (comp = 0; comp < 3; comp++)
          {
            wp_weight[clist][index][comp] = weight[clist][index][comp];
            wp_offset[clist][index][comp] = offset[clist][index][comp];
          }
        }
      }
    }
    else
    {    
      for (clist=0; clist<2 + list_offset; clist++)
      {
        for (index = 0; index < listXsize[clist]; index++)
        {
          wp_weight[clist][index][0] = 1<<luma_log_weight_denom;
          wp_weight[clist][index][1] = 1<<chroma_log_weight_denom;
          wp_weight[clist][index][2] = 1<<chroma_log_weight_denom;
          wp_offset[clist][index][0] = 0;
          wp_offset[clist][index][1] = 0;
          wp_offset[clist][index][2] = 0;
        }
      }
      for (i = 0; i < listXsize[LIST_0]; i++)
      {
        for (j = 0; j < listXsize[LIST_1]; j++)
        {
          for (comp = 0; comp < 3; comp++)
          {
            log_weight_denom = (comp == 0) ? luma_log_weight_denom : chroma_log_weight_denom;
            wbp_weight[0][i][j][comp] = wp_weight[0][i][comp];
            wbp_weight[1][i][j][comp] = wp_weight[1][j][comp];
          }
          
          //printf ("bpw weight[%d][%d] = %d  , %d (%d %d %d) (%d %d) (%d %d)\n", i, j, wbp_weight[0][i][j][0], wbp_weight[1][i][j][0],
          //enc_picture->poc,listX[LIST_0][i]->poc, listX[LIST_1][j]->poc,
          //z,x,pt,p0);
          
        }
      }
    }
      }
    }
    
    
