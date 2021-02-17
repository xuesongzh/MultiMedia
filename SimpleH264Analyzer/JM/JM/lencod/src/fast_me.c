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
/*
***********************************************************************
* COPYRIGHT AND WARRANTY INFORMATION
*
* Copyright 2001, International Telecommunications Union, Geneva
*
* DISCLAIMER OF WARRANTY
*
* These software programs are available to the user without any
* license fee or royalty on an "as is" basis. The ITU disclaims
* any and all warranties, whether express, implied, or
* statutory, including any implied warranties of merchantability
* or of fitness for a particular purpose.  In no event shall the
* contributor or the ITU be liable for any incidental, punitive, or
* consequential damages of any kind whatsoever arising from the
* use of these programs.
*
* This disclaimer of warranty extends to the user of these programs
* and user's customers, employees, agents, transferees, successors,
* and assigns.
*
* The ITU does not represent or warrant that the programs furnished
* hereunder are free of infringement of any third-party patents.
* Commercial implementations of ITU-T Recommendations, including
* shareware, may be subject to royalty fees to patent holders.
* Information regarding the ITU-T patent policy is available from
* the ITU Web site at http://www.itu.int.
*
* THIS IS NOT A GRANT OF PATENT RIGHTS - SEE THE ITU-T PATENT POLICY.
************************************************************************
*/
/*!
 ************************************************************************
 * \brief
 * Fast integer pel motion estimation and fractional pel motion estimation
 * algorithms are described in this file.
 * 1. get_mem_FME() and free_mem_FME() are functions for allocation and release
 *    of memories about motion estimation
 * 2. FME_BlockMotionSearch() is the function for fast integer pel motion 
 *    estimation and fractional pel motion estimation
 * 3. DefineThreshold() defined thresholds for early termination
 * \ Main contributors: (see contributors.h for copyright, address and affiliation details)
 *   Zhibo Chen         <chenzhibo@tsinghua.org.cn>
 *   JianFeng Xu        <fenax@video.mdc.tsinghua.edu.cn>  
 * \date   : 2003.8
 ************************************************************************
 */

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <memory.h>
#include <assert.h>



#include "memalloc.h"
#include "fast_me.h"
#include "refbuf.h"
#include "mbuffer.h"
#include "image.h"

#define Q_BITS          15

extern  int*   byte_abs;
extern  int*   mvbits;
extern  int*   spiral_search_x;
extern  int*   spiral_search_y;


static pel_t (*PelY_14) (pel_t**, int, int, int, int);
static const int quant_coef[6][4][4] = {
  {{13107, 8066,13107, 8066},{ 8066, 5243, 8066, 5243},{13107, 8066,13107, 8066},{ 8066, 5243, 8066, 5243}},
  {{11916, 7490,11916, 7490},{ 7490, 4660, 7490, 4660},{11916, 7490,11916, 7490},{ 7490, 4660, 7490, 4660}},
  {{10082, 6554,10082, 6554},{ 6554, 4194, 6554, 4194},{10082, 6554,10082, 6554},{ 6554, 4194, 6554, 4194}},
  {{ 9362, 5825, 9362, 5825},{ 5825, 3647, 5825, 3647},{ 9362, 5825, 9362, 5825},{ 5825, 3647, 5825, 3647}},
  {{ 8192, 5243, 8192, 5243},{ 5243, 3355, 5243, 3355},{ 8192, 5243, 8192, 5243},{ 5243, 3355, 5243, 3355}},
  {{ 7282, 4559, 7282, 4559},{ 4559, 2893, 4559, 2893},{ 7282, 4559, 7282, 4559},{ 4559, 2893, 4559, 2893}}
};


void DefineThreshold()
{
  static float ThresholdFac[8] = {0,8,4,4,2.5,1.5,1.5,1}; 
  static int ThreshUp[8] = {0, 1024,512,512,448,384,384,384};

  AlphaSec[1] = 0.01f;
  AlphaSec[2] = 0.01f;
  AlphaSec[3] = 0.01f;
  AlphaSec[4] = 0.02f;
  AlphaSec[5] = 0.03f;
  AlphaSec[6] = 0.03f;
  AlphaSec[7] = 0.04f;

  AlphaThird[1] = 0.06f;
  AlphaThird[2] = 0.07f;
  AlphaThird[3] = 0.07f;
  AlphaThird[4] = 0.08f;
  AlphaThird[5] = 0.12f;
  AlphaThird[6] = 0.11f;
  AlphaThird[7] = 0.15f;

  DefineThresholdMB();
  return;
}

void DefineThresholdMB()
{
  int gb_qp_per    = (input->qpN-MIN_QP)/6;
  int gb_qp_rem    = (input->qpN-MIN_QP)%6;
  
  int gb_q_bits    = Q_BITS+gb_qp_per;
  int gb_qp_const,Thresh4x4;

  if (img->type == I_SLICE)
    gb_qp_const=(1<<gb_q_bits)/3;    // intra
  else
    gb_qp_const=(1<<gb_q_bits)/6;    // inter
  
  Thresh4x4 =   ((1<<gb_q_bits) - gb_qp_const)/quant_coef[gb_qp_rem][0][0];
  Quantize_step = Thresh4x4/(4*5.61f);
  Bsize[7]=(16*16)*Quantize_step;

  Bsize[6]=Bsize[7]*4;
  Bsize[5]=Bsize[7]*4;
  Bsize[4]=Bsize[5]*4;
  Bsize[3]=Bsize[4]*4;
  Bsize[2]=Bsize[4]*4;
  Bsize[1]=Bsize[2]*4;
}

/*!
 ************************************************************************
 * \brief
 *    Dynamic memory allocation of all infomation needed for Fast ME
 * \par Input:
 * \return Number of allocated bytes
 * \Date: 2003/3
 ************************************************************************
 */

int get_mem_mincost (int****** mv)
{
  int i, j, k, l;

  if ((*mv = (int*****)calloc(input->img_width/4,sizeof(int****))) == NULL)
    no_mem_exit ("get_mem_mv: mv");
  for (i=0; i<input->img_width/4; i++)
  {
    if (((*mv)[i] = (int****)calloc(input->img_height/4,sizeof(int***))) == NULL)
      no_mem_exit ("get_mem_mv: mv");
    for (j=0; j<input->img_height/4; j++)
    {
      if (((*mv)[i][j] = (int***)calloc(img->max_num_references, sizeof(int**))) == NULL)
        no_mem_exit ("get_mem_mv: mv");
      for (k=0; k<img->max_num_references; k++)
      {
        if (((*mv)[i][j][k] = (int**)calloc(9,sizeof(int*))) == NULL)
          no_mem_exit ("get_mem_mv: mv");
        for (l=0; l<9; l++)
          if (((*mv)[i][j][k][l] = (int*)calloc(3,sizeof(int))) == NULL)
            no_mem_exit ("get_mem_mv: mv");
      }
    }
  }

  return input->img_width/4*input->img_height/4*img->max_num_references*9*3*sizeof(int);
}
/*!
 *******************************************************************************
 * \brief
 *    Dynamic memory allocation of all infomation needed for backward prediction
 * \par Input:
 * \return Number of allocated bytes
 * \Date: 2003/3
 *******************************************************************************
 */
int get_mem_bwmincost (int****** mv)
{
  int i, j, k, l;


  if ((*mv = (int*****)calloc(input->img_width/4,sizeof(int****))) == NULL)
    no_mem_exit ("get_mem_mv: mv");
  for (i=0; i<input->img_width/4; i++)
  {
    if (((*mv)[i] = (int****)calloc(input->img_height/4,sizeof(int***))) == NULL)
      no_mem_exit ("get_mem_mv: mv");
    for (j=0; j<input->img_height/4; j++)
    {
      if (((*mv)[i][j] = (int***)calloc(img->max_num_references,sizeof(int**))) == NULL)
        no_mem_exit ("get_mem_mv: mv");
      for (k=0; k<img->max_num_references; k++)
      {
        if (((*mv)[i][j][k] = (int**)calloc(9,sizeof(int*))) == NULL)
          no_mem_exit ("get_mem_mv: mv");
        for (l=0; l<9; l++)
          if (((*mv)[i][j][k][l] = (int*)calloc(3,sizeof(int))) == NULL)
            no_mem_exit ("get_mem_mv: mv");
      }
    }
  }

  return input->img_width/4*input->img_height/4*img->max_num_references*9*3*sizeof(int);
}

int get_mem_FME()
{
  int memory_size = 0;
  memory_size += get_mem2Dint(&McostState, 2*input->search_range+1, 2*input->search_range+1);
  memory_size += get_mem_mincost (&(all_mincost));
  memory_size += get_mem_bwmincost(&(all_bwmincost));
  memory_size += get_mem2D(&SearchState,7,7);
  
  return memory_size;
}
/*!
 ************************************************************************
 * \brief
 *    free the memory allocated for of all infomation needed for Fast ME
 * \par Input:
 * \Date: 2003/3
 ************************************************************************
 */
void free_mem_mincost (int***** mv)
{
  int i, j, k, l;

  for (i=0; i<input->img_width/4; i++)
  {
    for (j=0; j<input->img_height/4; j++)
    {
      for (k=0; k<img->max_num_references; k++)
      {
        for (l=0; l<9; l++)
          free (mv[i][j][k][l]);
        free (mv[i][j][k]);
      }
      free (mv[i][j]);
    }
    free (mv[i]);
  }
  free (mv);
}

/*!
 ***********************************************************************************
 * \brief
 *    free the memory allocated for of all infomation needed for backward prediction
 * \par Input:
 * \Date: 2003/3
 ***********************************************************************************
 */
void free_mem_bwmincost (int***** mv)
{
  int i, j, k, l;

  for (i=0; i<input->img_width/4; i++)
  {
    for (j=0; j<input->img_height/4; j++)
    {
      for (k=0; k<img->max_num_references; k++)
      {
        for (l=0; l<9; l++)
          free (mv[i][j][k][l]);
        free (mv[i][j][k]);
      }
      free (mv[i][j]);
    }
    free (mv[i]);
  }
  free (mv);
}

void free_mem_FME()
{
  free_mem2Dint(McostState);
  free_mem_mincost (all_mincost);
  free_mem_bwmincost(all_bwmincost);

  free_mem2D(SearchState);
}


int PartCalMad(pel_t *ref_pic,pel_t** orig_pic,pel_t *(*get_ref_line)(int, pel_t*, int, int, int, int), int blocksize_y,int blocksize_x, int blocksize_x4,int mcost,int min_mcost,int cand_x,int cand_y)
{
  int y,x4;
  int height=((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))?img->height/2:img->height;
  pel_t *orig_line, *ref_line;
  for (y=0; y<blocksize_y; y++)
    {
    ref_line  = get_ref_line (blocksize_x, ref_pic, cand_y+y, cand_x, /*img->*/height, img->width);//2004.3.3
    orig_line = orig_pic [y];
    
    for (x4=0; x4<blocksize_x4; x4++)
    {
      mcost += byte_abs[ *orig_line++ - *ref_line++ ];
      mcost += byte_abs[ *orig_line++ - *ref_line++ ];
      mcost += byte_abs[ *orig_line++ - *ref_line++ ];
      mcost += byte_abs[ *orig_line++ - *ref_line++ ];
    }
    if (mcost >= min_mcost)
    {
      break;
    }
    }
    return mcost;
}

/*!
 ************************************************************************
 * \brief
 *    FastIntegerPelBlockMotionSearch: fast pixel block motion search 
 *    this algrithm is called UMHexagonS(see JVT-D016),which includes 
 *    four steps with different kinds of search patterns
 * \par Input:
 * pel_t**   orig_pic,     // <--  original picture
 * int       ref,          // <--  reference frame (0... or -1 (backward))
 * int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB block
 * int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
 * int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
 * int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
 * int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
 * int*      mv_x,         //  --> motion vector (x) - in pel units
 * int*      mv_y,         //  --> motion vector (y) - in pel units
 * int       search_range, // <--  1-d search range in pel units                         
 * int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
 * double    lambda        // <--  lagrangian parameter for determining motion cost
 * \par
 * Three macro definitions defined in this program:
 * 1. EARLY_TERMINATION: early termination algrithm, refer to JVT-D016.doc
 * 2. SEARCH_ONE_PIXEL: search one pixel in search range
 * 3. SEARCH_ONE_PIXEL1(value_iAbort): search one pixel in search range,
 *                                 but give a parameter to show if mincost refeshed
 * \ Main contributors: (see contributors.h for copyright, address and affiliation details)
 *   Zhibo Chen         <chenzhibo@tsinghua.org.cn>
 *   JianFeng Xu        <fenax@video.mdc.tsinghua.edu.cn>  
 * \date   : 2003.8
 ************************************************************************
 */
int                                     //  ==> minimum motion cost after search
FastIntegerPelBlockMotionSearch  (pel_t**   orig_pic,     // <--  not used
                  int       ref,          // <--  reference frame (0... or -1 (backward))
                  int       list,
                  int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB block
                  int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
                  int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
                  int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
                  int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
                  int*      mv_x,         //  --> motion vector (x) - in pel units
                  int*      mv_y,         //  --> motion vector (y) - in pel units
                  int       search_range, // <--  1-d search range in pel units                         
                  int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
                  double    lambda)       // <--  lagrangian parameter for determining motion cost
{
  static int Diamond_x[4] = {-1, 0, 1, 0};
  static int Diamond_y[4] = {0, 1, 0, -1};
  static int Hexagon_x[6] = {2, 1, -1, -2, -1, 1};
  static int Hexagon_y[6] = {0, -2, -2, 0,  2, 2};
  static int Big_Hexagon_x[16] = {0,-2, -4,-4,-4, -4, -4, -2,  0,  2,  4,  4, 4, 4, 4, 2};
  static int Big_Hexagon_y[16] = {4, 3, 2,  1, 0, -1, -2, -3, -4, -3, -2, -1, 0, 1, 2, 3};

  int   pos, cand_x, cand_y,  mcost;
  pel_t *(*get_ref_line)(int, pel_t*, int, int, int, int);
  int   list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;
  pel_t*  ref_pic       = listX[list+list_offset][ref]->imgY_11;//img->type==B_IMG? Refbuf11 [ref+((mref==mref_fld)) +1] : Refbuf11[ref];
  int   best_pos      = 0;                                        // position with minimum motion cost
  int   max_pos       = (2*search_range+1)*(2*search_range+1);    // number of search positions
  int   lambda_factor = LAMBDA_FACTOR (lambda);                   // factor for determining lagragian motion cost
  int   mvshift       = 2;                  // motion vector shift for getting sub-pel units
  int   blocksize_y   = input->blc_size[blocktype][1];            // vertical block size
  int   blocksize_x   = input->blc_size[blocktype][0];            // horizontal block size
  int   blocksize_x4  = blocksize_x >> 2;                         // horizontal block size in 4-pel units
  int   pred_x        = (pic_pix_x << mvshift) + pred_mv_x;       // predicted position x (in sub-pel units)
  int   pred_y        = (pic_pix_y << mvshift) + pred_mv_y;       // predicted position y (in sub-pel units)
  int   center_x      = pic_pix_x + *mv_x;                        // center position x (in pel units)
  int   center_y      = pic_pix_y + *mv_y;                        // center position y (in pel units)
  int    best_x, best_y;
  int   check_for_00  = (blocktype==1 && !input->rdopt && img->type!=B_SLICE && ref==0);
  int   search_step,iYMinNow, iXMinNow;
  int   i,m, iSADLayer; 
  int   iAbort;
  int       N_Bframe = input->successive_Bframe;
  float betaSec,betaThird;
  int height=((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))?img->height/2:img->height;
   

  //===== set function for getting reference picture lines =====
  if ((center_x > search_range) && (center_x < img->width -1-search_range-blocksize_x) &&
    (center_y > search_range) && (center_y < height-1-search_range-blocksize_y)   )
  {
    get_ref_line = FastLineX;
  }
  else
  {
    get_ref_line = UMVLineX;
  }
  
  //////allocate memory for search state//////////////////////////
  memset(McostState[0],0,(2*search_range+1)*(2*search_range+1)*4);
  
   ///////Threshold defined for early termination///////////////////  
  if(ref>0) 
  {
    if(pred_SAD_ref!=0)
    {
      betaSec = Bsize[blocktype]/(pred_SAD_ref*pred_SAD_ref)-AlphaSec[blocktype];
      betaThird = Bsize[blocktype]/(pred_SAD_ref*pred_SAD_ref)-AlphaThird[blocktype];
    }
    else
    {
      betaSec = 0;
      betaThird = 0;
    }
  }
  else 
  {
    if(blocktype==1)
    {
      if(pred_SAD_space !=0)
      {
        betaSec = Bsize[blocktype]/(pred_SAD_space*pred_SAD_space)-AlphaSec[blocktype];
        betaThird = Bsize[blocktype]/(pred_SAD_space*pred_SAD_space)-AlphaThird[blocktype];
      }
      else
      {
        betaSec = 0;
        betaThird = 0;
      }
    }
    else
    {
      if(pred_SAD_uplayer !=0)
      {
        betaSec = Bsize[blocktype]/(pred_SAD_uplayer*pred_SAD_uplayer)-AlphaSec[blocktype];
        betaThird = Bsize[blocktype]/(pred_SAD_uplayer*pred_SAD_uplayer)-AlphaThird[blocktype];
      }
      else
      {
        betaSec = 0;
        betaThird = 0;
      }
    }
  }
  /*****************************/

  //check the center median predictor
  cand_x = center_x ;
  cand_y = center_y ;
  mcost = MV_COST (lambda_factor, mvshift, cand_x, cand_y, pred_x, pred_y);
  mcost = PartCalMad(ref_pic, orig_pic, get_ref_line,blocksize_y,blocksize_x,blocksize_x4,mcost,min_mcost,cand_x,cand_y);
  McostState[search_range][search_range] = mcost;
  if (mcost < min_mcost)
  {
    min_mcost = mcost;
    best_x = cand_x;
    best_y = cand_y;
  }

  iXMinNow = best_x;
  iYMinNow = best_y;
  for (m = 0; m < 4; m++)
  {   
    cand_x = iXMinNow + Diamond_x[m];
    cand_y = iYMinNow + Diamond_y[m];   
    SEARCH_ONE_PIXEL
  } 

  if(center_x != pic_pix_x || center_y != pic_pix_y)
  {
    cand_x = pic_pix_x ;
    cand_y = pic_pix_y ;
    SEARCH_ONE_PIXEL

    iXMinNow = best_x;
    iYMinNow = best_y;
    for (m = 0; m < 4; m++)
    {   
      cand_x = iXMinNow + Diamond_x[m];
      cand_y = iYMinNow + Diamond_y[m];   
      SEARCH_ONE_PIXEL
    } 
  }
  
    if(blocktype>1)
  {
    cand_x = pic_pix_x + (pred_MV_uplayer[0]/4);
    cand_y = pic_pix_y + (pred_MV_uplayer[1]/4);
    SEARCH_ONE_PIXEL
    if ((min_mcost-pred_SAD_uplayer)<pred_SAD_uplayer*betaThird)
      goto third_step;
    else if((min_mcost-pred_SAD_uplayer)<pred_SAD_uplayer*betaSec)
      goto sec_step;
  } 

  //coordinate position prediction
  if ((img->number > 1 + ref && ref!=-1) || (list == 1 && (Bframe_ctr%N_Bframe) > 1))  //for debug
  {
    cand_x = pic_pix_x + pred_MV_time[0]/4;
    cand_y = pic_pix_y + pred_MV_time[1]/4;
    SEARCH_ONE_PIXEL
  }

  //prediciton using mV of last ref moiton vector
  if (input->PicInterlace == FIELD_CODING)
  {
    if ((list==0 && ref > 0) || (img->type == B_SLICE && list == 0 && (ref==0 ||ref==2 ) )) 
      //Notes: for interlace case, ref==1 should be added
    {
      cand_x = pic_pix_x + pred_MV_ref[0]/4;
      cand_y = pic_pix_y + pred_MV_ref[1]/4;
      SEARCH_ONE_PIXEL
    }
  }
  else
  {
    if ((list==0 && ref > 0) || (img->type == B_SLICE && list == 0 && ref==0 )) 
      //Notes: for interlace case, ref==1 should be added
    {
      cand_x = pic_pix_x + pred_MV_ref[0]/4;
      cand_y = pic_pix_y + pred_MV_ref[1]/4;
      SEARCH_ONE_PIXEL
    }
  }
  //small local search
  iXMinNow = best_x;
  iYMinNow = best_y;
  for (m = 0; m < 4; m++)
  {   
    cand_x = iXMinNow + Diamond_x[m];
    cand_y = iYMinNow + Diamond_y[m];   
    SEARCH_ONE_PIXEL
  } 

  //early termination algrithm, refer to JVT-D016
    EARLY_TERMINATION
  
  if(blocktype>6)
    goto sec_step;
  else
    goto first_step;
  
first_step: //Unsymmetrical-cross search 
  iXMinNow = best_x;
  iYMinNow = best_y;
  
  for(i=1;i<=search_range/2;i++)
  {
    search_step = 2*i - 1;
    cand_x = iXMinNow + search_step;
    cand_y = iYMinNow ;
    SEARCH_ONE_PIXEL    
    cand_x = iXMinNow - search_step;
    cand_y = iYMinNow ;
    SEARCH_ONE_PIXEL
  }
  
  for(i=1;i<=search_range/4;i++)
  {
    search_step = 2*i - 1;
    cand_x = iXMinNow ;
    cand_y = iYMinNow + search_step;
    SEARCH_ONE_PIXEL
    cand_x = iXMinNow ;
    cand_y = iYMinNow - search_step;
    SEARCH_ONE_PIXEL
  }
  //early termination algrithm, refer to JVT-D016
    EARLY_TERMINATION
  
  iXMinNow = best_x;
  iYMinNow = best_y;
    // Uneven Multi-Hexagon-grid Search 
  for(pos=1;pos<25;pos++)
  {
    cand_x = iXMinNow + spiral_search_x[pos];
    cand_y = iYMinNow + spiral_search_y[pos];
    SEARCH_ONE_PIXEL
  }
  //early termination algrithm, refer to JVT-D016
    EARLY_TERMINATION
  
  for(i=1;i<=search_range/4; i++)
  {
    iAbort = 0;   
    for (m = 0; m < 16; m++)
    {
      cand_x = iXMinNow + Big_Hexagon_x[m]*i;
      cand_y = iYMinNow + Big_Hexagon_y[m]*i; 
      SEARCH_ONE_PIXEL1(1)
    }
    if (iAbort)
    { 
      //early termination algrithm, refer to JVT-D016
      EARLY_TERMINATION
    }
  }
sec_step:  //Extended Hexagon-based Search
      iXMinNow = best_x;
      iYMinNow = best_y;
      for(i=0;i<search_range;i++) 
      {
        iAbort = 1;   
        for (m = 0; m < 6; m++)
        {   
          cand_x = iXMinNow + Hexagon_x[m];
          cand_y = iYMinNow + Hexagon_y[m];   
          SEARCH_ONE_PIXEL1(0)
        } 
        if(iAbort)
          break;
        iXMinNow = best_x;
        iYMinNow = best_y;
      }
third_step: // the third step with a small search pattern
      iXMinNow = best_x;
      iYMinNow = best_y;
      for(i=0;i<search_range;i++) 
      {
        iSADLayer = 65536;
        iAbort = 1;   
        for (m = 0; m < 4; m++)
        {   
          cand_x = iXMinNow + Diamond_x[m];
          cand_y = iYMinNow + Diamond_y[m];   
          SEARCH_ONE_PIXEL1(0)
        } 
        if(iAbort)
          break;
        iXMinNow = best_x;
        iYMinNow = best_y;
      }

      *mv_x = best_x - pic_pix_x;
      *mv_y = best_y - pic_pix_y; 
      return min_mcost;
  }


  /*!
 ************************************************************************
 * \brief
 * Functions for fast fractional pel motion estimation.
 * 1. int AddUpSADQuarter() returns SADT of a fractiona pel MV
 * 2. int FastSubPelBlockMotionSearch () proceed the fast fractional pel ME
 * \authors: Zhibo Chen
 *           Dept.of EE, Tsinghua Univ.
 * \date   : 2003.4
 ************************************************************************
 */
int AddUpSADQuarter(int pic_pix_x,int pic_pix_y,int blocksize_x,int blocksize_y,
                    int cand_mv_x,int cand_mv_y, StorablePicture *ref_picture, pel_t**   orig_pic, 
                    int Mvmcost, int min_mcost,int useABT)
{
  int abort_search, y0, x0, rx0, ry0, ry; 
  pel_t *orig_line;
  int   diff[16], *d; 
  int  mcost = Mvmcost;
  int yy,kk,xx;
  int   curr_diff[MB_BLOCK_SIZE][MB_BLOCK_SIZE]; // for ABT SATD calculation
//2004.3.3
  pel_t **ref_pic = ref_picture->imgY_ups;
  int img_width  = ref_picture->size_x;
  int img_height = ref_picture->size_y;

  
  for (y0=0, abort_search=0; y0<blocksize_y && !abort_search; y0+=4)
  {
    ry0 = ((pic_pix_y+y0)<<2) + cand_mv_y;
    
    for (x0=0; x0<blocksize_x; x0+=4)
    {
      rx0 = ((pic_pix_x+x0)<<2) + cand_mv_x;
      d   = diff;
      
      orig_line = orig_pic [y0  ];    ry=ry0;
      *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
      *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
      *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
      *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+ 12, img_height, img_width);
      
      orig_line = orig_pic [y0+1];    ry=ry0+4;
      *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
      *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
      *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
      *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+ 12, img_height, img_width);
      
      orig_line = orig_pic [y0+2];    ry=ry0+8;
      *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
      *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
      *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
      *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+ 12, img_height, img_width);
      
      orig_line = orig_pic [y0+3];    ry=ry0+12;
      *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
      *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
      *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
      *d        = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+ 12, img_height, img_width);
      
      if (!useABT)
      {
        if ((mcost += SATD (diff, input->hadamard)) > min_mcost)
        {
          abort_search = 1;
          break;
        }
      }
      else  // copy diff to curr_diff for ABT SATD calculation
      {
        for (yy=y0,kk=0; yy<y0+4; yy++)
          for (xx=x0; xx<x0+4; xx++, kk++)
            curr_diff[yy][xx] = diff[kk];
      }
    }
  }
  
  return mcost;
}


int                                                   //  ==> minimum motion cost after search
FastSubPelBlockMotionSearch (pel_t**   orig_pic,      // <--  original pixel values for the AxB block
                             int       ref,           // <--  reference frame (0... or -1 (backward))
                             int       list,
                             int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                             int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                             int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                             int       pred_mv_x,     // <--  motion vector predictor (x) in sub-pel units
                             int       pred_mv_y,     // <--  motion vector predictor (y) in sub-pel units
                             int*      mv_x,          // <--> in: search center (x) / out: motion vector (x) - in pel units
                             int*      mv_y,          // <--> in: search center (y) / out: motion vector (y) - in pel units
                             int       search_pos2,   // <--  search positions for    half-pel search  (default: 9)
                             int       search_pos4,   // <--  search positions for quarter-pel search  (default: 9)
                             int       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                             double    lambda,
                             int useABT)        // <--  lagrangian parameter for determining motion cost
{
  static int Diamond_x[4] = {-1, 0, 1, 0};
  static int Diamond_y[4] = {0, 1, 0, -1};
  int   mcost;
  int   cand_mv_x, cand_mv_y;
  
  int   incr            = list==1 ? ((!img->fld_type)&&(enc_picture!=enc_frame_picture)&&(img->type==B_SLICE)) : (enc_picture==enc_frame_picture)&&(img->type==B_SLICE) ;
  int   list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;
  StorablePicture *ref_picture = listX[list+list_offset][ref];
  pel_t **ref_pic = ref_picture->imgY_ups;
  
  int   lambda_factor   = LAMBDA_FACTOR (lambda);
  int   mv_shift        = 0;
  int   check_position0 = (blocktype==1 && *mv_x==0 && *mv_y==0 && input->hadamard && !input->rdopt && img->type!=B_SLICE && ref==0);
  int   blocksize_x     = input->blc_size[blocktype][0];
  int   blocksize_y     = input->blc_size[blocktype][1];
  int   pic4_pix_x      = (pic_pix_x << 2);
  int   pic4_pix_y      = (pic_pix_y << 2);
  int   max_pos_x4      = ((ref_picture->size_x/*img->width*/-blocksize_x+1)<<2);
  int   max_pos_y4      = ((ref_picture->size_y/*img->height*/-blocksize_y+1)<<2);
  
  int   min_pos2        = (input->hadamard ? 0 : 1);
  int   max_pos2        = (input->hadamard ? max(1,search_pos2) : search_pos2);
  int   search_range_dynamic,iXMinNow,iYMinNow,i;
  int   iSADLayer,m,currmv_x,currmv_y,iCurrSearchRange;
  int   search_range = input->search_range;
  int   pred_frac_mv_x,pred_frac_mv_y,abort_search;
  int   mv_cost; 
  
  int   pred_frac_up_mv_x, pred_frac_up_mv_y;
  
  *mv_x <<= 2;
  *mv_y <<= 2;
  if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 2) &&
    (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 2)   )
  {
    PelY_14 = FastPelY_14;
  }
  else
  {
    PelY_14 = UMVPelY_14;
  }
  
  search_range_dynamic = 3;
  pred_frac_mv_x = (pred_mv_x - *mv_x)%4;
  pred_frac_mv_y = (pred_mv_y - *mv_y)%4; 
  
  pred_frac_up_mv_x = (pred_MV_uplayer[0] - *mv_x)%4;
  pred_frac_up_mv_y = (pred_MV_uplayer[1] - *mv_y)%4;
  
  
  memset(SearchState[0],0,(2*search_range_dynamic+1)*(2*search_range_dynamic+1));
  
  if(input->hadamard)
  {
    cand_mv_x = *mv_x;    
    cand_mv_y = *mv_y;    
    mv_cost = MV_COST (lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);    
    mcost = AddUpSADQuarter(pic_pix_x,pic_pix_y,blocksize_x,blocksize_y,cand_mv_x,cand_mv_y,ref_picture/*ref_pic*//*Wenfang Fu 2004.3.12*/,orig_pic,mv_cost,min_mcost,useABT);
    SearchState[search_range_dynamic][search_range_dynamic] = 1;
    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      currmv_x = cand_mv_x;
      currmv_y = cand_mv_y; 
    }
  }
  else
  {
    SearchState[search_range_dynamic][search_range_dynamic] = 1;
    currmv_x = *mv_x;
    currmv_y = *mv_y; 
  }
  
  if(pred_frac_mv_x!=0 || pred_frac_mv_y!=0)
  {
    cand_mv_x = *mv_x + pred_frac_mv_x;    
    cand_mv_y = *mv_y + pred_frac_mv_y;    
    mv_cost = MV_COST (lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);    
    mcost = AddUpSADQuarter(pic_pix_x,pic_pix_y,blocksize_x,blocksize_y,cand_mv_x,cand_mv_y,ref_picture/*ref_pic*//*Wenfang Fu 2004.3.12*/,orig_pic,mv_cost,min_mcost,useABT);
    SearchState[cand_mv_y -*mv_y + search_range_dynamic][cand_mv_x - *mv_x + search_range_dynamic] = 1;
    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      currmv_x = cand_mv_x;
      currmv_y = cand_mv_y; 
    }
  }
  
  
  iXMinNow = currmv_x;
  iYMinNow = currmv_y;
  iCurrSearchRange = 2*search_range_dynamic+1; 
  for(i=0;i<iCurrSearchRange;i++) 
  {
    abort_search=1;
    iSADLayer = 65536;
    for (m = 0; m < 4; m++)
    {
      cand_mv_x = iXMinNow + Diamond_x[m];    
      cand_mv_y = iYMinNow + Diamond_y[m]; 
      
      if(abs(cand_mv_x - *mv_x) <=search_range_dynamic && abs(cand_mv_y - *mv_y)<= search_range_dynamic)
      {
        if(!SearchState[cand_mv_y -*mv_y+ search_range_dynamic][cand_mv_x -*mv_x+ search_range_dynamic])
        {
          mv_cost = MV_COST (lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);    
		      mcost = AddUpSADQuarter(pic_pix_x,pic_pix_y,blocksize_x,blocksize_y,cand_mv_x,cand_mv_y,ref_picture/*ref_pic*//*Wenfang Fu 2004.3.12*/,orig_pic,mv_cost,min_mcost,useABT);
          SearchState[cand_mv_y - *mv_y + search_range_dynamic][cand_mv_x - *mv_x + search_range_dynamic] = 1;
          if (mcost < min_mcost)
          {
            min_mcost = mcost;
            currmv_x = cand_mv_x;
            currmv_y = cand_mv_y; 
            abort_search = 0; 
            
          }
        }
      }
    }
    iXMinNow = currmv_x;
    iYMinNow = currmv_y;
    if(abort_search)
      break;
  }
  
  *mv_x = currmv_x;
  *mv_y = currmv_y;
  
  //===== return minimum motion cost =====
  return min_mcost;
}

/*!
************************************************************************
* \brief
* Functions for SAD prediction of intra block cases.
* 1. void   decide_intrabk_SAD() judges the block coding type(intra/inter) 
*    of neibouring blocks
 * 2. void skip_intrabk_SAD() set the SAD to zero if neigouring block coding 
 *    type is intra
  * \date   : 2003.4
 ************************************************************************
 */
void   decide_intrabk_SAD()
{
  if (img->type != 0)
  {
    if (img->pix_x == 0 && img->pix_y == 0)
    {
      flag_intra_SAD = 0;
    }
    else if (img->pix_x == 0)
    {
      flag_intra_SAD = flag_intra[(img->pix_x)>>4];
    }
    else if (img->pix_y == 0)
    {
      flag_intra_SAD = flag_intra[((img->pix_x)>>4)-1];
    }
    else 
    {
      flag_intra_SAD = ((flag_intra[(img->pix_x)>>4])||(flag_intra[((img->pix_x)>>4)-1])||(flag_intra[((img->pix_x)>>4)+1])) ;
    }
  }
  return;
}

void skip_intrabk_SAD(int best_mode, int ref_max)
{
  int i,j,k, ref;
  if (img->number > 0) 
    flag_intra[(img->pix_x)>>4] = (best_mode == 9 || best_mode == 10) ? 1:0;
  if (img->type!=0  && (best_mode == 9 || best_mode == 10))
  {
    for (i=0; i < 4; i++)
    {
      for (j=0; j < 4; j++)
      {
        for (k=1; k < 8;k++)
        {
          for (ref=0; ref<ref_max;ref++)
          {
            all_mincost[(img->pix_x>>2)+i][(img->pix_y>>2)+j][ref][k][0] = 0;   
          }
        }
      }
    }
  
  }
  return;
}

