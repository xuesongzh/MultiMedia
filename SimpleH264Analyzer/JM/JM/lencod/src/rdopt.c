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
 ***************************************************************************
 * \file rdopt.c
 *
 * \brief
 *    Rate-Distortion optimized mode decision
 *
 * \author
 *    Heiko Schwarz <hschwarz@hhi.de>
 *
 * \date
 *    12. April 2001
 **************************************************************************
 */

#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include <assert.h>
#include "rdopt_coding_state.h"
#include "elements.h"
#include "refbuf.h"
#include "intrarefresh.h"
#include "vlc.h"
#include "mbuffer.h"
#include "image.h"
#include "mb_access.h"
#include "fast_me.h"
#include "ratectl.h"            // head file for rate control
#include "cabac.h"            // head file for rate control

//Rate control

int QP,QP2;
int DELTA_QP,DELTA_QP2;
int diffy[16][16];
static int pred[16][16];

extern       int  QP2QUANT  [40];

//==== MODULE PARAMETERS ====
int   best_mode;
int   rec_mbY[16][16], rec_mbU[8][8], rec_mbV[8][8], rec_mbY8x8[16][16];    // reconstruction values
int   mpr8x8[16][16];
int   ****cofAC=NULL, ****cofAC8x8=NULL;        // [8x8block][4x4block][level/run][scan_pos]
int   ***cofDC=NULL;                       // [yuv][level/run][scan_pos]
int   **cofAC4x4=NULL, ****cofAC4x4intern=NULL; // [level/run][scan_pos]
int   cbp, cbp8x8, cnt_nonz_8x8;
int   cbp_blk, cbp_blk8x8;
int   frefframe[4][4], brefframe[4][4], b8mode[4], b8pdir[4];
int   best8x8mode [4];                // [block]
int   best8x8pdir [MAXMODE][4];       // [mode][block]
int   best8x8fwref  [MAXMODE][4];       // [mode][block]
int   b8_ipredmode[16], b8_intra_pred_modes[16];
CSptr cs_mb=NULL, cs_b8=NULL, cs_cm=NULL, cs_imb=NULL, cs_ib8=NULL, cs_ib4=NULL, cs_pc=NULL;
int   best_c_imode;
int   best_i16offset;

int   best8x8bwref     [MAXMODE][4];       // [mode][block]
int   abp_typeframe[4][4];

/*!
 ************************************************************************
 * \brief
 *    delete structure for RD-optimized mode decision
 ************************************************************************
 */
void clear_rdopt ()
{
  free_mem_DCcoeff (cofDC);
  free_mem_ACcoeff (cofAC);
  free_mem_ACcoeff (cofAC8x8);
  free_mem_ACcoeff (cofAC4x4intern);

  // structure for saving the coding state
  delete_coding_state (cs_mb);
  delete_coding_state (cs_b8);
  delete_coding_state (cs_cm);
  delete_coding_state (cs_imb);
  delete_coding_state (cs_ib8);
  delete_coding_state (cs_ib4);
  delete_coding_state (cs_pc);
}


/*!
 ************************************************************************
 * \brief
 *    create structure for RD-optimized mode decision
 ************************************************************************
 */
void init_rdopt ()
{
  get_mem_DCcoeff (&cofDC);
  get_mem_ACcoeff (&cofAC);
  get_mem_ACcoeff (&cofAC8x8);
  get_mem_ACcoeff (&cofAC4x4intern);
  cofAC4x4 = cofAC4x4intern[0][0];

  // structure for saving the coding state
  cs_mb  = create_coding_state ();
  cs_b8  = create_coding_state ();
  cs_cm  = create_coding_state ();
  cs_imb = create_coding_state ();
  cs_ib8 = create_coding_state ();
  cs_ib4 = create_coding_state ();
  cs_pc  = create_coding_state ();
}



/*! 
 *************************************************************************************
 * \brief
 *    Updates the pixel map that shows, which reference frames are reliable for
 *    each MB-area of the picture.
 *
 * \note
 *    The new values of the pixel_map are taken from the temporary buffer refresh_map
 *
 *************************************************************************************
 */
void UpdatePixelMap()
{
  int mx,my,y,x,i,j;
  if (img->type==I_SLICE)
  {
    for (y=0; y<img->height; y++)
    for (x=0; x<img->width; x++)
    {
      pixel_map[y][x]=1;
    }
  }
  else
  {
    for (my=0; my<img->height/8; my++)
    for (mx=0; mx<img->width/8;  mx++)
    {
      j = my*8 + 8;
      i = mx*8 + 8;
      if (refresh_map[my][mx])
      {
        for (y=my*8; y<j; y++)
        for (x=mx*8; x<i; x++)  pixel_map[y][x] = 1;
      }
      else
      {
        for (y=my*8; y<j; y++)
        for (x=mx*8; x<i; x++)  pixel_map[y][x] = min(pixel_map[y][x]+1, input->num_reference_frames+1);
      }
    }
  }
}

/*! 
 *************************************************************************************
 * \brief
 *    Checks if a given reference frame is reliable for the current 
 *    macroblock, given the motion vectors that the motion search has 
 *    returned.
 *
 * \return
 *    If the return value is 1, the reference frame is reliable. If it 
 *    is 0, then it is not reliable.
 *
 * \note
 *    A specific area in each reference frame is assumed to be unreliable
 *    if the same area has been intra-refreshed in a subsequent frame.
 *    The information about intra-refreshed areas is kept in the pixel_map.
 *
 *************************************************************************************
 */
int CheckReliabilityOfRef (int block, int list_idx, int ref, int mode)
{
  int y,x, block_y, block_x, dy, dx, y_pos, x_pos, yy, xx, pres_x, pres_y;
  int maxold_x  = img->width-1;
  int maxold_y  = img->height-1;
  int ref_frame = ref+1;

  int by0 = (mode>=4?2*(block/2):mode==2?2*block:0);
  int by1 = by0 + (mode>=4||mode==2?2:4);
  int bx0 = (mode>=4?2*(block%2):mode==3?2*block:0);
  int bx1 = bx0 + (mode>=4||mode==3?2:4);

  for (block_y=by0; block_y<by1; block_y++)
    for (block_x=bx0; block_x<bx1; block_x++)
    {
      y_pos  = img->all_mv[block_x][block_y][list_idx][ref][mode][1];
      y_pos += (img->block_y+block_y) * BLOCK_SIZE * 4;
      x_pos  = img->all_mv[block_x][block_y][list_idx][ref][mode][0];
      x_pos += (img->block_x+block_x) * BLOCK_SIZE * 4;
      
      /* Here we specify which pixels of the reference frame influence
         the reference values and check their reliability. This is
         based on the function Get_Reference_Pixel */
      
      dy = y_pos & 3;
      dx = x_pos & 3;

      y_pos = (y_pos-dy)/4;
      x_pos = (x_pos-dx)/4;

      if (dy==0 && dx==0) //full-pel
      {
        for (y=0 ; y < BLOCK_SIZE ; y++)
          for (x=0 ; x < BLOCK_SIZE ; x++)
            if (pixel_map[max(0,min(maxold_y,y_pos+y))][max(0,min(maxold_x,x_pos+x))] < ref_frame)
              return 0;
      }
      else  /* other positions */
      {
        if (dy == 0) 
        {
          for (y=0 ; y < BLOCK_SIZE ; y++)
            for (x=0 ; x < BLOCK_SIZE ; x++)
            {
              pres_y = max(0,min(maxold_y,y_pos+y));
              for(xx=-2;xx<4;xx++) {
                pres_x = max(0,min(maxold_x,x_pos+x+xx));
                if (pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }
            }
        }

        else if (dx == 0) 
        {
          for (y=0 ; y < BLOCK_SIZE ; y++)
            for (x=0 ; x < BLOCK_SIZE ; x++)
            {
              pres_x = max(0,min(maxold_x,x_pos+x));
              for(yy=-2;yy<4;yy++) {
                pres_y = max(0,min(maxold_y,y_pos+yy+y));
                if (pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }
            }
        }
        else if (dx == 2) 
        {
          for (y=0 ; y < BLOCK_SIZE ; y++)
            for (x=0 ; x < BLOCK_SIZE ; x++)
            {
              for(yy=-2;yy<4;yy++) {
                pres_y = max(0,min(maxold_y,y_pos+yy+y));
                for(xx=-2;xx<4;xx++) {
                  pres_x = max(0,min(maxold_x,x_pos+xx+x));
                  if (pixel_map[pres_y][pres_x] < ref_frame)
                    return 0;
                }
              }
            }
        }
        else if (dy == 2) 
        {
          for (y=0 ; y < BLOCK_SIZE ; y++)
            for (x=0 ; x < BLOCK_SIZE ; x++)
            {
              for(xx=-2;xx<4;xx++) {
                pres_x = max(0,min(maxold_x,x_pos+xx+x));
                for(yy=-2;yy<4;yy++) {
                  pres_y = max(0,min(maxold_y,y_pos+yy+y));
                  if (pixel_map[pres_y][pres_x] < ref_frame)
                    return 0;
                }
              }
            }
        }
        else 
        {
          for (y=0 ; y < BLOCK_SIZE ; y++)
            for (x=0 ; x < BLOCK_SIZE ; x++)
            {
              pres_y = dy == 1 ? y_pos+y : y_pos+y+1;
              pres_y = max(0,min(maxold_y,pres_y));

              for(xx=-2;xx<4;xx++) {
                pres_x = max(0,min(maxold_x,x_pos+xx+x));
                if (pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }

              pres_x = dx == 1 ? x_pos+x : x_pos+x+1;
              pres_x = max(0,min(maxold_x,pres_x));

              for(yy=-2;yy<4;yy++) {
                pres_y = max(0,min(maxold_y,y_pos+yy+y));
                if (pixel_map[pres_y][pres_x] < ref_frame)
                  return 0;
              }
            }
        }

      }
    }

  return 1;
}



/*! 
 *************************************************************************************
 * \brief
 *    R-D Cost for an 4x4 Intra block
 *************************************************************************************
 */
double RDCost_for_4x4IntraBlocks (int*    nonzero,
                           int     b8,
                           int     b4,
                           int    ipmode,
                           double  lambda,
                           double  min_rdcost,
                           int mostProbableMode)
{
  double  rdcost;
  int     dummy, x, y, rate;
  int     distortion  = 0;
  int     block_x     = 8*(b8%2)+4*(b4%2);
  int     block_y     = 8*(b8/2)+4*(b4/2);
  int     pic_pix_x   = img->pix_x+block_x;
  int     pic_pix_y   = img->pix_y+block_y;
  int     pic_opix_y  = img->opix_y+block_y;
  byte    **imgY      = enc_picture->imgY;

  Slice          *currSlice    =  img->currentSlice;
  Macroblock     *currMB       = &img->mb_data[img->current_mb_nr];
  SyntaxElement  *currSE       = &img->MB_SyntaxElements[currMB->currSEnr];
  const int      *partMap      = assignSE2partition[input->partition_mode];
  DataPartition  *dataPart;

  //===== perform DCT, Q, IQ, IDCT, Reconstruction =====
  dummy = 0;
  *nonzero = dct_luma (block_x, block_y, &dummy, 1);

  //===== get distortion (SSD) of 4x4 block =====
  for (y=0; y<4; y++)
  for (x=pic_pix_x; x<pic_pix_x+4; x++)  
    distortion += img->quad [imgY_org[pic_opix_y+y][x] - imgY[pic_pix_y+y][x]];

  //===== RATE for INTRA PREDICTION MODE  (SYMBOL MODE MUST BE SET TO UVLC) =====
  currSE->value1 = (mostProbableMode == ipmode) ? -1 : ipmode < mostProbableMode ? ipmode : ipmode-1;

  //--- set position and type ---
  currSE->context = 4*b8 + b4;
  currSE->type    = SE_INTRAPREDMODE;

  //--- set function pointer ----
  if (input->symbol_mode != UVLC)    
  currSE->writing = writeIntraPredMode_CABAC;

  //--- choose data partition ---
  dataPart = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);
  //--- encode and update rate ---
  if (input->symbol_mode == UVLC)    writeSyntaxElement_Intra4x4PredictionMode(currSE, dataPart);
  else                               dataPart->writeSyntaxElement (currSE, dataPart);

  rate = currSE->len;
  currSE++;
  currMB->currSEnr++;

  //===== RATE for LUMINANCE COEFFICIENTS =====
  if (input->symbol_mode == UVLC)
  {
    rate  += writeCoeff4x4_CAVLC (LUMA, b8, b4, 0);
  }
  else
  {
    rate  += writeLumaCoeff4x4_CABAC (b8, b4, 1);
  }
  rdcost = (double)distortion + lambda*(double)rate;

  return rdcost;
}

/*! 
 *************************************************************************************
 * \brief
 *    Mode Decision for an 4x4 Intra block
 *************************************************************************************
 */
int Mode_Decision_for_4x4IntraBlocks (int  b8,  int  b4,  double  lambda,  int*  min_cost)
{
  int     ipmode, best_ipmode = 0, i, j, k, x, y, cost, dummy;
  int     c_nz, nonzero = 0, rec4x4[4][4], diff[16];
  double  rdcost;
  int     block_x     = 8*(b8%2)+4*(b4%2);
  int     block_y     = 8*(b8/2)+4*(b4/2);
  int     pic_pix_x   = img->pix_x+block_x;
  int     pic_pix_y   = img->pix_y+block_y;
  int     pic_opix_x   = img->opix_x+block_x;
  int     pic_opix_y   = img->opix_y+block_y;
  int     pic_block_x = pic_pix_x/4;
  int     pic_block_y = pic_pix_y/4;
  double  min_rdcost  = 1e30;

  int left_available, up_available, all_available;

  int     upMode;
  int     leftMode;
  int     mostProbableMode;

  PixelPos left_block;
  PixelPos top_block;

  getLuma4x4Neighbour(img->current_mb_nr, block_x/4, block_y/4, -1,  0, &left_block);
  getLuma4x4Neighbour(img->current_mb_nr, block_x/4, block_y/4,  0, -1, &top_block);


  // constrained intra pred
  if (input->UseConstrainedIntraPred)
  {
    left_block.available = left_block.available ? img->intra_block[left_block.mb_addr] : 0;
    top_block.available  = top_block.available  ? img->intra_block[top_block.mb_addr]  : 0;
  }
  
  upMode            = top_block.available ? img->ipredmode[top_block.pos_x ][top_block.pos_y ] : -1;
  leftMode          = left_block.available ? img->ipredmode[left_block.pos_x][left_block.pos_y] : -1;
  
  mostProbableMode  = (upMode < 0 || leftMode < 0) ? DC_PRED : upMode < leftMode ? upMode : leftMode;

  *min_cost = (1<<20);

  //===== INTRA PREDICTION FOR 4x4 BLOCK =====
  intrapred_luma (pic_pix_x, pic_pix_y, &left_available, &up_available, &all_available);

  //===== LOOP OVER ALL 4x4 INTRA PREDICTION MODES =====
  for (ipmode=0; ipmode<NO_INTRA_PMODE; ipmode++)
  {
    int available_mode =  (ipmode==DC_PRED) ||
        ((ipmode==VERT_PRED||ipmode==VERT_LEFT_PRED||ipmode==DIAG_DOWN_LEFT_PRED) && up_available ) ||
        ((ipmode==HOR_PRED||ipmode==HOR_UP_PRED) && left_available ) ||(all_available);
    if( available_mode)
    {
      if (!input->rdopt)
      {
        for (k=j=0; j<4; j++)
          for (i=0; i<4; i++, k++)
          {
            diff[k] = imgY_org[pic_opix_y+j][pic_opix_x+i] - img->mprr[ipmode][j][i];
          }
        cost  = (ipmode == mostProbableMode) ? 0 : (int)floor(4 * lambda );
        cost += SATD (diff, input->hadamard);
        if (cost < *min_cost)
        {
          best_ipmode = ipmode;
          *min_cost   = cost;
        }
      }
      else
      {
        // get prediction and prediction error
        for (j=0; j<4; j++)
        for (i=0; i<4; i++)
        {
          img->mpr[block_x+i][block_y+j]  = img->mprr[ipmode][j][i];
          img->m7[i][j]                   = imgY_org[pic_opix_y+j][pic_opix_x+i] - img->mprr[ipmode][j][i];
        }

        //===== store the coding state =====
        store_coding_state (cs_cm);
        // get and check rate-distortion cost
        if ((rdcost = RDCost_for_4x4IntraBlocks (&c_nz, b8, b4, ipmode, lambda, min_rdcost, mostProbableMode)) < min_rdcost)
        {
          //--- set coefficients ---
          for (j=0; j<2; j++)
          for (i=0; i<18;i++)  cofAC4x4[j][i]=img->cofAC[b8][b4][j][i];

          //--- set reconstruction ---
          for (y=0; y<4; y++)
          for (x=0; x<4; x++)  rec4x4[y][x] = enc_picture->imgY[pic_pix_y+y][pic_pix_x+x];

          //--- flag if dct-coefficients must be coded ---
          nonzero = c_nz;

          //--- set best mode update minimum cost ---
          min_rdcost  = rdcost;
          best_ipmode = ipmode;
        }
        reset_coding_state (cs_cm);
      }
    }
  }

  //===== set intra mode prediction =====
  img->ipredmode[pic_block_x][pic_block_y] = best_ipmode;
  img->mb_data[img->current_mb_nr].intra_pred_modes[4*b8+b4] = mostProbableMode == best_ipmode ? -1 : best_ipmode < mostProbableMode ? best_ipmode : best_ipmode-1;

  if (!input->rdopt)
  {
    // get prediction and prediction error
    for (j=0; j<4; j++)
      for (i=0; i<4; i++)
      {
        img->mpr[block_x+i][block_y+j]  = img->mprr[best_ipmode][j][i];
        img->m7[i][j]                   = imgY_org[pic_opix_y+j][pic_opix_x+i] - img->mprr[best_ipmode][j][i];
      }
    nonzero = dct_luma (block_x, block_y, &dummy, 1);
  }
  else
  {
    //===== restore coefficients =====
    for (j=0; j<2; j++)
    for (i=0; i<18;i++)  img->cofAC[b8][b4][j][i]=cofAC4x4[j][i];
  
    //===== restore reconstruction and prediction (needed if single coeffs are removed) =====
    for (y=0; y<4; y++)
    for (x=0; x<4; x++)
    {
      enc_picture->imgY[pic_pix_y+y][pic_pix_x+x] = rec4x4[y][x];
      img->mpr[block_x+x][block_y+y] = img->mprr[best_ipmode][y][x];
    }
  }

  return nonzero;
}


/*! 
 *************************************************************************************
 * \brief
 *    Mode Decision for an 8x8 Intra block
 *************************************************************************************
 */
int Mode_Decision_for_8x8IntraBlocks(int b8,double lambda,int *cost)
{
  int  nonzero=0, b4;
  int  cost4x4;
  
  *cost = (int)floor(6.0 * lambda + 0.4999);

  for (b4=0; b4<4; b4++)
  {
    if (Mode_Decision_for_4x4IntraBlocks (b8, b4, lambda, &cost4x4))
    {
      nonzero        = 1;
    }
    *cost += cost4x4;
  }

  return nonzero;
}

/*! 
 *************************************************************************************
 * \brief
 *    4x4 Intra mode decision for an macroblock
 *************************************************************************************
 */
int Mode_Decision_for_Intra4x4Macroblock (double lambda,  int* cost)

{
  int  cbp=0, b8, cost8x8;

  for (*cost=0, b8=0; b8<4; b8++)
  {
    if (Mode_Decision_for_8x8IntraBlocks (b8, lambda, &cost8x8))
    {
      cbp |= (1<<b8);
    }
    *cost += cost8x8;
  }

  return cbp;
}


/*! 
 *************************************************************************************
 * \brief
 *    R-D Cost for an 8x8 Partition
 *************************************************************************************
 */
double RDCost_for_8x8blocks (int*    cnt_nonz,   // --> number of nonzero coefficients
                             int*    cbp_blk,    // --> cbp blk
                             double  lambda,     // <-- lagrange multiplier
                             int     block,      // <-- 8x8 block number
                             int     mode,       // <-- partitioning mode
                             int     pdir,       // <-- prediction direction
                             int     ref,        // <-- reference frame
                             int     bwd_ref)    // <-- abp type
{
  int  i, j, k;
  int  rate=0, distortion=0;
  int  dummy, mrate;
  int  fw_mode, bw_mode;
  int  cbp     = 0;
  int  pax     = 8*(block%2);
  int  pay     = 8*(block/2);
  int  i0      = pax/4;
  int  j0      = pay/4;
  int  bframe  = (img->type==B_SLICE);
  int  direct  = (bframe && mode==0);
  int  b8value = B8Mode2Value (mode, pdir);

  Macroblock    *currMB    = &img->mb_data[img->current_mb_nr];
  SyntaxElement *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
  Slice         *currSlice = img->currentSlice;
  DataPartition *dataPart;
  const int     *partMap   = assignSE2partition[input->partition_mode];

  EncodingEnvironmentPtr eep_dp;

  //=====
  //=====  GET COEFFICIENTS, RECONSTRUCTIONS, CBP
  //=====
  if (direct)
  {
    if (direct_pdir[img->block_x+i0][img->block_y+j0]<0) // mode not allowed
    {
      return (1e20);
    }
    else
    {
      *cnt_nonz = LumaResidualCoding8x8 (&cbp, cbp_blk, block, direct_pdir[img->block_x+i0][img->block_y+j0], 0, 0, max(0,direct_ref_idx[LIST_0][img->block_x+i0][img->block_y+j0]), direct_ref_idx[LIST_1][img->block_x+i0][img->block_y+j0]);
    }
  }
  else
  {
    fw_mode   = (pdir==0||pdir==2 ? mode : 0);
    bw_mode   = (pdir==1||pdir==2 ? mode : 0);
    *cnt_nonz = LumaResidualCoding8x8 (&cbp, cbp_blk, block, pdir, fw_mode, bw_mode, ref, bwd_ref);
  }

  //===== get residue =====
  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    // We need the reconstructed prediction residue for the simulated decoders.
    compute_residue_b8block (block, -1);
  }

  //=====
  //=====   GET DISTORTION
  //=====
  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    for (k=0; k<input->NoOfDecoders ;k++)
    {
      decode_one_b8block (k, P8x8, block, mode, ref);
      for (j=img->opix_y+pay; j<img->opix_y+pay+8; j++)
      for (i=img->opix_x+pax; i<img->opix_x+pax+8; i++)
      {
        distortion += img->quad[imgY_org[j][i] - decs->decY[k][j][i]];
      }
    }
    distortion /= input->NoOfDecoders;
  }
  else
  {
    for (j=pay; j<pay+8; j++)
    for (i=img->pix_x+pax; i<img->pix_x+pax+8; i++)
    {
      distortion += img->quad [imgY_org[img->opix_y+j][i] - enc_picture->imgY[img->pix_y+j][i]];
    }
  }

  //=====
  //=====   GET RATE
  //=====
  //----- block 8x8 mode -----
  if (input->symbol_mode == UVLC)
  {
    ue_linfo (b8value, dummy, &mrate, &dummy);
    rate += mrate;
  }
  else
  {
    currSE->value1  = b8value;
    currSE->writing = writeB8_typeInfo_CABAC;
    currSE->type    = SE_MBTYPE;
    dataPart = &(currSlice->partArr[partMap[currSE->type]]);
    dataPart->writeSyntaxElement (currSE, dataPart);
    rate += currSE->len;
    currSE++;
    currMB->currSEnr++;
  }

  //----- motion information -----
  if (!direct)
  {
    if ((img->num_ref_idx_l0_active > 1 ) && (pdir==0 || pdir==2))
      rate  += writeReferenceFrame (mode, i0, j0, 1, ref);
    if(img->num_ref_idx_l1_active > 1 && img->type== B_SLICE)
    {
      if (pdir==1 || pdir==2)
      {
        rate  += writeReferenceFrame (mode, i0, j0, 0, bwd_ref);
      }
    }

    if (pdir==0 || pdir==2)
    {
      rate  += writeMotionVector8x8 (i0, j0, i0+2, j0+2, ref, LIST_0, mode);
    }
    if (pdir==1 || pdir==2)
    {
      rate  += writeMotionVector8x8 (i0, j0, i0+2, j0+2, bwd_ref, LIST_1, mode);
    }
  }

  //----- coded block pattern (for CABAC only) -----
  if (input->symbol_mode == CABAC)
  {
    dataPart = &(currSlice->partArr[partMap[SE_CBP_INTER]]);
    eep_dp   = &(dataPart->ee_cabac);
    mrate    = arienco_bits_written (eep_dp);
    writeCBP_BIT_CABAC (block, ((*cnt_nonz>0)?1:0), cbp8x8, currMB, 1, eep_dp);
    mrate    = arienco_bits_written (eep_dp) - mrate;
    rate    += mrate;
  }

  //----- luminance coefficients -----
  if (*cnt_nonz)
  {
    rate += writeLumaCoeff8x8 (block, 0);
  }

  return (double)distortion + lambda * (double)rate;
}


/*! 
 *************************************************************************************
 * \brief
 *    Gets mode offset for intra16x16 mode
 *************************************************************************************
 */
int I16Offset (int cbp, int i16mode)
{
  return (cbp&15?13:1) + i16mode + ((cbp&0x30)>>2);
}


/*! 
 *************************************************************************************
 * \brief
 *    Sets modes and reference frames for an macroblock
 *************************************************************************************
 */
void SetModesAndRefframeForBlocks (int mode)
{
  int i,j,k,l;
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];
  int  bframe  = (img->type==B_SLICE);

  int list_offset   = ((img->MbaffFrameFlag)&&(currMB->mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  //--- macroblock type ---
  currMB->mb_type = mode;
  
  //--- block 8x8 mode and prediction direction ---
  switch (mode)
  {
  case 0:
    for(i=0;i<4;i++)
    {
      currMB->b8mode[i] = 0;
      currMB->b8pdir[i] = (bframe?direct_pdir[img->block_x+(i%2)*2][img->block_y+(i/2)*2]:0);
    }
    break;
  case 1:
  case 2:
  case 3:
    for(i=0;i<4;i++)
    {
      currMB->b8mode[i] = mode;
      currMB->b8pdir[i] = best8x8pdir[mode][i];
    }
    break;
  case P8x8:
    for(i=0;i<4;i++)
    {
      currMB->b8mode[i]   = best8x8mode[i];
      currMB->b8pdir[i]   = best8x8pdir[mode][i];
    }
    break;
  case I4MB:
    for(i=0;i<4;i++)
    {
      currMB->b8mode[i] = IBLOCK;
      currMB->b8pdir[i] = -1;
    }
    break;
  case I16MB:
    for(i=0;i<4;i++)
    {
      currMB->b8mode[i] =  0;
      currMB->b8pdir[i] = -1;
    }
    break;
  default:
    printf ("Unsupported mode in SetModesAndRefframeForBlocks!\n");
    exit (1);
  }
  
#define IS_FW ((best8x8pdir[mode][k]==0 || best8x8pdir[mode][k]==2) && (mode!=P8x8 || best8x8mode[k]!=0 || !bframe))
#define IS_BW ((best8x8pdir[mode][k]==1 || best8x8pdir[mode][k]==2) && (mode!=P8x8 || best8x8mode[k]!=0))
  //--- reference frame arrays ---
  if (mode==0 || mode==I4MB || mode==I16MB)
  {
    if (bframe)
    { 
      for (j=0;j<4;j++)
        for (i=0;i<4;i++)
        {
          if(!mode)
          {     //direct mode
            enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = direct_ref_idx[LIST_0][img->block_x+i][img->block_y+j];
            enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = direct_ref_idx[LIST_1][img->block_x+i][img->block_y+j];
          }
          else
          {   //intra
            enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = -1;
            enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = -1;
            
          }
        }
    }
    else
    {
      for (j=0;j<4;j++)
        for (i=0;i<4;i++)
        {
          enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = (mode==0?0:-1);
        }
    }
  }
  else
  {
    if (bframe)
    {
      for (j=0;j<4;j++)
        for (i=0;i<4;i++)
        {
          k = 2*(j/2)+(i/2);
          l = 2*(j%2)+(i%2);
          
          if(mode == P8x8 && best8x8mode[k]==0)
          {           
            enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = direct_ref_idx[LIST_0][img->block_x+i][img->block_y+j];
            enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = direct_ref_idx[LIST_1][img->block_x+i][img->block_y+j];
          }
          else
          {
            enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = (IS_FW ? best8x8fwref[mode][k] : -1);
            enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = (IS_BW ? best8x8bwref[mode][k] : -1);
          }
        }
    }
    else
    {
      for (j=0;j<4;j++)
        for (i=0;i<4;i++)
        {
          k = 2*(j/2)+(i/2);
          l = 2*(j%2)+(i%2);
          enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = (IS_FW ? best8x8fwref[mode][k] : -1);
        }
    }
  }

  for (j=0;j<4;j++)
  {
    for (i=0;i<4;i++)
    {
      enc_picture->ref_pic_id [LIST_0][img->block_x+i][img->block_y+j] = 
        (enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]>=0 ? 
         enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]]:
         -1);
    }
  }
  if (bframe)
  {
    for (j=0;j<4;j++)
    {
      for (i=0;i<4;i++)
      {
        enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+j] = 
          (enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j]>=0 ? 
           enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j]]:
           -1);
      }
      
    }
  }


#undef IS_FW
#undef IS_BW
}


/*! 
 *************************************************************************************
 * \brief
 *    Intra 16x16 mode decision
 *************************************************************************************
 */
void
Intra16x16_Mode_Decision (Macroblock* currMB, int* i16mode)
{
  intrapred_luma_16x16 ();   /* make intra pred for all 4 new modes */
  find_sad_16x16 (i16mode);   /* get best new intra mode */
  currMB->cbp = dct_luma_16x16 (*i16mode);
}



/*! 
 *************************************************************************************
 * \brief
 *    Sets Coefficients and reconstruction for an 8x8 block
 *************************************************************************************
 */
void SetCoeffAndReconstruction8x8 (Macroblock* currMB)
{
  int block, k, j, i;
  int block_y = img->block_y;
  int **ipredmodes = img->ipredmode;

  //--- restore coefficients ---
  for (block=0; block<6; block++)
  for (k=0; k<4; k++)
  for (j=0; j<2; j++)
  for (i=0; i<65; i++)
    img->cofAC[block][k][j][i] = cofAC8x8[block][k][j][i];

  if (cnt_nonz_8x8<=5 && img->type!=SP_SLICE)
  {
    currMB->cbp     = 0;
    currMB->cbp_blk = 0;
    for (j=0; j<16; j++)
    for (i=0; i<16; i++)  enc_picture->imgY[img->pix_y+j][img->pix_x+i] = mpr8x8[j][i];
  }
  else
  {
    currMB->cbp     = cbp8x8;
    currMB->cbp_blk = cbp_blk8x8;
    for (j=0; j<16; j++)
    for (i=0; i<16; i++)  enc_picture->imgY[img->pix_y+j][img->pix_x+i] = rec_mbY8x8[j][i];
  }

  //===== restore intra prediction modes for 8x8+ macroblock mode =====
  for (k=0, j=block_y; j<block_y+4; j++)
  for (     i=img->block_x; i<img->block_x+4; i++, k++)
  {
    ipredmodes    [i][j] = b8_ipredmode       [k];
    currMB->intra_pred_modes[k] = b8_intra_pred_modes[k];
  }
}


/*! 
 *************************************************************************************
 * \brief
 *    Sets motion vectors for an macroblock
 *************************************************************************************
 */
void SetMotionVectorsMB (Macroblock* currMB, int bframe)
{
  int i, j, k, l, m, mode8, pdir8, ref, by, bx, bxr;
  int ******all_mv  = img->all_mv;
  int ******pred_mv = img->pred_mv;
  int  bw_ref;

  for (j=0; j<4; j++)
    for (i=0; i<4; i++)
    {
      mode8 = currMB->b8mode[k=2*(j/2)+(i/2)];
      l     = 2*(j%2)+(i%2);
      by    = img->block_y+j;
      bxr   = img->block_x+i;
      bx    = img->block_x+i+4;


      
        pdir8 = currMB->b8pdir[k];
        ref    = enc_picture->ref_idx[LIST_0][bxr][by];
        bw_ref = enc_picture->ref_idx[LIST_1][bxr][by];
      
      
      if (!bframe)
      {
        if (pdir8>=0) //(mode8!=IBLOCK)&&(mode8!=I16MB))  // && ref != -1)
        {
          enc_picture->mv[LIST_0][bxr][by][0] = all_mv [i][j][LIST_0][ ref][mode8][0];
          enc_picture->mv[LIST_0][bxr][by][1] = all_mv [i][j][LIST_0][ ref][mode8][1];
        }
        else
        {
          enc_picture->mv[LIST_0][bxr][by][0] = 0;
          enc_picture->mv[LIST_0][bxr][by][1] = 0;
        }
      }
      else
      {
        if (pdir8==-1) // intra
        {
          enc_picture->mv[LIST_0][bxr][by][0] = 0;
          enc_picture->mv[LIST_0][bxr][by][1] = 0;
          enc_picture->mv[LIST_1][bxr][by][0] = 0;
          enc_picture->mv[LIST_1][bxr][by][1] = 0;
        }
        else if (pdir8==0) // forward
        {
          enc_picture->mv[LIST_0][bxr][by][0] = all_mv [i][j][LIST_0][ ref][mode8][0];
          enc_picture->mv[LIST_0][bxr][by][1] = all_mv [i][j][LIST_0][ ref][mode8][1];
          enc_picture->mv[LIST_1][bxr][by][0] = 0;
          enc_picture->mv[LIST_1][bxr][by][1] = 0;
        }
        else if (pdir8==1) // backward
        {
          enc_picture->mv[LIST_0][bxr][by][0] = 0;
          enc_picture->mv[LIST_0][bxr][by][1] = 0;
          
          enc_picture->mv[LIST_1][bxr][by][0] = all_mv[i][j][LIST_1][bw_ref][mode8][0];
          enc_picture->mv[LIST_1][bxr][by][1] = all_mv[i][j][LIST_1][bw_ref][mode8][1];
        }
        else if (pdir8==2) // bidir
        {
          enc_picture->mv[LIST_0][bxr][by][0] = all_mv [i][j][LIST_0][ ref][mode8][0];
          enc_picture->mv[LIST_0][bxr][by][1] = all_mv [i][j][LIST_0][ ref][mode8][1];
          

          enc_picture->mv[LIST_1][bxr][by][0] = all_mv[i][j][LIST_1][bw_ref][mode8][0];
          enc_picture->mv[LIST_1][bxr][by][1] = all_mv[i][j][LIST_1][bw_ref][mode8][1];
        }
        else 
        {
          error("invalid direction mode", 255);
        }
      }
  }
  
  // copy all the motion vectors into rdopt structure
  // Can simplify this by copying the MV's of the best mode (TBD)
  if(img->MbaffFrameFlag)
  {
    for(i=0;i<4;i++)
      for(j=0;j<4;j++)
        for (k=0;k<2;k++)
          for(l=0;l<img->max_num_references;l++)
            for(m=0;m<9;m++)
            {
              rdopt->all_mv [i][j][k][l][m][0]  = all_mv [i][j][k][l][m][0];
              rdopt->pred_mv[i][j][k][l][m][0]  = pred_mv[i][j][k][l][m][0];
              
              rdopt->all_mv [i][j][k][l][m][1]  = all_mv [i][j][k][l][m][1];
              rdopt->pred_mv[i][j][k][l][m][1]  = pred_mv[i][j][k][l][m][1];
            }
  }
}



/*! 
 *************************************************************************************
 * \brief
 *    R-D Cost for a macroblock
 *************************************************************************************
 */
int RDCost_for_macroblocks (double   lambda,      // <-- lagrange multiplier
                        int      mode,        // <-- modus (0-COPY/DIRECT, 1-16x16, 2-16x8, 3-8x16, 4-8x8(+), 5-Intra4x4, 6-Intra16x16)
                        double*  min_rdcost)  // <-> minimum rate-distortion cost
{
  int         i, j, k; //, k, ****ip4;
  int         i16mode, rate=0, distortion=0;
  double      rdcost;
  Macroblock  *currMB   = &img->mb_data[img->current_mb_nr];
  Macroblock  *prevMB   = img->current_mb_nr ? &img->mb_data[img->current_mb_nr-1] : NULL;
  int         bframe    = (img->type==B_SLICE);
  int         tmp_cc;
  int         use_of_cc =  (img->type!=I_SLICE &&  input->symbol_mode!=CABAC);
  int         cc_rate, dummy;
  
  //=====
  //=====  SET REFERENCE FRAMES AND BLOCK MODES
  //=====
  SetModesAndRefframeForBlocks (mode);

  //=====
  //=====  GET COEFFICIENTS, RECONSTRUCTIONS, CBP
  //=====
  if (bframe && mode==0)
  {
    int block_x=img->pix_x>>2;
    int block_y=img->pix_y>>2;
    for (i=0;i<4;i++)
      for (j=0;j<4;j++)
        if (direct_pdir[block_x+i][block_y+j]<0)
          return 0;
  }

  if (mode<P8x8)
  {
    LumaResidualCoding ();
  }
  else if (mode==P8x8)
  {
    SetCoeffAndReconstruction8x8 (currMB);
  }
  else if (mode==I4MB)
  {
    currMB->cbp = Mode_Decision_for_Intra4x4Macroblock (lambda, &dummy);
  }
  else if (mode==I16MB)
  {
    Intra16x16_Mode_Decision  (currMB, &i16mode);
  }

  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    // We need the reconstructed prediction residue for the simulated decoders.
    compute_residue_mb (mode==I16MB?i16mode:-1);
  }

  //Rate control
  if (mode == I16MB)
  {
	  for(i=0; i<16; i++)
	  for(j=0; j<16; j++)
	    pred[j][i] = img->mprr_2[i16mode][j][i];
  }else
  {
	  for(i=0; i<16; i++)
	  for(j=0; j<16; j++)
	    pred[j][i] = img->mpr[i][j];
  }

  img->i16offset = 0;
  dummy = 0;
  ChromaResidualCoding (&dummy);
  if (mode==I16MB)     img->i16offset = I16Offset  (currMB->cbp, i16mode);


  //=====
  //=====   GET DISTORTION
  //=====
  // LUMA
  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    for (k=0; k<input->NoOfDecoders ;k++)
    {
      decode_one_mb (k, currMB);
      for (j=0; j<MB_BLOCK_SIZE; j++)
      for (i=img->opix_x; i<img->opix_x+MB_BLOCK_SIZE; i++)
      {
        distortion += img->quad [imgY_org[img->opix_y+j][i] - decs->decY[k][img->opix_y+j][i]];
      }
    }
    distortion /= input->NoOfDecoders;
  }
  else
  {
    for (j=0; j<16; j++)
    for (i=img->opix_x; i<img->opix_x+16; i++)
    {
      distortion += img->quad [imgY_org[j+img->opix_y][i] - enc_picture->imgY[img->pix_y+j][i]];
    }
  }

  // CHROMA
  for (j=0; j<8; j++)
  for (i=img->opix_c_x; i<img->opix_c_x+8; i++)
  {
    distortion += img->quad [imgUV_org[0][j+img->opix_c_y][i] - enc_picture->imgUV[0][img->pix_c_y+j][i]];
    distortion += img->quad [imgUV_org[1][j+img->opix_c_y][i] - enc_picture->imgUV[1][img->pix_c_y+j][i]];
  }


  //=====   S T O R E   C O D I N G   S T A T E   =====
  //---------------------------------------------------
  store_coding_state (cs_cm);
  

  //=====
  //=====   GET RATE
  //=====
  //----- macroblock header -----
  if (use_of_cc)
  {
    if (currMB->mb_type!=0 || (bframe && currMB->cbp!=0))
    {
      // cod counter and macroblock mode are written ==> do not consider code counter
      tmp_cc = img->cod_counter;
      rate   = writeMBHeader (1); 
      ue_linfo (tmp_cc, dummy, &cc_rate, &dummy);
      rate  -= cc_rate;
      img->cod_counter = tmp_cc;
    }
    else
    {
      // cod counter is just increased  ==> get additional rate
      ue_linfo (img->cod_counter+1, dummy, &rate,    &dummy);
      ue_linfo (img->cod_counter,   dummy, &cc_rate, &dummy);
      rate -= cc_rate;
    }
  }
  else
  {
    rate = writeMBHeader (1); 
  }
  if (mode)
  {
    //----- motion information -----
    rate  += writeMotionInfo2NAL  ();
  }
  if (mode || (bframe && (currMB->cbp!=0 || input->symbol_mode==CABAC)))
  {
    rate  += writeCBPandLumaCoeff ();

    rate  += writeChromaCoeff     ();
  }


  //=====   R E S T O R E   C O D I N G   S T A T E   =====
  //-------------------------------------------------------
  reset_coding_state (cs_cm);


  rdcost = (double)distortion + lambda * (double)rate;

  if (rdcost >= *min_rdcost)
  {
    return 0;
  }


  if ((img->MbaffFrameFlag) && (mode ? 0: ((img->type == B_SLICE) ? !currMB->cbp:1)))  // AFF and current is skip
  {
    if (img->current_mb_nr%2) //bottom
    {
      if (prevMB->mb_type ? 0:((img->type == B_SLICE) ? !prevMB->cbp:1)) //top is skip
      {
        if (!(field_flag_inference() == currMB->mb_field)) //skip only allowed when correct inference
          return 0;
      }
    }
  }

 
  //=====   U P D A T E   M I N I M U M   C O S T   =====
  //-----------------------------------------------------
  *min_rdcost = rdcost;
  return 1;
}





/*! 
 *************************************************************************************
 * \brief
 *    Store macroblock parameters
 *************************************************************************************
 */
void store_macroblock_parameters (int mode)
{
  int  i, j, k, ****i4p, ***i3p;
  Macroblock *currMB  = &img->mb_data[img->current_mb_nr];
  int        bframe   = (img->type==B_SLICE);

  //--- store best mode ---
  best_mode = mode;
  best_c_imode = currMB->c_ipred_mode;
  best_i16offset = img->i16offset;
  for (i=0; i<4; i++)
  {
    b8mode[i]   = currMB->b8mode[i];
    b8pdir[i]   = currMB->b8pdir[i];
  }

  //--- reconstructed blocks ----
  for (j=0; j<16; j++)
  for (i=0; i<16; i++)
  {
    rec_mbY[j][i] = enc_picture->imgY[img->pix_y+j][img->pix_x+i];
  }
  for (j=0; j<8; j++)
  for (i=0; i<8; i++)
  {
    rec_mbU[j][i] = enc_picture->imgUV[0][img->pix_c_y+j][img->pix_c_x+i];
    rec_mbV[j][i] = enc_picture->imgUV[1][img->pix_c_y+j][img->pix_c_x+i];
  }


  //--- store results of decoders ---
  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    for (k=0; k<input->NoOfDecoders; k++)
    {
      for (j=img->pix_y; j<img->pix_y+16; j++)
      for (i=img->pix_x; i<img->pix_x+16; i++)
      {
        // Keep the decoded values of each MB for updating the ref frames
        decs->decY_best[k][j][i] = decs->decY[k][j][i];
      }
    }
  }

  //--- coeff, cbp, kac ---
  if (mode || bframe)
  {
    i4p=cofAC; cofAC=img->cofAC; img->cofAC=i4p;
    i3p=cofDC; cofDC=img->cofDC; img->cofDC=i3p;
    cbp     = currMB->cbp;
    cbp_blk = currMB->cbp_blk;
  }
  else
  {
    cbp = cbp_blk = 0;
  }

  for (j=0; j<4; j++)
  for (i=0; i<4; i++)
  {  
    frefframe[j][i] = enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j];

    if (bframe)
    {
      brefframe[j][i] = enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j];
    }
  }

}


/*! 
 *************************************************************************************
 * \brief
 *    Set stored macroblock parameters
 *************************************************************************************
 */
void set_stored_macroblock_parameters ()
{
  int  i, j, k, ****i4p, ***i3p,l;
  Macroblock  *currMB  = &img->mb_data[img->current_mb_nr];
  int         mode     = best_mode;
  int         bframe   = (img->type==B_SLICE);
  int         **ipredmodes = img->ipredmode;
  
  byte        **imgY  = enc_picture->imgY;
  byte       ***imgUV = enc_picture->imgUV;

  int        list_offset   = ((img->MbaffFrameFlag)&&(currMB->mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  //===== reconstruction values =====
  for (j=0; j<16; j++)
  for (i=0; i<16; i++)
  {
    imgY[img->pix_y+j][img->pix_x+i] = rec_mbY[j][i];

    if(img->MbaffFrameFlag)
      rdopt->rec_mbY[j][i]       = rec_mbY[j][i]; 
  }

  for (j=0; j<8; j++)
  for (i=0; i<8; i++)
  {
    imgUV[0][img->pix_c_y+j][img->pix_c_x+i] = rec_mbU[j][i];
    imgUV[1][img->pix_c_y+j][img->pix_c_x+i] = rec_mbV[j][i];

    if(img->MbaffFrameFlag)
    {
      rdopt->rec_mbU[j][i]           = rec_mbU[j][i];   
      rdopt->rec_mbV[j][i]           = rec_mbV[j][i];   
    }
  }

  //===== coefficients and cbp =====
  i4p=cofAC; cofAC=img->cofAC; img->cofAC=i4p;
  i3p=cofDC; cofDC=img->cofDC; img->cofDC=i3p;
  currMB->cbp      = cbp;
  currMB->cbp_blk = cbp_blk;
  //==== macroblock type ====
  currMB->mb_type = mode;


  if(img->MbaffFrameFlag)
  {
    rdopt->mode = mode;
    rdopt->i16offset = img->i16offset;
    rdopt->cbp = cbp;
    rdopt->cbp_blk = cbp_blk;
    rdopt->mb_type  = mode;

    rdopt->prev_qp=currMB->prev_qp;
    rdopt->prev_delta_qp=currMB->prev_delta_qp;
    rdopt->qp=currMB->qp;

    for(i=0;i<6;i++)
      for(j=0;j<4;j++)
        for(k=0;k<2;k++)
          for(l=0;l<18;l++)
            rdopt->cofAC[i][j][k][l] = img->cofAC[i][j][k][l];

    for(i=0;i<3;i++)
        for(k=0;k<2;k++)
          for(l=0;l<18;l++)
            rdopt->cofDC[i][k][l] = img->cofDC[i][k][l];
  }


  for (i=0; i<4; i++)
  {
    currMB->b8mode[i]   = b8mode[i];
    currMB->b8pdir[i]   = b8pdir[i];
    if(img->MbaffFrameFlag)
    {
      rdopt->b8mode[i]  = b8mode[i];                  
      rdopt->b8pdir[i]  = b8pdir[i];                  
    }
  }

  if (input->rdopt==2 && img->type!=B_SLICE)
  {
    //! save the MB Mode of every macroblock
    decs->dec_mb_mode[img->mb_x][img->mb_y] = mode;
  }

  //==== reference frames =====
  for (j=0; j<4; j++)
  for (i=0; i<4; i++)
  {
    // backward prediction or intra
    if ((currMB->b8pdir[i/2+(j/2)*2] == 1) || IS_INTRA(currMB))
    {
      enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = -1;
      enc_picture->ref_pic_id [LIST_0][img->block_x+i][img->block_y+j] = -1;

      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] =0;
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = 0;
      if(img->MbaffFrameFlag)
        rdopt->refar[LIST_0][j][i] = -1;
    }
    else
    {
      enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = frefframe[j][i]; 
      enc_picture->ref_pic_id [LIST_0][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]];

      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] = img->all_mv[i][j][LIST_0][frefframe[j][i]][currMB->b8mode[i/2+(j/2)*2]][0];
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = img->all_mv[i][j][LIST_0][frefframe[j][i]][currMB->b8mode[i/2+(j/2)*2]][1];
      if(img->MbaffFrameFlag)
        rdopt->refar[LIST_0][j][i] = frefframe[j][i];
    }

    // forward prediction or intra
    if ((currMB->b8pdir[i/2+(j/2)*2] == 0) || IS_INTRA(currMB))
    {
      enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = -1;
      enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+j] = -1;  
      enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] =0;
      enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = 0;
      if(img->MbaffFrameFlag)
        rdopt->refar[LIST_1][j][i] = -1;
    }
    
  }

  if (bframe)
  {
    for (j=0; j<4; j++)
      for (i=0; i<4; i++)
      {
        
        // forward
        if (IS_INTRA(currMB)||(currMB->b8pdir[i/2+(j/2)*2] == 0))
        {
          enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = -1;
          enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+j] = -1;
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] = 0;
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = 0;
          if(img->MbaffFrameFlag)
            rdopt->refar[LIST_1][j][i] = -1;
        }
        else
        {
          enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = brefframe[j][i];
          enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j]];
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] = img->all_mv[i][j][LIST_1][brefframe[j][i]][currMB->b8mode[i/2+(j/2)*2]][0];
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = img->all_mv[i][j][LIST_1][brefframe[j][i]][currMB->b8mode[i/2+(j/2)*2]][1];
          if(img->MbaffFrameFlag)
            rdopt->refar[LIST_1][j][i] = brefframe[j][i];
        }
      }
  }

  //==== intra prediction modes ====
  currMB->c_ipred_mode = best_c_imode;
  img->i16offset = best_i16offset;
  if (mode==P8x8)
  {
    for (k=0, j=img->block_y; j<img->block_y+4; j++)
      for (   i=img->block_x; i<img->block_x+4; i++, k++)
      {
        ipredmodes           [i][j] = b8_ipredmode       [k];
        currMB->intra_pred_modes[k] = b8_intra_pred_modes[k];
      }
  }
  else if (mode!=I4MB)
  {
    for (k=0, j=img->block_y; j<img->block_y+4; j++)
      for (   i=img->block_x; i<img->block_x+4; i++, k++)
      {
        ipredmodes           [i][j] = DC_PRED;
        currMB->intra_pred_modes[k] = DC_PRED;
      }
  }

  if(img->MbaffFrameFlag)
  {
    for (k=0, j=img->block_y; j<img->block_y+4; j++)
      for (   i=img->block_x; i<img->block_x+4; i++, k++)
      {
        rdopt->ipredmode[i][j]     = ipredmodes[i][j];
        rdopt->intra_pred_modes[k] = currMB->intra_pred_modes[k];
      }
    rdopt->c_ipred_mode = currMB->c_ipred_mode;
    rdopt->i16offset = img->i16offset;  // DH
  }

  //==== motion vectors =====
  SetMotionVectorsMB (currMB, bframe);
}



/*! 
 *************************************************************************************
 * \brief
 *    Set reference frames and motion vectors
 *************************************************************************************
 */
void SetRefAndMotionVectors (int block, int mode, int pdir, int fwref, int bwref)
{
  int     i, j=0;
  int     bslice  = (img->type==B_SLICE);
  int     pmode   = (mode==1||mode==2||mode==3?mode:4);
  int     j0      = ((block/2)<<1);
  int     i0      = ((block%2)<<1);
  int     j1      = j0 + (input->blc_size[pmode][1]>>2);
  int     i1      = i0 + (input->blc_size[pmode][0]>>2);

  int list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;


  if (pdir<0)
  {
    for (j=j0; j<j1; j++)
    for (i=i0; i<i1; i++)
    {
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] = 0;
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = 0;
      enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] = 0;
      enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = 0;
      enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = -1;
      enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = -1;
      enc_picture->ref_pic_id[LIST_0][img->block_x+i][img->block_y+j] = -1;
      enc_picture->ref_pic_id[LIST_1][img->block_x+i][img->block_y+j] = -1;
    }
    return;
  }

  if (!bslice)
  {
    for (j=j0; j<j1; j++)
    for (i=i0; i<i1; i++)
    {
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] = img->all_mv[i][j][LIST_0][fwref][mode][0];
      enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = img->all_mv[i][j][LIST_0][fwref][mode][1];
      enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = fwref;
      enc_picture->ref_pic_id[LIST_0][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_0+list_offset][enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]];
    }
  }
  else
  {
    for (j=j0; j<j1; j++)
      for (i=i0; i<i1; i++)
      {
        if (mode==0)
        {
          pdir  =direct_pdir[img->block_x+i][img->block_y+j];
          fwref =direct_ref_idx[LIST_0][img->block_x+i][img->block_y+j];
          bwref =direct_ref_idx[LIST_1][img->block_x+i][img->block_y+j];
        }
        
        if ((pdir==0 || pdir==2))
        {
          enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] = img->all_mv[i][j][LIST_0][fwref][mode][0];
          enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = img->all_mv[i][j][LIST_0][fwref][mode][1];
          enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = fwref;
          enc_picture->ref_pic_id[LIST_0][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_0+list_offset][enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]];
        }
        else
        {
          enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][0] = 0;
          enc_picture->mv[LIST_0][img->block_x+i][img->block_y+j][1] = 0;
          enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j] = -1;
          enc_picture->ref_pic_id[LIST_0][img->block_x+i][img->block_y+j] = -1;
        }

        if ((pdir==1 || pdir==2))
        {
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] = img->all_mv[i][j][LIST_1][bwref][mode][0];
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = img->all_mv[i][j][LIST_1][bwref][mode][1];
          enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = bwref;
          enc_picture->ref_pic_id[LIST_1][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_1+list_offset][enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j]];
        }
        else
        {
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][0] = 0;
          enc_picture->mv[LIST_1][img->block_x+i][img->block_y+j][1] = 0;
          enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j] = -1;
          enc_picture->ref_pic_id[LIST_1][img->block_x+i][img->block_y+j] = -1;
        }
      }
  }
}

/*! 
 *************************************************************************************
 * \brief
 *    skip macroblock field inference
 * \return
 *    inferred field flag
 *************************************************************************************
 */
int field_flag_inference()
{
  int mb_field;
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  if (currMB->mbAvailA)
  {
    mb_field = img->mb_data[currMB->mbAddrA].mb_field;
  }
  else
  {
    // check top macroblock pair
    if (currMB->mbAvailB)
    {
      mb_field = img->mb_data[currMB->mbAddrB].mb_field;
    }
    else
      mb_field = 0;
  }

  return mb_field;
}

/*! 
 *************************************************************************************
 * \brief
 *    Mode Decision for a macroblock
 *************************************************************************************
 */
 void encode_one_macroblock ()
 {
   static const int  b8_mode_table[6]  = {0, 4, 5, 6, 7};         // DO NOT CHANGE ORDER !!!
   static const int  mb_mode_table[7]  = {0, 1, 2, 3, P8x8, I16MB, I4MB}; // DO NOT CHANGE ORDER !!!
   
   int         valid[MAXMODE];
   int         rerun, block, index, mode, i0, i1, j0, j1, pdir, ref, i, j, k, ctr16x16, dummy;
   double      qp, lambda_mode, lambda_motion, min_rdcost, rdcost = 0, max_rdcost=1e30;
   int         lambda_motion_factor;
   int         fw_mcost, bw_mcost, bid_mcost, mcost, max_mcost=(1<<30);
   int         curr_cbp_blk, cnt_nonz = 0, best_cnt_nonz = 0, best_fw_ref = 0, best_pdir;
   int         cost=0;
   int         min_cost = max_mcost, min_cost8x8, cost8x8, cost_direct=0, have_direct=0, i16mode;
   int         intra1 = 0;
   
   int         intra       = (((img->type==P_SLICE||img->type==SP_SLICE) && img->mb_y==img->mb_y_upd && img->mb_y_upd!=img->mb_y_intra) || img->type==I_SLICE);
   int         spframe     = (img->type==SP_SLICE);
   int         siframe     = (img->type==SI_SLICE);
   int         bframe      = (img->type==B_SLICE);
   int         runs        = (input->RestrictRef==1 && input->rdopt==2 && (img->type==P_SLICE || img->type==SP_SLICE || (img->type==B_SLICE && img->nal_reference_idc>0)) ? 2 : 1);
   
   int         checkref    = (input->rdopt && input->RestrictRef && (img->type==P_SLICE || img->type==SP_SLICE));
   Macroblock* currMB      = &img->mb_data[img->current_mb_nr];
   Macroblock* prevMB      = img->current_mb_nr ? &img->mb_data[img->current_mb_nr-1]:NULL ;
   
   int     **ipredmodes = img->ipredmode;
   int     best_bw_ref = -1;
   int     ******allmvs = img->all_mv;

   
   int  l,list_offset;

   int curr_mb_field = ((img->MbaffFrameFlag)&&(currMB->mb_field));

   // find out the correct list offsets
   if (curr_mb_field)
   {
     if(img->current_mb_nr%2)
       list_offset = 4; // bottom field mb
     else
       list_offset = 2; // top field mb
   }
   else
   {
     list_offset = 0;  // no mb aff or frame mb
   }

   if(input->FMEnable)
     decide_intrabk_SAD();
   
   intra |= RandomIntra (img->current_mb_nr);    // Forced Pseudo-Random Intra

   //===== SET VALID MODES =====
   valid[I4MB]   = 1;
   valid[I16MB]  = 1;
   
   valid[0]      = (!intra );
   valid[1]      = (!intra && input->InterSearch16x16);
   valid[2]      = (!intra && input->InterSearch16x8);
   valid[3]      = (!intra && input->InterSearch8x16);
   valid[4]      = (!intra && input->InterSearch8x8);
   valid[5]      = (!intra && input->InterSearch8x4);
   valid[6]      = (!intra && input->InterSearch4x8);
   valid[7]      = (!intra && input->InterSearch4x4);
   valid[P8x8]   = (valid[4] || valid[5] || valid[6] || valid[7]);
   valid[12]     = (siframe);

   if (!img->MbaffFrameFlag)
   {
     for (l=0+list_offset;l<(2+list_offset);l++)
     {
       for(k = 0; k < listXsize[l]; k++)
       {
         listX[l][k]->chroma_vector_adjustment= 0;
         if(img->structure == TOP_FIELD && img->structure != listX[l][k]->structure)
           listX[l][k]->chroma_vector_adjustment = -2;
         if(img->structure == BOTTOM_FIELD && img->structure != listX[l][k]->structure)
           listX[l][k]->chroma_vector_adjustment = 2;
       }
     }
   }
   else
   {
     if (curr_mb_field)
     {
       for (l=0+list_offset;l<(2+list_offset);l++)
       {
         for(k = 0; k < listXsize[l]; k++)
         {
           listX[l][k]->chroma_vector_adjustment= 0;
           if(img->current_mb_nr % 2 == 0 && listX[l][k]->structure == BOTTOM_FIELD)
             listX[l][k]->chroma_vector_adjustment = -2;
           if(img->current_mb_nr % 2 == 1 && listX[l][k]->structure == TOP_FIELD)
             listX[l][k]->chroma_vector_adjustment = 2;
         }
       }
     }
     else
     {
       for (l=0+list_offset;l<(2+list_offset);l++)
       {
         for(k = 0; k < listXsize[l]; k++)
         {
           listX[l][k]->chroma_vector_adjustment= 0;
         }
       }
     }
     
   }
   //===== SET LAGRANGE PARAMETERS =====
   if (input->rdopt)
   {
     qp = (double)img->qp - SHIFT_QP;

     if (input->successive_Bframe>0)
       lambda_mode   = 0.68 * pow (2, qp/3.0) * (img->type==B_SLICE? max(2.00,min(4.00,(qp / 6.0))):spframe?max(1.4,min(3.0,(qp / 12.0))):1.0);  
     else
       lambda_mode   = 0.85 * pow (2, qp/3.0) * (img->type==B_SLICE? 4.0:spframe?max(1.4,min(3.0,(qp / 12.0))):1.0);  

     lambda_motion = sqrt (lambda_mode);
   }
   else
   {
     lambda_mode = lambda_motion = QP2QUANT[max(0,img->qp-SHIFT_QP)];
   }
   lambda_motion_factor = LAMBDA_FACTOR (lambda_motion);
   
   
   for (rerun=0; rerun<runs; rerun++)
   {
     if (runs==2)
     {
       if (rerun==0)   input->rdopt=1;
       else            input->rdopt=2;
     }
     
     // reset chroma intra predictor to default
     currMB->c_ipred_mode = DC_PRED_8;
     
     if (!intra)
     {
       //===== set direct motion vectors =====
       if (bframe)
       {
         Get_Direct_Motion_Vectors ();
       }
       
       //===== MOTION ESTIMATION FOR 16x16, 16x8, 8x16 BLOCKS =====
       for (min_cost=1<<20, best_mode=1, mode=1; mode<4; mode++)
       {
         if (valid[mode])
         {
           for (cost=0, block=0; block<(mode==1?1:2); block++)
           {
             PartitionMotionSearch (mode, block, lambda_motion);

             //--- set 4x4 block indizes (for getting MV) ---
             j = (block==1 && mode==2 ? 2 : 0);
             i = (block==1 && mode==3 ? 2 : 0);

             //--- get cost and reference frame for forward prediction ---
             for (fw_mcost=max_mcost, ref=0; ref<listXsize[LIST_0+list_offset]; ref++)
             {
               if (!checkref || ref==0 || CheckReliabilityOfRef (block, LIST_0, ref, mode))
               {
                 mcost  = (input->rdopt ? REF_COST (lambda_motion_factor, ref, LIST_0 + list_offset) : (int)(2*lambda_motion*min(ref,1)));

                 mcost += motion_cost[mode][LIST_0][ref][block];
                 if (mcost < fw_mcost)
                 {
                   fw_mcost    = mcost;
                   best_fw_ref = ref;
                 }
               }
             }

             if (bframe)
             {
               //--- get cost for bidirectional prediction ---
               for (bw_mcost=max_mcost, ref=0; ref<listXsize[LIST_1 + list_offset]; ref++)
               {
                 mcost  = (input->rdopt ? REF_COST (lambda_motion_factor, ref, LIST_1 + list_offset) : (int)(2*lambda_motion*min(ref,1)));
                 mcost += motion_cost[mode][LIST_1][ref][block];
                 if (mcost < bw_mcost)
                 {
                   bw_mcost    = mcost;
                   best_bw_ref = ref;
                 }
               }

               // search bidirectional between best forward and ref_idx=0 backward
               bid_mcost  = (input->rdopt ? (REF_COST (lambda_motion_factor, best_fw_ref,LIST_0+list_offset)+REF_COST (lambda_motion_factor, 0,LIST_1+list_offset)) : (int)(2*lambda_motion*min(best_fw_ref,1)));
               bid_mcost += BIDPartitionCost (mode, block, best_fw_ref, 0, lambda_motion_factor);

               //--- get prediction direction ----
               if (fw_mcost<=bw_mcost && fw_mcost<=bid_mcost)
               {
                 best_pdir = 0;
                 best_bw_ref = 0;
                 cost += fw_mcost;
               }
               else if (bw_mcost<=fw_mcost && bw_mcost<=bid_mcost)
               {
                 best_pdir = 1;
                 cost += bw_mcost;
                 best_fw_ref = 0;
               }
               else
               {
                 best_pdir = 2;
                 cost += bid_mcost;
                 best_bw_ref = 0;
               }
             }
             else // if (bframe)
             {
               best_pdir  = 0;
               cost      += fw_mcost;
             }

             if (mode==1)
             {
               if (best_pdir==1)
               {
                 for (j=0; j<4; j++)
                 {
                   for (i=0; i<4; i++)
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = -1;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = -1; 
                     enc_picture->mv[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][0] = 0;
                     enc_picture->mv[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][1] = 0;
                   }
                 }
               }
               else
               {
                 for (j=0; j<4; j++)
                 {
                   for (i=0; i<4; i++)
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = best_fw_ref;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j]];  
                     enc_picture->mv[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][0] = img->all_mv[i][j][LIST_0][best_fw_ref][mode][0];
                     enc_picture->mv[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][1] = img->all_mv[i][j][LIST_0][best_fw_ref][mode][1];
                   }
                 }
               }

               if (bframe)
               {
                 if (best_pdir==0)
                 {
                   for (j=0; j<4; j++)
                   {
                     for (i=0; i<4; i++)
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = -1;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = -1;   
                       enc_picture->mv[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][0] = 0;
                       enc_picture->mv[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][1] = 0;
                     }
                   }
                 }
                 else
                 {
                   for (j=0; j<4; j++)
                   {
                     for (i=0; i<4; i++)
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = best_bw_ref;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j]];
                       if(best_bw_ref>=0)
                       {
                         enc_picture->mv[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][0] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][0];
                         enc_picture->mv[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j][1] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][1];
                       }
                     }
                   }
                 }
               }
             }
             else if (mode==2)
             {
               for (j=0; j<2; j++)
               {
                 for (i=0; i<4; i++)
                 {
                   if (best_pdir==1)
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+block*2+j] = -1;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+i][img->block_y+block*2+j] = -1;
                     enc_picture->mv[LIST_0][img->block_x+i][img->block_y+block*2+j][0] = 0;
                     enc_picture->mv[LIST_0][img->block_x+i][img->block_y+block*2+j][1] = 0;
                   }
                   else
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+block*2+j] = best_fw_ref;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+i][img->block_y+block*2+j] = enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+block*2+j]];
                     enc_picture->mv[LIST_0][img->block_x+i][img->block_y+block*2+j][0] = img->all_mv[i][j+block*2][LIST_0][best_fw_ref][mode][0];
                     enc_picture->mv[LIST_0][img->block_x+i][img->block_y+block*2+j][1] = img->all_mv[i][j+block*2][LIST_0][best_fw_ref][mode][1];
                   }

                   if (bframe)
                   {
                     if (best_pdir==0)
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+block*2+j] = -1;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+block*2+j] = -1;
                       enc_picture->mv[LIST_1][img->block_x+i][img->block_y+block*2+j][0] = 0;
                       enc_picture->mv[LIST_1][img->block_x+i][img->block_y+block*2+j][1] = 0;
                     }
                     else
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+block*2+j] = best_bw_ref;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+block*2+j] = enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+block*2+j]];
                       if(best_bw_ref>=0)
                       {
                         enc_picture->mv[LIST_1][img->block_x+i][img->block_y+block*2+j][0] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][0];
                         enc_picture->mv[LIST_1][img->block_x+i][img->block_y+block*2+j][1] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][1];
                       }
                     }
                   }
                 }
               }
             }
             else
             {
               for (j=0; j<4; j++)
               {
                 for (i=0; i<2; i++)
                 {
                   if (best_pdir==1)
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+block*2+i][img->block_y+j] = -1;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+block*2+i][img->block_y+j] = -1;
                     enc_picture->mv[LIST_0][img->block_x+block*2+i][img->block_y+j][0] = 0;
                     enc_picture->mv[LIST_0][img->block_x+block*2+i][img->block_y+j][1] = 0;
                   }
                   else
                   {
                     enc_picture->ref_idx[LIST_0][img->block_x+block*2+i][img->block_y+j] = best_fw_ref;
                     enc_picture->ref_pic_id [LIST_0][img->block_x+block*2+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+block*2+i][img->block_y+j]];
                     enc_picture->mv[LIST_0][img->block_x+block*2+i][img->block_y+j][0] = img->all_mv[i][j][LIST_0][best_fw_ref][mode][0];
                     enc_picture->mv[LIST_0][img->block_x+block*2+i][img->block_y+j][1] = img->all_mv[i][j][LIST_0][best_fw_ref][mode][1];
                   }

                   if (bframe)
                   {
                     if (best_pdir==0)
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+block*2+i][img->block_y+j] = -1;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+block*2+i][img->block_y+j] = -1;
                       enc_picture->mv[LIST_1][img->block_x+block*2+i][img->block_y+j][0] = 0;
                       enc_picture->mv[LIST_1][img->block_x+block*2+i][img->block_y+j][1] = 0;
                     }
                     else
                     {
                       enc_picture->ref_idx[LIST_1][img->block_x+block*2+i][img->block_y+j] = best_bw_ref;
                       enc_picture->ref_pic_id [LIST_1][img->block_x+block*2+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+block*2+i][img->block_y+j]];
                       if(best_bw_ref>=0)
                       {
                         enc_picture->mv[LIST_1][img->block_x+block*2+i][img->block_y+j][0] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][0];
                         enc_picture->mv[LIST_1][img->block_x+block*2+i][img->block_y+j][1] = img->all_mv[i][j][LIST_1][best_bw_ref][mode][1];
                       }
                     }
                   }
                 }
               }
             }

             //----- set reference frame and direction parameters -----
             if (mode==3)
             {
               best8x8fwref   [3][block] = best8x8fwref   [3][block+2] = best_fw_ref;
               best8x8pdir    [3][block] = best8x8pdir    [3][block+2] = best_pdir;
               best8x8bwref   [3][block] = best8x8bwref   [3][block+2] = best_bw_ref;
             }
             else if (mode==2)
             {
               best8x8fwref   [2][2*block] = best8x8fwref   [2][2*block+1] = best_fw_ref;
               best8x8pdir    [2][2*block] = best8x8pdir    [2][2*block+1] = best_pdir;
               best8x8bwref   [2][2*block] = best8x8bwref   [2][2*block+1] = best_bw_ref;
             }
             else
             {
               best8x8fwref   [1][0] = best8x8fwref   [1][1] = best8x8fwref   [1][2] = best8x8fwref   [1][3] = best_fw_ref;
               best8x8pdir    [1][0] = best8x8pdir    [1][1] = best8x8pdir    [1][2] = best8x8pdir    [1][3] = best_pdir;
               best8x8bwref   [1][0] = best8x8bwref   [1][1] = best8x8bwref   [1][2] = best8x8bwref   [1][3] = best_bw_ref;
             }

             //--- set reference frames and motion vectors ---
             if (mode>1 && block==0) 
               SetRefAndMotionVectors (block, mode, best_pdir, best_fw_ref, best_bw_ref);

          } // for (block=0; block<(mode==1?1:2); block++)

          if (cost < min_cost)
          {
            best_mode = mode;
            min_cost  = cost;
          }
        } // if (valid[mode])
      } // for (mode=1; mode<4; mode++)
      
      if (valid[P8x8])
      {
        cost8x8 = 0;
        
        //===== store coding state of macroblock =====
        store_coding_state (cs_mb);
        
        //=====  LOOP OVER 8x8 SUB-PARTITIONS  (Motion Estimation & Mode Decision) =====
        for (cbp8x8=cbp_blk8x8=cnt_nonz_8x8=0, block=0; block<4; block++)
        {
          //--- set coordinates ---
          j0 = ((block/2)<<3);    j1 = (j0>>2);
          i0 = ((block%2)<<3);    i1 = (i0>>2);
          
          //=====  LOOP OVER POSSIBLE CODING MODES FOR 8x8 SUB-PARTITION  =====
          for (min_cost8x8=(1<<20), min_rdcost=1e30, index=(bframe?0:1); index<5; index++)
          {
            if (valid[mode=b8_mode_table[index]])
            {
              curr_cbp_blk = 0;
              
              if (mode==0)
              {
                //--- Direct Mode ---
                if (!input->rdopt)
                {
                  cost_direct += (cost = Get_Direct_Cost8x8 ( block, lambda_mode ));
                  if (cost==1<<30)
                    cost_direct = (1<<30);
                  have_direct ++;
                }
                best_fw_ref = direct_ref_idx[LIST_0][img->block_x+(block&1)*2][img->block_y+(block&2)];
                best_bw_ref = direct_ref_idx[LIST_1][img->block_x+(block&1)*2][img->block_y+(block&2)];
                best_pdir   = direct_pdir[img->block_x+(block&1)*2][img->block_y+(block&2)];
              } // if (mode==0)
              else
              {
                //--- motion estimation for all reference frames ---
                PartitionMotionSearch (mode, block, lambda_motion);
                
                //--- get cost and reference frame for forward prediction ---
                for (fw_mcost=max_mcost, ref=0; ref<listXsize[LIST_0+list_offset]; ref++)
                {
                  if (!checkref || ref==0 || CheckReliabilityOfRef (block, LIST_0, ref, mode))
                  {
                    mcost  = (input->rdopt ? REF_COST(lambda_motion_factor,ref,LIST_0+list_offset) : (int)(2*lambda_motion*min(ref,1)));
                    
                    mcost += motion_cost[mode][LIST_0][ref][block];
                    if (mcost < fw_mcost)
                    {
                      fw_mcost    = mcost;
                      best_fw_ref = ref;
                    }
                  }
                }
                
                //store forward reference index for every block
                for (j=0; j<2; j++)
                  for (i=0; i<2; i++)
                  {
                    enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = best_fw_ref;
                    enc_picture->ref_pic_id [LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j]];
                  }

                if (bframe)
                {
                  for (bw_mcost=max_mcost, ref=0; ref<listXsize[LIST_1+list_offset]; ref++)
                  {
                    mcost  = (input->rdopt ? REF_COST(lambda_motion_factor,ref,LIST_1+list_offset) : (int)(2*lambda_motion*min(ref,1)));
                    
                    mcost += motion_cost[mode][LIST_1][ref][block];
                    if (mcost < bw_mcost)
                    {
                      bw_mcost    = mcost;
                      best_bw_ref = ref;
                    }
                  }
                  
                  // bidirectional uses best forward and zero backward reference
                  bid_mcost  = (input->rdopt ? (REF_COST (lambda_motion_factor, best_fw_ref, LIST_0 + list_offset)+REF_COST (lambda_motion_factor, 0, LIST_1 + list_offset)) : (int)(2*lambda_motion*min(best_fw_ref,1)));
                  bid_mcost += BIDPartitionCost (mode, block, best_fw_ref, 0, lambda_motion_factor );
                  
                  //--- get prediction direction ----
                  if      (fw_mcost<=bw_mcost && fw_mcost<=bid_mcost)
                  {
                    best_pdir = 0;
                    cost = fw_mcost;
                    best_bw_ref = -1;
                  }
                  else if (bw_mcost<=fw_mcost && bw_mcost<=bid_mcost)
                  {
                    best_pdir = 1;
                    cost = bw_mcost;
                    best_fw_ref = -1;
                  }
                  else
                  {
                    best_pdir = 2;
                    cost = bid_mcost;
                    best_bw_ref = 0;
                  }
                    //store backward reference index for every block
                  for (j=0; j<2; j++)
                    for (i=0; i<2; i++)
                    {
                      enc_picture->ref_idx[LIST_0][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = best_fw_ref;
                      enc_picture->ref_idx[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = best_bw_ref;
                      //enc_picture->ref_pic_id [LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j] = enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+(block&1)*2+i][img->block_y+(block&2)+j]];  
                    }
                } // if (bframe)
                else
                {
                  best_pdir = 0;
                  cost      = fw_mcost;
                }
              } // if (mode!=0)
              
              //--- store coding state before coding with current mode ---
              store_coding_state (cs_cm);
              
              if (input->rdopt)
              {
                //--- get and check rate-distortion cost ---
                rdcost = RDCost_for_8x8blocks (&cnt_nonz, &curr_cbp_blk, lambda_mode,
                                               block, mode, best_pdir, best_fw_ref, best_bw_ref);
              }
              else
              {
                cost += (REF_COST (lambda_motion_factor, B8Mode2Value (mode, best_pdir), list_offset + (best_pdir<1?0:1)) - 1);
              }
              
              //--- set variables if best mode has changed ---
              if (( input->rdopt && rdcost < min_rdcost) ||
                  (!input->rdopt && cost   < min_cost8x8  )   )
              {
                min_cost8x8                  = cost;
                min_rdcost                   = rdcost;
                best8x8mode          [block] = mode;
                best8x8pdir    [P8x8][block] = best_pdir;
                best8x8fwref   [P8x8][block] = best_fw_ref;
                best8x8bwref   [P8x8][block] = best_bw_ref;

                
                //--- store number of nonzero coefficients ---
                best_cnt_nonz  = cnt_nonz;
                
                if (input->rdopt)
                {
                  //--- store block cbp ---
                  cbp_blk8x8    &= (~(0x33 << (((block>>1)<<3)+((block%2)<<1)))); // delete bits for block
                  cbp_blk8x8    |= curr_cbp_blk;
                  
                  //--- store coefficients ---
                  for (k=0; k< 4; k++)
                    for (j=0; j< 2; j++)
                      for (i=0; i<65; i++)  cofAC8x8[block][k][j][i] = img->cofAC[block][k][j][i]; // 18->65 for ABT
                      
                      //--- store reconstruction and prediction ---
                  for (j=j0; j<j0+8; j++)
                    for (i=i0; i<i0+8; i++)
                    {
                      rec_mbY8x8[j][i] = enc_picture->imgY[img->pix_y+j][img->pix_x+i];
                      mpr8x8    [j][i] = img->mpr[i][j];
                    }
                }
                
                //--- store coding state ---
                store_coding_state (cs_b8);
              } // if (rdcost <= min_rdcost)
              
              //--- re-set coding state as it was before coding with current mode was performed ---
              reset_coding_state (cs_cm);
            } // if (valid[mode=b8_mode_table[index]])
          } // for (min_rdcost=1e30, index=(bframe?0:1); index<6; index++)
          
          cost8x8 += min_cost8x8;

//          if (!input->rdopt) cost8x8+= min_cost8x8;
//          else cost8x8 += min_rdcost;          
          
          if (!input->rdopt)
          {
            mode = best8x8mode[block];
            pdir = best8x8pdir[P8x8][block];
            
            curr_cbp_blk  = 0;
            best_cnt_nonz = LumaResidualCoding8x8 (&dummy, &curr_cbp_blk, block, pdir,
                                                   (pdir==0||pdir==2?mode:0),
                                                   (pdir==1||pdir==2?mode:0),
                                                   (best8x8fwref[P8x8][block]),
                                                   (best8x8bwref[P8x8][block]));
            cbp_blk8x8   &= (~(0x33 << (((block>>1)<<3)+((block%2)<<1)))); // delete bits for block
            cbp_blk8x8   |= curr_cbp_blk;
            
            //--- store coefficients ---
            for (k=0; k< 4; k++)
              for (j=0; j< 2; j++)
                for (i=0; i<65; i++)  cofAC8x8[block][k][j][i] = img->cofAC[block][k][j][i]; // 18->65 for ABT
                
                //--- store reconstruction and prediction ---
            for (j=j0; j<j0+8; j++)
              for (i=i0; i<i0+8; i++)
              {
                rec_mbY8x8[j][i] = enc_picture->imgY[img->pix_y+j][img->pix_x+i];
                mpr8x8    [j][i] = img->mpr[i][j];
              }
          }
          
          //----- set cbp and count of nonzero coefficients ---
          if (best_cnt_nonz)
          {
            cbp8x8        |= (1<<block);
            cnt_nonz_8x8  += best_cnt_nonz;
          }
          
          mode=best8x8mode[block];
          //===== reset intra prediction modes (needed for prediction, must be stored after 8x8 mode dec.) =====
          j0 = img->block_y+2*(block/2);
          i0 = img->block_x+2*(block%2);
          for (j=j0; j<j0+2; j++)
            for (i=i0; i<i0+2; i++) 
              ipredmodes[i][j]         = DC_PRED;
          i0 = 4*block;
          for (i=i0; i<i0+4; i++)    currMB->intra_pred_modes[i]  = DC_PRED;
            
          if (block<3)
          {
            //===== re-set reconstructed block =====
            j0   = 8*(block/2);
            i0   = 8*(block%2);
            for (j=j0; j<j0+8; j++)
              for (i=i0; i<i0+8; i++)  
                enc_picture->imgY[img->pix_y+j][img->pix_x+i] = rec_mbY8x8[j][i];
          } // if (block<3)
            
          //===== set motion vectors and reference frames (prediction) =====
          SetRefAndMotionVectors (block, mode, best8x8pdir[P8x8][block], best8x8fwref[P8x8][block], best8x8bwref[P8x8][block]);
          //===== set the coding state after current block =====
          reset_coding_state (cs_b8);
        } // for (cbp8x8=cbp_blk8x8=cnt_nonz_8x8=0, block=0; block<4; block++)
        
        //===== store intra prediction modes for 8x8+ macroblock mode =====
        for (k=0, j=img->block_y; j<img->block_y+4; j++)
        {
          for (     i=img->block_x; i<img->block_x+4; i++, k++)
          {
            b8_ipredmode       [k] = ipredmodes    [i][j];
            b8_intra_pred_modes[k] = currMB->intra_pred_modes[k];
          }
        }
        
        //--- re-set coding state (as it was before 8x8 block coding) ---
        reset_coding_state (cs_mb);
        for (i=0; i<16; i++)
          for(j=0; j<16; j++)
            diffy[j][i] = imgY_org[img->opix_y+j][img->opix_x+i]-img->mpr[j][i];

        if(cost8x8 < min_cost)
        {
           best_mode = P8x8;
           min_cost = cost8x8;
        }
      }
      else // if (valid[P8x8])
      {
        cost8x8 = (1<<20);
      }
   
      // Find a motion vector for the Skip mode
      if((img->type == P_SLICE)||(img->type == SP_SLICE))
        FindSkipModeMotionVector ();
    }
    else // if (img->type!=I_SLICE)
    {
      min_cost = (1<<20);
    }
    
    if (input->rdopt)
    {
      int mb_available_up;
      int mb_available_left;
      int mb_available_up_left;
      
      min_rdcost = max_rdcost;
      
      // precompute all new chroma intra prediction modes
      IntraChromaPrediction8x8(&mb_available_up, &mb_available_left, &mb_available_up_left);
      
      for (currMB->c_ipred_mode=DC_PRED_8; currMB->c_ipred_mode<=PLANE_8; currMB->c_ipred_mode++)
      {
        
        // bypass if c_ipred_mode is not allowed
        if ((currMB->c_ipred_mode==VERT_PRED_8 && !mb_available_up) ||
          (currMB->c_ipred_mode==HOR_PRED_8 && !mb_available_left) ||
          (currMB->c_ipred_mode==PLANE_8 && (!mb_available_left || !mb_available_up || !mb_available_up_left)))
          continue;
        
        
        //===== GET BEST MACROBLOCK MODE =====
        for (ctr16x16=0, index=0; index<7; index++)
        {
          mode = mb_mode_table[index];
          
          //--- for INTER16x16 check all prediction directions ---
          if (mode==1 && img->type==B_SLICE)
          {
            best8x8pdir[1][0] = best8x8pdir[1][1] = best8x8pdir[1][2] = best8x8pdir[1][3] = ctr16x16;
            if (ctr16x16 < 2) index--;
            ctr16x16++;
          }
          
          img->NoResidueDirect = 0;
          
          if (valid[mode])
          {
            // bypass if c_ipred_mode not used
            SetModesAndRefframeForBlocks (mode);
            if (currMB->c_ipred_mode == DC_PRED_8 ||
              (IS_INTRA(currMB) ))
            {
              if (RDCost_for_macroblocks (lambda_mode, mode, &min_rdcost))
              {
                //Rate control
                if(mode == P8x8)
                {
                  for (i=0; i<16; i++)
                    for(j=0; j<16; j++)
                      diffy[j][i] = imgY_org[img->opix_y+j][img->opix_x+i] - mpr8x8[j][i];
                }else
                {
                  for (i=0; i<16; i++)
                    for(j=0; j<16; j++)
                      diffy[j][i] = imgY_org[img->opix_y+j][img->opix_x+i] - pred[j][i];
                }
                
                store_macroblock_parameters (mode);
              }
            }
          }
          if (valid[0] && bframe && mode == 0 && currMB->cbp && (currMB->cbp&15) != 15) //g050
          {
            img->NoResidueDirect = 1;
            if (RDCost_for_macroblocks (lambda_mode, mode, &min_rdcost))
            {
              //Rate control
              for (i=0; i<16; i++)
                for(j=0; j<16; j++)
                  diffy[j][i] = imgY_org[img->opix_y+j][img->opix_x+i] - pred[j][i];
                store_macroblock_parameters (mode);
            }
          }
        }
      }
    }
    else
    {
      if (valid[0] && bframe) // check DIRECT MODE
      {
        cost  = (have_direct?cost_direct:Get_Direct_CostMB (lambda_mode));
        cost -= (int)floor(16*lambda_motion+0.4999);
        if (cost <= min_cost)
        {
          //Rate control
          for (i=0; i<16; i++)
            for(j=0; j<16; j++)
              diffy[j][i] = imgY_org[img->pix_y+j][img->pix_x+i]-img->mpr[i][j];
            
            min_cost  = cost;
            best_mode = 0;
        }
      }
      if (valid[I4MB]) // check INTRA4x4
      {
        currMB->cbp = Mode_Decision_for_Intra4x4Macroblock (lambda_mode, &cost);
        if (cost <= min_cost)
        {
          //Rate control
          for (i=0; i<16; i++)
            for(j=0; j<16; j++)
              diffy[j][i] = imgY_org[img->pix_y+j][img->pix_x+i]-img->mpr[i][j];
            
            min_cost  = cost;
            best_mode = I4MB;
        }
      }
      if (valid[I16MB]) // check INTRA16x16
      {
        intrapred_luma_16x16 ();
        cost = find_sad_16x16 (&i16mode);
        if (cost < min_cost)
        {
          //Rate control
          for (i=0; i<16; i++)
            for(j=0; j<16; j++)
              diffy[j][i] = imgY_org[img->pix_y+j][img->pix_x+i]-img->mprr_2[i16mode][j][i];
            
            best_mode   = I16MB;
            currMB->cbp = dct_luma_16x16 (i16mode);
        }
      }
    }
    
    if (rerun==0)
    {
      intra1 = (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
    }
  } // for (rerun=0; rerun<runs; rerun++)
  
  if (input->rdopt)
  {
    
    if ((cbp!=0 || best_mode==I16MB ))
      currMB->prev_cbp = 1;
    else if (cbp==0 && !input->RCEnable)
    {
      currMB->delta_qp = 0;
      currMB->qp = currMB->prev_qp;
      img->qp = currMB->qp;
      currMB->prev_cbp = 0;
    }

    set_stored_macroblock_parameters ();
  }
  else
  {
    //===== set parameters for chosen mode =====
    SetModesAndRefframeForBlocks (best_mode);
    if (best_mode==P8x8)
    {
      SetCoeffAndReconstruction8x8 (currMB);
    }
    else
    {
      if (best_mode!=I4MB)
      {
        for (k=0, j=img->block_y; j<img->block_y+4; j++)
          for (     i=img->block_x; i<img->block_x+4; i++, k++)
          {
            ipredmodes    [i][j] = DC_PRED;
            currMB->intra_pred_modes[k] = DC_PRED;
          }
          if (best_mode!=I16MB)
          {
            LumaResidualCoding ();
            //Rate control
            for (i=0; i<16; i++)
              for(j=0; j<16; j++)
                diffy[j][i] = imgY_org[img->pix_y+j][img->pix_x+i]-img->mpr[i][j];
          }
      }
    }
    // precompute all chroma intra prediction modes
    IntraChromaPrediction8x8(NULL, NULL, NULL);
    img->i16offset = 0;
    dummy = 0;
    ChromaResidualCoding (&dummy);
    if (best_mode==I16MB)
    {
      img->i16offset = I16Offset  (currMB->cbp, i16mode);
    }
    SetMotionVectorsMB (currMB, bframe);
    
    //===== check for SKIP mode =====
    if ((img->type==P_SLICE || img->type==SP_SLICE) && best_mode==1 && currMB->cbp==0 &&
      enc_picture->ref_idx[LIST_0][img->block_x][img->block_y]==0 &&
      enc_picture->mv[LIST_0][img->block_x][img->block_y][0]==allmvs[0][0][0][0][0][0] &&
      enc_picture->mv[LIST_0][img->block_x][img->block_y][1]==allmvs[0][0][0][0][0][1]               )
    {
      currMB->mb_type=currMB->b8mode[0]=currMB->b8mode[1]=currMB->b8mode[2]=currMB->b8mode[3]=0;
    }
    
    if(img->MbaffFrameFlag)
      set_mbaff_parameters();
  }
  
  // Rate control
  if(input->RCEnable)
  {   
    if(img->type==P_SLICE)
    {
      img->MADofMB[img->current_mb_nr] = calc_MAD();
      
      if(input->basicunit<img->Frame_Total_Number_MB)
      {
        img->TotalMADBasicUnit +=img->MADofMB[img->current_mb_nr];
        
        /* delta_qp is present only for non-skipped macroblocks*/
        if ((cbp!=0 || best_mode==I16MB))
          currMB->prev_cbp = 1;
        else
        {
          img->qp -= currMB->delta_qp;
          currMB->delta_qp = 0;
          currMB->qp = img->qp;
          currMB->prev_cbp = 0;
        }
        /* When MBAFF is used, delta_qp is only present for the first non-skipped macroblock of each 
        macroblock pair*/
        if (input->MbInterlace)
        {
          if(!currMB->mb_field)
          {
            DELTA_QP = currMB->delta_qp;
            QP      = currMB->qp;
          }
          else
          {
            DELTA_QP2 = currMB->delta_qp;
            QP2      = currMB->qp;
          }
        }       
      }
    }
  }
  
    
  if(input->rdopt)
    rdopt->min_rdcost = min_rdcost;
  else
    rdopt->min_rdcost = min_cost; 

  if(img->MbaffFrameFlag)
  {
    if (img->current_mb_nr%2) //bottom
    {
      if ((currMB->mb_type ? 0:((img->type == B_SLICE) ? !currMB->cbp:1))  // bottom is skip
        &&(prevMB->mb_type ? 0:((img->type == B_SLICE) ? !prevMB->cbp:1))) // top is skip
      {
        if (!(field_flag_inference() == curr_mb_field))
        {
          rdopt->min_rdcost = 1e30;  // don't allow coding of an MB pair as skip if wrong inference
        }
      }
    }
  }
  
  //===== Decide if this MB will restrict the reference frames =====
  if (input->RestrictRef==1)
  {
    if (input->rdopt==1)
    {
      refresh_map[2*img->mb_y  ][2*img->mb_x  ] = (intra ? 1 : 0);
      refresh_map[2*img->mb_y  ][2*img->mb_x+1] = (intra ? 1 : 0);
      refresh_map[2*img->mb_y+1][2*img->mb_x  ] = (intra ? 1 : 0);
      refresh_map[2*img->mb_y+1][2*img->mb_x+1] = (intra ? 1 : 0);
    }
    else if (input->rdopt==2)
    {
      refresh_map[2*img->mb_y  ][2*img->mb_x  ] = (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
      refresh_map[2*img->mb_y  ][2*img->mb_x+1] = (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
      refresh_map[2*img->mb_y+1][2*img->mb_x  ] = (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
      refresh_map[2*img->mb_y+1][2*img->mb_x+1] = (intra1==0 && (currMB->mb_type==I16MB || currMB->mb_type==I4MB) ? 1 : 0);
    }
  }
  else if (input->RestrictRef==2)
  {
    refresh_map[2*img->mb_y  ][2*img->mb_x  ] = (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
    refresh_map[2*img->mb_y  ][2*img->mb_x+1] = (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
    refresh_map[2*img->mb_y+1][2*img->mb_x  ] = (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
    refresh_map[2*img->mb_y+1][2*img->mb_x+1] = (currMB->mb_type==I16MB || currMB->mb_type==I4MB ? 1 : 0);
  }
  
  if(input->FMEnable)
    skip_intrabk_SAD(best_mode, listXsize[LIST_0+list_offset]);
}


void set_mbaff_parameters()
{
  int  i, j, k, l;
  Macroblock  *currMB  = &img->mb_data[img->current_mb_nr];
  int         mode     = best_mode;
  int         bframe   = (img->type==B_SLICE);
  int     **ipredmodes = img->ipredmode;

  if (!img->MbaffFrameFlag)
    return;

  //===== reconstruction values =====
  for (j=0; j<16; j++)
    for (i=0; i<16; i++)
      rdopt->rec_mbY[j][i]           = enc_picture->imgY[img->pix_y+j][img->pix_x+i]; 

  for (j=0; j<8; j++)
    for (i=0; i<8; i++)
    {
      rdopt->rec_mbU[j][i]           = enc_picture->imgUV[0][img->pix_c_y+j][img->pix_c_x+i];   
      rdopt->rec_mbV[j][i]           = enc_picture->imgUV[1][img->pix_c_y+j][img->pix_c_x+i];    
    }

  //===== coefficients and cbp =====
  rdopt->mode = mode;
  rdopt->i16offset = img->i16offset;  // For MBINTLC  -Rajeev
  rdopt->cbp = currMB->cbp;
  rdopt->cbp_blk = currMB->cbp_blk;
  rdopt->mb_type  =currMB->mb_type;

  if(rdopt->mb_type == 0 && mode != 0)
  {
    mode=0;
    rdopt->mode=0;
  }

  for(i=0;i<6;i++)
    for(j=0;j<4;j++)
      for(k=0;k<2;k++)
        for(l=0;l<18;l++)
          rdopt->cofAC[i][j][k][l] = img->cofAC[i][j][k][l];

  for(i=0;i<3;i++)
    for(k=0;k<2;k++)
      for(l=0;l<18;l++)
        rdopt->cofDC[i][k][l] = img->cofDC[i][k][l];


  for (i=0; i<4; i++)
  {
    rdopt->b8mode[i]  = currMB->b8mode[i];                  
    rdopt->b8pdir[i]  = currMB->b8pdir[i];                  
  }

  //==== reference frames =====
  for (j=0; j<4; j++)
    for (i=0; i<4; i++)
    {
      rdopt->refar[LIST_0][j][i]       = enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j];
    }

  if (bframe)
  {
    for (j=0; j<4; j++)
      for (i=0; i<4; i++)
      {
        rdopt->refar[LIST_1][j][i]     = enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j];
      }
  }


  for   (k=0, j=img->block_y; j<img->block_y+4; j++)
    for (     i=img->block_x; i<img->block_x+4; i++, k++)
    {
      rdopt->ipredmode[i][j]     = ipredmodes[i][j];
      rdopt->intra_pred_modes[k] = currMB->intra_pred_modes[k];
    }
}
