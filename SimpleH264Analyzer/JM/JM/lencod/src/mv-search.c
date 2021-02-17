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
 * \file mv-search.c
 *
 * \brief
 *    Motion Vector Search, unified for B and P Pictures
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *      - Inge Lille-Langøy               <inge.lille-langoy@telenor.com>
 *      - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *      - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *      - Jani Lainema                    <jani.lainema@nokia.com>
 *      - Detlev Marpe                    <marpe@hhi.de>
 *      - Thomas Wedi                     <wedi@tnt.uni-hannover.de>
 *      - Heiko Schwarz                   <hschwarz@hhi.de>
 *
 *************************************************************************************
*/

#include "contributors.h"

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>

#include "global.h"
#include "image.h"
#include "mv-search.h"
#include "refbuf.h"
#include "memalloc.h"
#include "mb_access.h"
#include "fast_me.h"

#include <time.h>
#include <sys/timeb.h>

// These procedure pointers are used by motion_search() and one_eigthpel()
static pel_t  (*PelY_14)     (pel_t**, int, int, int, int);
static pel_t *(*PelYline_11) (pel_t *, int, int, int, int);

// Statistics, temporary
int     max_mvd;
int*    spiral_search_x;
int*    spiral_search_y;
int*    mvbits;
int*    refbits;
int*    byte_abs;
int**** motion_cost;


void SetMotionVectorPredictor (int  pmv[2],
                               int  ***refPic,
                               int  ****tmp_mv,
                               int  ref_frame,
                               int  list,
                               int  block_x,
                               int  block_y,
                               int  blockshape_x,
                               int  blockshape_y);

#ifdef _FAST_FULL_ME_

/*****
 *****  static variables for fast integer motion estimation
 *****
 */
static int  **search_setup_done;  //!< flag if all block SAD's have been calculated yet
static int  **search_center_x;    //!< absolute search center for fast full motion search
static int  **search_center_y;    //!< absolute search center for fast full motion search
static int  **pos_00;             //!< position of (0,0) vector
static int  *****BlockSAD;        //!< SAD for all blocksize, ref. frames and motion vectors
static int  **max_search_range;

extern ColocatedParams *Co_located;

/*!
 ***********************************************************************
 * \brief
 *    function creating arrays for fast integer motion estimation
 ***********************************************************************
 */
void
InitializeFastFullIntegerSearch ()
{
  int  i, j, k, list;
  int  search_range = input->search_range;
  int  max_pos      = (2*search_range+1) * (2*search_range+1);

  if ((BlockSAD = (int*****)malloc (2 * sizeof(int****))) == NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");

  for (list=0; list<2;list++)
  {
    if ((BlockSAD[list] = (int****)malloc ((img->max_num_references+1) * sizeof(int***))) == NULL)
      no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
    for (i = 0; i <= img->max_num_references; i++)
    {
      if ((BlockSAD[list][i] = (int***)malloc (8 * sizeof(int**))) == NULL)
        no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
      for (j = 1; j < 8; j++)
      {
        if ((BlockSAD[list][i][j] = (int**)malloc (16 * sizeof(int*))) == NULL)
          no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
        for (k = 0; k < 16; k++)
        {
          if ((BlockSAD[list][i][j][k] = (int*)malloc (max_pos * sizeof(int))) == NULL)
            no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
        }
      }
    }
  }

  if ((search_setup_done = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_setup_done");
  if ((search_center_x = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_x");
  if ((search_center_y = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_y");
  if ((pos_00 = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: pos_00");
  if ((max_search_range = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: max_search_range");

  for (list=0; list<2; list++)
  {
  if ((search_setup_done[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_setup_done");
  if ((search_center_x[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_x");
  if ((search_center_y[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_y");
  if ((pos_00[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: pos_00");
  if ((max_search_range[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: max_search_range");
  }

  // assign max search ranges for reference frames
  if (input->full_search == 2)
  {
    for (list=0;list<2;list++)
      for (i=0; i<=img->max_num_references; i++)  
        max_search_range[list][i] = search_range;
  }
  else
  {
    for (list=0;list<2;list++)
    {
      max_search_range[list][0] = max_search_range[list][img->max_num_references] = search_range;
      for (i=1; i< img->max_num_references; i++)  max_search_range[list][i] = search_range / 2;
    }
  }

}



/*!
 ***********************************************************************
 * \brief
 *    function for deleting the arrays for fast integer motion estimation
 ***********************************************************************
 */
void
ClearFastFullIntegerSearch ()
{
  int  i, j, k, list;

  for (list=0; list<2; list++)
  {
    for (i = 0; i <= img->max_num_references; i++)
    {
      for (j = 1; j < 8; j++)
      {
        for (k = 0; k < 16; k++)
        {
          free (BlockSAD[list][i][j][k]);
        }
        free (BlockSAD[list][i][j]);
      }
      free (BlockSAD[list][i]);
    }
    free (BlockSAD[list]);
  }
  free (BlockSAD);

  for (list=0; list<2; list++)
  {
    free (search_setup_done[list]);
    free (search_center_x[list]);
    free (search_center_y[list]);
    free (pos_00[list]);
    free (max_search_range[list]);
  }
  free (search_setup_done);
  free (search_center_x);
  free (search_center_y);
  free (pos_00);
  free (max_search_range);

}


/*!
 ***********************************************************************
 * \brief
 *    function resetting flags for fast integer motion estimation
 *    (have to be called in start_macroblock())
 ***********************************************************************
 */
void
ResetFastFullIntegerSearch ()
{
  int i,list;

  for (list=0; list<2; list++)
    for (i = 0; i <= img->max_num_references; i++)
      search_setup_done [list][i] = 0;
}

/*!
 ***********************************************************************
 * \brief
 *    calculation of SAD for larger blocks on the basis of 4x4 blocks
 ***********************************************************************
 */
void
SetupLargerBlocks (int list, int refindex, int max_pos)
{
#define ADD_UP_BLOCKS()   _o=*_bo; _i=*_bi; _j=*_bj; for(pos=0;pos<max_pos;pos++) _o[pos] = _i[pos] + _j[pos];
#define INCREMENT(inc)    _bo+=inc; _bi+=inc; _bj+=inc;

  int    pos, **_bo, **_bi, **_bj;
  register int *_o,   *_i,   *_j;

  //--- blocktype 6 ---
  _bo = BlockSAD[list][refindex][6];
  _bi = BlockSAD[list][refindex][7];
  _bj = _bi + 4;
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(5);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS();

  //--- blocktype 5 ---
  _bo = BlockSAD[list][refindex][5];
  _bi = BlockSAD[list][refindex][7];
  _bj = _bi + 1;
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 4 ---
  _bo = BlockSAD[list][refindex][4];
  _bi = BlockSAD[list][refindex][6];
  _bj = _bi + 1;
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(6);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 3 ---
  _bo = BlockSAD[list][refindex][3];
  _bi = BlockSAD[list][refindex][4];
  _bj = _bi + 8;
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 2 ---
  _bo = BlockSAD[list][refindex][2];
  _bi = BlockSAD[list][refindex][4];
  _bj = _bi + 2;
  ADD_UP_BLOCKS(); INCREMENT(8);
  ADD_UP_BLOCKS();

  //--- blocktype 1 ---
  _bo = BlockSAD[list][refindex][1];
  _bi = BlockSAD[list][refindex][3];
  _bj = _bi + 2;
  ADD_UP_BLOCKS();
}


/*!
 ***********************************************************************
 * \brief
 *    Setup the fast search for an macroblock
 ***********************************************************************
 */
void SetupFastFullPelSearch (int ref, int list)  // <--  reference frame parameter, list0 or 1
{
  int     pmv[2];
  pel_t   orig_blocks[256], *orgptr=orig_blocks, *refptr;
  int     offset_x, offset_y, x, y, range_partly_outside, ref_x, ref_y, pos, abs_x, abs_y, bindex, blky;
  int     LineSadBlk0, LineSadBlk1, LineSadBlk2, LineSadBlk3;
  int     max_width, max_height;
  int     img_width, img_height;

  StorablePicture *ref_picture;
  pel_t   *ref_pic;

  int**   block_sad     = BlockSAD[list][ref][7];
  int     search_range  = max_search_range[list][ref];
  int     max_pos       = (2*search_range+1) * (2*search_range+1);

  int     list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  int     apply_weights = ( (active_pps->weighted_pred_flag && (img->type == P_SLICE || img->type == SP_SLICE)) ||
                            (active_pps->weighted_bipred_idc && (img->type == B_SLICE)));

  
  ref_picture     = listX[list+list_offset][ref];

  if (apply_weights)
    ref_pic       = ref_picture->imgY_11_w;
  else
    ref_pic       = ref_picture->imgY_11;

  max_width     = ref_picture->size_x - 17;
  max_height    = ref_picture->size_y - 17;
  
  img_width     = ref_picture->size_x;
  img_height    = ref_picture->size_y;

  //===== get search center: predictor of 16x16 block =====
  SetMotionVectorPredictor (pmv, enc_picture->ref_idx, enc_picture->mv, ref, list, 0, 0, 16, 16);
  search_center_x[list][ref] = pmv[0] / 4;
  search_center_y[list][ref] = pmv[1] / 4;

  if (!input->rdopt)
  {
    //--- correct center so that (0,0) vector is inside ---
    search_center_x[list][ref] = max(-search_range, min(search_range, search_center_x[list][ref]));
    search_center_y[list][ref] = max(-search_range, min(search_range, search_center_y[list][ref]));
  }

  search_center_x[list][ref] += img->opix_x;
  search_center_y[list][ref] += img->opix_y;

  offset_x = search_center_x[list][ref];
  offset_y = search_center_y[list][ref];

  //===== copy original block for fast access =====
  for   (y = img->opix_y; y < img->opix_y+16; y++)
    for (x = img->opix_x; x < img->opix_x+16; x++)
      *orgptr++ = imgY_org [y][x];


  //===== check if whole search range is inside image =====
  if (offset_x >= search_range && offset_x <= max_width  - search_range &&
      offset_y >= search_range && offset_y <= max_height - search_range   )
  {
    range_partly_outside = 0; PelYline_11 = FastLine16Y_11;
  }
  else
  {
    range_partly_outside = 1;
  }

  //===== determine position of (0,0)-vector =====
  if (!input->rdopt)
  {
    ref_x = img->opix_x - offset_x;
    ref_y = img->opix_y - offset_y;

    for (pos = 0; pos < max_pos; pos++)
    {
      if (ref_x == spiral_search_x[pos] &&
          ref_y == spiral_search_y[pos])
      {
        pos_00[list][ref] = pos;
        break;
      }
    }
  }

  //===== loop over search range (spiral search): get blockwise SAD =====
  for (pos = 0; pos < max_pos; pos++)
  {
    abs_y = offset_y + spiral_search_y[pos];
    abs_x = offset_x + spiral_search_x[pos];

    if (range_partly_outside)
    {
      if (abs_y >= 0 && abs_y <= max_height &&
          abs_x >= 0 && abs_x <= max_width    )
      {
        PelYline_11 = FastLine16Y_11;
      }
      else
      {
        PelYline_11 = UMVLine16Y_11;
      }
    }

    orgptr = orig_blocks;
    bindex = 0;
    for (blky = 0; blky < 4; blky++)
    {
      LineSadBlk0 = LineSadBlk1 = LineSadBlk2 = LineSadBlk3 = 0;
      for (y = 0; y < 4; y++)
      {
        refptr = PelYline_11 (ref_pic, abs_y++, abs_x, img_height, img_width);

        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
      }
      block_sad[bindex++][pos] = LineSadBlk0;
      block_sad[bindex++][pos] = LineSadBlk1;
      block_sad[bindex++][pos] = LineSadBlk2;
      block_sad[bindex++][pos] = LineSadBlk3;
    }
  }


  //===== combine SAD's for larger block types =====
  SetupLargerBlocks (list, ref, max_pos);


  //===== set flag marking that search setup have been done =====
  search_setup_done[list][ref] = 1;
}
#endif // _FAST_FULL_ME_

/*!
 ************************************************************************
 * \brief
 *    Set motion vector predictor
 ************************************************************************
 */
void SetMotionVectorPredictor (int  pmv[2],
                               int  ***refPic,
                               int  ****tmp_mv,
                               int  ref_frame,
                               int  list,
                               int  block_x,
                               int  block_y,
                               int  blockshape_x,
                               int  blockshape_y)
{
  int mb_x                 = 4*block_x;
  int mb_y                 = 4*block_y;
  int mb_nr                = img->current_mb_nr;

  int mv_a, mv_b, mv_c, pred_vec=0;
  int mvPredType, rFrameL, rFrameU, rFrameUR;
  int hv;

  PixelPos block_a, block_b, block_c, block_d;

  int SAD_a=0, SAD_b=0, SAD_c=0, SAD_d=0;
  int temp_pred_SAD[2];

  if (input->FMEnable) pred_SAD_space=0;

  getLuma4x4Neighbour(mb_nr, block_x, block_y,           -1,  0, &block_a);
  getLuma4x4Neighbour(mb_nr, block_x, block_y,            0, -1, &block_b);
  getLuma4x4Neighbour(mb_nr, block_x, block_y, blockshape_x, -1, &block_c);
  getLuma4x4Neighbour(mb_nr, block_x, block_y,           -1, -1, &block_d);

  if (mb_y > 0)
  {
    if (mb_x < 8)  // first column of 8x8 blocks
    {
      if (mb_y==8)
      {
        if (blockshape_x == 16)      block_c.available  = 0;
        else                         block_c.available &= 1;
      }
      else
      {
        if (mb_x+blockshape_x != 8)  block_c.available &= 1;
        else                         block_c.available  = 0;
      }
    }
    else
    {
      if (mb_x+blockshape_x != 16)   block_c.available &= 1;
      else                           block_c.available  = 0;
    }
  }

  if (!block_c.available)
  {
    block_c=block_d;
  }

  mvPredType = MVPRED_MEDIAN;

  if (!img->MbaffFrameFlag)
  {
    rFrameL    = block_a.available    ? refPic[list][block_a.pos_x][block_a.pos_y] : -1;
    rFrameU    = block_b.available    ? refPic[list][block_b.pos_x][block_b.pos_y] : -1;
    rFrameUR   = block_c.available    ? refPic[list][block_c.pos_x][block_c.pos_y] : -1;
  }
  else
  {
    if (img->mb_data[img->current_mb_nr].mb_field)
    {
      rFrameL    = block_a.available    ? 
        img->mb_data[block_a.mb_addr].mb_field ? 
        refPic[list][block_a.pos_x][block_a.pos_y]:
        refPic[list][block_a.pos_x][block_a.pos_y] * 2: 
        -1;
      rFrameU    = block_b.available    ? 
        img->mb_data[block_b.mb_addr].mb_field ? 
        refPic[list][block_b.pos_x][block_b.pos_y]:
        refPic[list][block_b.pos_x][block_b.pos_y] * 2: 
        -1;
      rFrameUR    = block_c.available    ? 
        img->mb_data[block_c.mb_addr].mb_field ? 
        refPic[list][block_c.pos_x][block_c.pos_y]:
        refPic[list][block_c.pos_x][block_c.pos_y] * 2: 
        -1;
    }
    else
    {
      rFrameL    = block_a.available    ? 
        img->mb_data[block_a.mb_addr].mb_field ? 
        refPic[list][block_a.pos_x][block_a.pos_y] >>1:
        refPic[list][block_a.pos_x][block_a.pos_y] : 
        -1;
      rFrameU    = block_b.available    ? 
        img->mb_data[block_b.mb_addr].mb_field ? 
        refPic[list][block_b.pos_x][block_b.pos_y] >>1:
        refPic[list][block_b.pos_x][block_b.pos_y] : 
        -1;
      rFrameUR    = block_c.available    ? 
        img->mb_data[block_c.mb_addr].mb_field ? 
        refPic[list][block_c.pos_x][block_c.pos_y] >>1:
        refPic[list][block_c.pos_x][block_c.pos_y] : 
        -1;
    }
  }


  /* Prediction if only one of the neighbors uses the reference frame
   * we are checking
   */
  if(rFrameL == ref_frame && rFrameU != ref_frame && rFrameUR != ref_frame)       mvPredType = MVPRED_L;
  else if(rFrameL != ref_frame && rFrameU == ref_frame && rFrameUR != ref_frame)  mvPredType = MVPRED_U;
  else if(rFrameL != ref_frame && rFrameU != ref_frame && rFrameUR == ref_frame)  mvPredType = MVPRED_UR;
  // Directional predictions 
  if(blockshape_x == 8 && blockshape_y == 16)
  {
    if(mb_x == 0)
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;
    }
    else
    {
      if( rFrameUR == ref_frame)
        mvPredType = MVPRED_UR;
    }
  }
  else if(blockshape_x == 16 && blockshape_y == 8)
  {
    if(mb_y == 0)
    {
      if(rFrameU == ref_frame)
        mvPredType = MVPRED_U;
    }
    else
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;
    }
  }

  for (hv=0; hv < 2; hv++)
  {
    if (!img->MbaffFrameFlag || hv==0)
    {
      mv_a = block_a.available  ? tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] : 0;
      mv_b = block_b.available  ? tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] : 0;
      mv_c = block_c.available  ? tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] : 0;
    }
    else
    {
      if (img->mb_data[img->current_mb_nr].mb_field)
      {
        mv_a = block_a.available  ? img->mb_data[block_a.mb_addr].mb_field?
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv]:
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] / 2: 
          0;
        mv_b = block_b.available  ? img->mb_data[block_b.mb_addr].mb_field?
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv]:
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] / 2: 
          0;
        mv_c = block_c.available  ? img->mb_data[block_c.mb_addr].mb_field?
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv]:
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] / 2: 
          0;
      }
      else
      {
        mv_a = block_a.available  ? img->mb_data[block_a.mb_addr].mb_field?
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] * 2:
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv]: 
          0;
        mv_b = block_b.available  ? img->mb_data[block_b.mb_addr].mb_field?
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] * 2:
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv]: 
          0;
        mv_c = block_c.available  ? img->mb_data[block_c.mb_addr].mb_field?
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] * 2:
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv]: 
          0;
      }
    }

  if(input->FMEnable)
  {
    SAD_a = block_a.available ? ((list==1) ? all_bwmincost[block_a.pos_x][block_a.pos_y][0][FME_blocktype][0]:all_mincost[block_a.pos_x][block_a.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_b = block_b.available ? ((list==1) ? all_bwmincost[block_b.pos_x][block_b.pos_y][0][FME_blocktype][0]:all_mincost[block_b.pos_x][block_b.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_d = block_d.available ? ((list==1) ? all_bwmincost[block_d.pos_x][block_d.pos_y][0][FME_blocktype][0]:all_mincost[block_d.pos_x][block_d.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_c = block_c.available ? ((list==1) ? all_bwmincost[block_c.pos_x][block_c.pos_y][0][FME_blocktype][0]:all_mincost[block_c.pos_x][block_c.pos_y][ref_frame][FME_blocktype][0]):SAD_d;
  }

    switch (mvPredType)
    {
    case MVPRED_MEDIAN:
      if(!(block_b.available || block_c.available))
      {
        pred_vec = mv_a;
        if(input->FMEnable) temp_pred_SAD[hv] = SAD_a;
      }
      else
      {
        pred_vec = mv_a+mv_b+mv_c-min(mv_a,min(mv_b,mv_c))-max(mv_a,max(mv_b,mv_c));
      }
      if(input->FMEnable)
      {
         if (pred_vec == mv_a && SAD_a != 0) temp_pred_SAD[hv] = SAD_a;
         else if (pred_vec == mv_b && SAD_b!=0) temp_pred_SAD[hv] = SAD_b;
              else temp_pred_SAD[hv] = SAD_c;
      }
      break;
    case MVPRED_L:
      pred_vec = mv_a;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_a;
      break;
    case MVPRED_U:
      pred_vec = mv_b;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_b;
      break;
    case MVPRED_UR:
      pred_vec = mv_c;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_c;
      break;
    default:
      break;
    }

    pmv[hv] = pred_vec;

  }

  if(input->FMEnable) pred_SAD_space = temp_pred_SAD[0]>temp_pred_SAD[1]?temp_pred_SAD[1]:temp_pred_SAD[0];
}

/*!
 ************************************************************************
 * \brief
 *    Initialize the motion search
 ************************************************************************
 */
void
Init_Motion_Search_Module ()
{
  int bits, i, imin, imax, k, l;

  int search_range               = input->search_range;
  int number_of_reference_frames = img->max_num_references;
  int max_search_points          = (2*search_range+1)*(2*search_range+1);
  int max_ref_bits               = 1 + 2 * (int)floor(log(max(16,number_of_reference_frames+1)) / log(2) + 1e-10);
  int max_ref                    = (1<<((max_ref_bits>>1)+1))-1;
  int number_of_subpel_positions = 4 * (2*search_range+3);
  int max_mv_bits                = 3 + 2 * (int)ceil (log(number_of_subpel_positions+1) / log(2) + 1e-10);
  max_mvd                        = (1<<( max_mv_bits >>1)   )-1;


  //=====   CREATE ARRAYS   =====
  //-----------------------------
  if ((spiral_search_x = (int*)calloc(max_search_points, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: spiral_search_x");
  if ((spiral_search_y = (int*)calloc(max_search_points, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: spiral_search_y");
  if ((mvbits = (int*)calloc(2*max_mvd+1, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: mvbits");
  if ((refbits = (int*)calloc(max_ref, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: refbits");
  if ((byte_abs = (int*)calloc(512, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: byte_abs");

  get_mem4Dint (&motion_cost, 8, 2, img->max_num_references+1, 4);

  //--- set array offsets ---
  mvbits   += max_mvd;
  byte_abs += 256;


  //=====   INIT ARRAYS   =====
  //---------------------------
  //--- init array: motion vector bits ---
  mvbits[0] = 1;
  for (bits=3; bits<=max_mv_bits; bits+=2)
  {
    imax = 1    << (bits >> 1);
    imin = imax >> 1;

    for (i = imin; i < imax; i++)   mvbits[-i] = mvbits[i] = bits;
  }
  //--- init array: reference frame bits ---
  refbits[0] = 1;
  for (bits=3; bits<=max_ref_bits; bits+=2)
  {
    imax = (1   << ((bits >> 1) + 1)) - 1;
    imin = imax >> 1;

    for (i = imin; i < imax; i++)   refbits[i] = bits;
  }
  //--- init array: absolute value ---
  byte_abs[0] = 0;
  for (i=1; i<256; i++)   byte_abs[i] = byte_abs[-i] = i;
  //--- init array: search pattern ---
  spiral_search_x[0] = spiral_search_y[0] = 0;
  for (k=1, l=1; l<=max(1,search_range); l++)
  {
    for (i=-l+1; i< l; i++)
    {
      spiral_search_x[k] =  i;  spiral_search_y[k++] = -l;
      spiral_search_x[k] =  i;  spiral_search_y[k++] =  l;
    }
    for (i=-l;   i<=l; i++)
    {
      spiral_search_x[k] = -l;  spiral_search_y[k++] =  i;
      spiral_search_x[k] =  l;  spiral_search_y[k++] =  i;
    }
  }

#ifdef _FAST_FULL_ME_
  if(!input->FMEnable)
    InitializeFastFullIntegerSearch ();
#endif
}


/*!
 ************************************************************************
 * \brief
 *    Free memory used by motion search
 ************************************************************************
 */
void
Clear_Motion_Search_Module ()
{
  //--- correct array offset ---
  mvbits   -= max_mvd;
  byte_abs -= 256;

  //--- delete arrays ---
  free (spiral_search_x);
  free (spiral_search_y);
  free (mvbits);
  free (refbits);
  free (byte_abs);
  free_mem4Dint (motion_cost, 8, 2);

#ifdef _FAST_FULL_ME_
  if(!input->FMEnable)
    ClearFastFullIntegerSearch ();
#endif
}



/*!
 ***********************************************************************
 * \brief
 *    Full pixel block motion search
 ***********************************************************************
 */
int                                               //  ==> minimum motion cost after search
FullPelBlockMotionSearch (pel_t**   orig_pic,     // <--  original pixel values for the AxB block
                          int       ref,          // <--  reference frame (0... or -1 (backward))
                          int       list,
                          int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB block
                          int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
                          int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
                          int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
                          int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
                          int*      mv_x,         // <--> in: search center (x) / out: motion vector (x) - in pel units
                          int*      mv_y,         // <--> in: search center (y) / out: motion vector (y) - in pel units
                          int       search_range, // <--  1-d search range in pel units
                          int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
                          double    lambda)       // <--  lagrangian parameter for determining motion cost
{
  int   pos, cand_x, cand_y, y, x4, mcost;
  
  pel_t *orig_line, *ref_line;
  pel_t *(*get_ref_line)(int, pel_t*, int, int, int, int);

  int   list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;
  pel_t *ref_pic			= listX[list+list_offset][ref]->imgY_11;
  int   img_width     = listX[list+list_offset][ref]->size_x;
  int   img_height    = listX[list+list_offset][ref]->size_y;

  int   best_pos      = 0;                                        // position with minimum motion cost
  int   max_pos       = (2*search_range+1)*(2*search_range+1);    // number of search positions
  int   lambda_factor = LAMBDA_FACTOR (lambda);                   // factor for determining lagragian motion cost
  int   blocksize_y   = input->blc_size[blocktype][1];            // vertical block size
  int   blocksize_x   = input->blc_size[blocktype][0];            // horizontal block size
  int   blocksize_x4  = blocksize_x >> 2;                         // horizontal block size in 4-pel units
  int   pred_x        = (pic_pix_x << 2) + pred_mv_x;       // predicted position x (in sub-pel units)
  int   pred_y        = (pic_pix_y << 2) + pred_mv_y;       // predicted position y (in sub-pel units)
  int   center_x      = pic_pix_x + *mv_x;                        // center position x (in pel units)
  int   center_y      = pic_pix_y + *mv_y;                        // center position y (in pel units)
  int   check_for_00  = (blocktype==1 && !input->rdopt && img->type!=B_SLICE && ref==0);

  //===== set function for getting reference picture lines =====
  if ((center_x > search_range) && (center_x < img->width -1-search_range-blocksize_x) &&
      (center_y > search_range) && (center_y < img->height-1-search_range-blocksize_y)   )
  {
     get_ref_line = FastLineX;
  }
  else
  {
     get_ref_line = UMVLineX;
  }


  //===== loop over all search positions =====
  for (pos=0; pos<max_pos; pos++)
  {
    //--- set candidate position (absolute position in pel units) ---
    cand_x = center_x + spiral_search_x[pos];
    cand_y = center_y + spiral_search_y[pos];

    //--- initialize motion cost (cost for motion vector) and check ---
    mcost = MV_COST (lambda_factor, 2, cand_x, cand_y, pred_x, pred_y);
    if (check_for_00 && cand_x==pic_pix_x && cand_y==pic_pix_y)
    {
      mcost -= WEIGHTED_COST (lambda_factor, 16);
    }
    if (mcost >= min_mcost)   continue;

    //--- add residual cost to motion cost ---
    for (y=0; y<blocksize_y; y++)
    {
      ref_line  = get_ref_line (blocksize_x, ref_pic, cand_y+y, cand_x, img_height, img_width);
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

    //--- check if motion cost is less than minimum cost ---
    if (mcost < min_mcost)
    {
      best_pos  = pos;
      min_mcost = mcost;
    }
  }


  //===== set best motion vector and return minimum motion cost =====
  if (best_pos)
  {
    *mv_x += spiral_search_x[best_pos];
    *mv_y += spiral_search_y[best_pos];
  }
  return min_mcost;
}


#ifdef _FAST_FULL_ME_
/*!
 ***********************************************************************
 * \brief
 *    Fast Full pixel block motion search
 ***********************************************************************
 */
int                                                   //  ==> minimum motion cost after search
FastFullPelBlockMotionSearch (pel_t**   orig_pic,     // <--  not used
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
  int   pos, offset_x, offset_y, cand_x, cand_y, mcost;

  int   max_pos       = (2*search_range+1)*(2*search_range+1);              // number of search positions
  int   lambda_factor = LAMBDA_FACTOR (lambda);                             // factor for determining lagragian motion cost
  int   best_pos      = 0;                                                  // position with minimum motion cost
  int   block_index;                                                        // block index for indexing SAD array
  int*  block_sad;                                                          // pointer to SAD array

  block_index   = (pic_pix_y-img->opix_y)+((pic_pix_x-img->opix_x)>>2); // block index for indexing SAD array
  block_sad     = BlockSAD[list][ref][blocktype][block_index];         // pointer to SAD array

  //===== set up fast full integer search if needed / set search center =====
  if (!search_setup_done[list][ref])
  {
    SetupFastFullPelSearch (ref, list);
  }

  offset_x = search_center_x[list][ref] - img->opix_x;
  offset_y = search_center_y[list][ref] - img->opix_y;

  //===== cost for (0,0)-vector: it is done before, because MVCost can be negative =====
  if (!input->rdopt)
  {
    mcost = block_sad[pos_00[list][ref]] + MV_COST (lambda_factor, 2, 0, 0, pred_mv_x, pred_mv_y);

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos_00[list][ref];
    }
  }

  //===== loop over all search positions =====
  for (pos=0; pos<max_pos; pos++, block_sad++)
  {
    //--- check residual cost ---
    if (*block_sad < min_mcost)
    {
      //--- get motion vector cost ---
      cand_x = offset_x + spiral_search_x[pos];
      cand_y = offset_y + spiral_search_y[pos];
      mcost  = *block_sad;
      mcost += MV_COST (lambda_factor, 2, cand_x, cand_y, pred_mv_x, pred_mv_y);

      //--- check motion cost ---
      if (mcost < min_mcost)
      {
        min_mcost = mcost;
        best_pos  = pos;
      }
    }
  }

  //===== set best motion vector and return minimum motion cost =====
  *mv_x = offset_x + spiral_search_x[best_pos];
  *mv_y = offset_y + spiral_search_y[best_pos];
  return min_mcost;
}
#endif


/*!
 ***********************************************************************
 * \brief
 *    Calculate SA(T)D
 ***********************************************************************
 */
int
SATD (int* diff, int use_hadamard)
{
  int k, satd = 0, m[16], dd, *d=diff;
  
  if (use_hadamard)
  {
    /*===== hadamard transform =====*/
    m[ 0] = d[ 0] + d[12];
    m[ 4] = d[ 4] + d[ 8];
    m[ 8] = d[ 4] - d[ 8];
    m[12] = d[ 0] - d[12];
    m[ 1] = d[ 1] + d[13];
    m[ 5] = d[ 5] + d[ 9];
    m[ 9] = d[ 5] - d[ 9];
    m[13] = d[ 1] - d[13];
    m[ 2] = d[ 2] + d[14];
    m[ 6] = d[ 6] + d[10];
    m[10] = d[ 6] - d[10];
    m[14] = d[ 2] - d[14];
    m[ 3] = d[ 3] + d[15];
    m[ 7] = d[ 7] + d[11];
    m[11] = d[ 7] - d[11];
    m[15] = d[ 3] - d[15];
    
    d[ 0] = m[ 0] + m[ 4];
    d[ 8] = m[ 0] - m[ 4];
    d[ 4] = m[ 8] + m[12];
    d[12] = m[12] - m[ 8];
    d[ 1] = m[ 1] + m[ 5];
    d[ 9] = m[ 1] - m[ 5];
    d[ 5] = m[ 9] + m[13];
    d[13] = m[13] - m[ 9];
    d[ 2] = m[ 2] + m[ 6];
    d[10] = m[ 2] - m[ 6];
    d[ 6] = m[10] + m[14];
    d[14] = m[14] - m[10];
    d[ 3] = m[ 3] + m[ 7];
    d[11] = m[ 3] - m[ 7];
    d[ 7] = m[11] + m[15];
    d[15] = m[15] - m[11];
    
    m[ 0] = d[ 0] + d[ 3];
    m[ 1] = d[ 1] + d[ 2];
    m[ 2] = d[ 1] - d[ 2];
    m[ 3] = d[ 0] - d[ 3];
    m[ 4] = d[ 4] + d[ 7];
    m[ 5] = d[ 5] + d[ 6];
    m[ 6] = d[ 5] - d[ 6];
    m[ 7] = d[ 4] - d[ 7];
    m[ 8] = d[ 8] + d[11];
    m[ 9] = d[ 9] + d[10];
    m[10] = d[ 9] - d[10];
    m[11] = d[ 8] - d[11];
    m[12] = d[12] + d[15];
    m[13] = d[13] + d[14];
    m[14] = d[13] - d[14];
    m[15] = d[12] - d[15];
    
    d[ 0] = m[ 0] + m[ 1];
    d[ 1] = m[ 0] - m[ 1];
    d[ 2] = m[ 2] + m[ 3];
    d[ 3] = m[ 3] - m[ 2];
    d[ 4] = m[ 4] + m[ 5];
    d[ 5] = m[ 4] - m[ 5];
    d[ 6] = m[ 6] + m[ 7];
    d[ 7] = m[ 7] - m[ 6];
    d[ 8] = m[ 8] + m[ 9];
    d[ 9] = m[ 8] - m[ 9];
    d[10] = m[10] + m[11];
    d[11] = m[11] - m[10];
    d[12] = m[12] + m[13];
    d[13] = m[12] - m[13];
    d[14] = m[14] + m[15];
    d[15] = m[15] - m[14];
    
    /*===== sum up =====*/
    for (dd=diff[k=0]; k<16; dd=diff[++k])
    {
      satd += (dd < 0 ? -dd : dd);
    }
    satd >>= 1;
  }
  else
  {
    /*===== sum up =====*/
    for (k = 0; k < 16; k++)
    {
      satd += byte_abs [diff [k]];
    }
  }
  
  return satd;
}



/*!
 ***********************************************************************
 * \brief
 *    Sub pixel block motion search
 ***********************************************************************
 */
int                                               //  ==> minimum motion cost after search
SubPelBlockMotionSearch (pel_t**   orig_pic,      // <--  original pixel values for the AxB block
                         int       ref,           // <--  reference frame (0... or -1 (backward))
                         int       list,          // <--  reference picture list 
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
                         double    lambda         // <--  lagrangian parameter for determining motion cost
                         )
{
  int   diff[16], *d;
  int   pos, best_pos, mcost, abort_search;
  int   y0, x0, ry0, rx0, ry;
  int   cand_mv_x, cand_mv_y;
  int   max_pos_x4, max_pos_y4;
  pel_t *orig_line;
  pel_t **ref_pic;      
  StorablePicture *ref_picture;
  int   lambda_factor   = LAMBDA_FACTOR (lambda);
  int   mv_shift        = 0;
  int   check_position0 = (blocktype==1 && *mv_x==0 && *mv_y==0 && input->hadamard && !input->rdopt && img->type!=B_SLICE && ref==0);
  int   blocksize_x     = input->blc_size[blocktype][0];
  int   blocksize_y     = input->blc_size[blocktype][1];
  int   pic4_pix_x      = (pic_pix_x << 2);
  int   pic4_pix_y      = (pic_pix_y << 2);
  int   min_pos2        = (input->hadamard ? 0 : 1);
  int   max_pos2        = (input->hadamard ? max(1,search_pos2) : search_pos2);
  int   list_offset     = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  int  apply_weights = ( (active_pps->weighted_pred_flag && (img->type == P_SLICE || img->type == SP_SLICE)) ||
                         (active_pps->weighted_bipred_idc && (img->type == B_SLICE)));  

  int   img_width, img_height;
  
  ref_picture     = listX[list+list_offset][ref];

  if (apply_weights)
  {
    ref_pic = listX[list+list_offset][ref]->imgY_ups_w;
  }
  else
    ref_pic = listX[list+list_offset][ref]->imgY_ups;

  img_width  = ref_picture->size_x;
  img_height = ref_picture->size_y;

  max_pos_x4      = ((ref_picture->size_x - blocksize_x+1)<<2);
  max_pos_y4      = ((ref_picture->size_y - blocksize_y+1)<<2);
  
  /*********************************
   *****                       *****
   *****  HALF-PEL REFINEMENT  *****
   *****                       *****
   *********************************/
  //===== convert search center to quarter-pel units =====
  *mv_x <<= 2;
  *mv_y <<= 2;
  //===== set function for getting pixel values =====
  if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 2) &&
      (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 2)   )
  {
    PelY_14 = FastPelY_14;
  }
  else
  {
    PelY_14 = UMVPelY_14;
  }
  //===== loop over search positions =====
  for (best_pos = 0, pos = min_pos2; pos < max_pos2; pos++)
  {
    cand_mv_x = *mv_x + (spiral_search_x[pos] << 1);    // quarter-pel units
    cand_mv_y = *mv_y + (spiral_search_y[pos] << 1);    // quarter-pel units

    //----- set motion vector cost -----
    mcost = MV_COST (lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);
    if (check_position0 && pos==0)
    {
      mcost -= WEIGHTED_COST (lambda_factor, 16);
    }

    if (mcost >= min_mcost) continue;

    //----- add up SATD -----
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
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+1];    ry=ry0+4;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+2];    ry=ry0+8;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+3];    ry=ry0+12;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d        = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        if ((mcost += SATD (diff, input->hadamard)) > min_mcost)
        {
          abort_search = 1;
          break;
        }
      }
    }

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
  }
  if (best_pos)
  {
    *mv_x += (spiral_search_x [best_pos] << 1);
    *mv_y += (spiral_search_y [best_pos] << 1);
  }


  /************************************
   *****                          *****
   *****  QUARTER-PEL REFINEMENT  *****
   *****                          *****
   ************************************/
  //===== set function for getting pixel values =====
  if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 1) &&
      (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 1)   )
  {
    PelY_14 = FastPelY_14;
  }
  else
  {
    PelY_14 = UMVPelY_14;
  }
  //===== loop over search positions =====
  for (best_pos = 0, pos = 1; pos < search_pos4; pos++)
  {
    cand_mv_x = *mv_x + spiral_search_x[pos];    // quarter-pel units
    cand_mv_y = *mv_y + spiral_search_y[pos];    // quarter-pel units

    //----- set motion vector cost -----
    mcost = MV_COST (lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

    if (mcost >= min_mcost) continue;

    //----- add up SATD -----
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
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+1];    ry=ry0+4;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+2];    ry=ry0+8;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+3];    ry=ry0+12;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d        = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        if ((mcost += SATD (diff, input->hadamard)) > min_mcost)
        {
          abort_search = 1;
          break;
        }
      }
    }

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
  }
  if (best_pos)
  {
    *mv_x += spiral_search_x [best_pos];
    *mv_y += spiral_search_y [best_pos];
  }

  //===== return minimum motion cost =====
  return min_mcost;
}



/*!
 ***********************************************************************
 * \brief
 *    Block motion search
 ***********************************************************************
 */
int                                         //!< minimum motion cost after search
BlockMotionSearch (int       ref,           //!< reference idx
                   int       list,          //!< reference pciture list
                   int       mb_x,          //!< x-coordinate inside macroblock
                   int       mb_y,          //!< y-coordinate inside macroblock
                   int       blocktype,     //!< block type (1-16x16 ... 7-4x4)
                   int       search_range,  //!< 1-d search range for integer-position search
                   double    lambda         //!< lagrangian parameter for determining motion cost
                   )
{
  static pel_t   orig_val [256];
  static pel_t  *orig_pic  [16] = {orig_val,     orig_val+ 16, orig_val+ 32, orig_val+ 48,
                                   orig_val+ 64, orig_val+ 80, orig_val+ 96, orig_val+112,
                                   orig_val+128, orig_val+144, orig_val+160, orig_val+176,
                                   orig_val+192, orig_val+208, orig_val+224, orig_val+240};

  int       pred_mv_x, pred_mv_y, mv_x, mv_y, i, j;

  int       max_value = (1<<20);
  int       min_mcost = max_value;

  int       block_x   = (mb_x>>2);
  int       block_y   = (mb_y>>2);
  
  int       bsx       = input->blc_size[blocktype][0];
  int       bsy       = input->blc_size[blocktype][1];

  int       pic_pix_x = img->opix_x + mb_x;
  int       pic_pix_y = img->opix_y + mb_y;

  int*      pred_mv;

  int***    mv_array  = enc_picture->mv[list];

  int****** all_mv    = img->all_mv;

#ifdef WIN32
  struct _timeb tstruct1;
  struct _timeb tstruct2;
#else
  struct timeb tstruct1;
  struct timeb tstruct2;
#endif
  
  int me_tmp_time;

  int  N_Bframe=0, n_Bframe=0;
  if(input->FMEnable)
  {
    N_Bframe = input->successive_Bframe;
    n_Bframe =(N_Bframe) ? ((Bframe_ctr%N_Bframe)+1) : 0 ;
  }

   pred_mv = img->pred_mv[block_x][block_y][list][ref][blocktype];

  //==================================
  //=====   GET ORIGINAL BLOCK   =====
  //==================================
  for (j = 0; j < bsy; j++)
  {
    for (i = 0; i < bsx; i++)
    {
      orig_pic[j][i] = imgY_org[pic_pix_y+j][pic_pix_x+i];
    }
  }

  if(input->FMEnable)
  {
    
    if(blocktype>6)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][5][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][5][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][5][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][5][0]);
      pred_SAD_uplayer   /= 2; 
      
    }
    else if(blocktype>4)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][4][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][4][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][4][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][4][0]);
      pred_SAD_uplayer   /= 2; 
      
    }
    else if(blocktype == 4)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][2][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][2][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][2][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][2][0]);
      pred_SAD_uplayer   /= 2; 
    }
    else if(blocktype > 1)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][1][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][1][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][1][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][1][0]);
      pred_SAD_uplayer   /= 2; 
    }
    
    if ((img->type==B_SLICE)&& (img->nal_reference_idc>0))
    {
      if(blocktype>6)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][5][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][5][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][5][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][5][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype>4)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][4][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][4][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][4][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][4][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype == 4)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][2][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][2][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][2][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][2][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype > 1)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][1][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][1][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][1][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][1][0]);
        pred_SAD_uplayer   /= 2; 
      }
    }
    
    pred_SAD_uplayer = flag_intra_SAD ? 0 : pred_SAD_uplayer;// for irregular motion
    
    //Coordinate prediction
    if (img->number > ref+1)
    {
      pred_SAD_time = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][0];
      pred_MV_time[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1];
      pred_MV_time[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2];
    }
    
    if(list==1 && (Bframe_ctr%N_Bframe) > 1) 
    {
      pred_SAD_time = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][0];
      pred_MV_time[0] = (int)(all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][1] * ((n_Bframe==1) ? (N_Bframe) : (N_Bframe-n_Bframe+1.0)/(N_Bframe-n_Bframe+2.0)) );//should add a factor
      pred_MV_time[1] = (int)(all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][2] *((n_Bframe==1) ? (N_Bframe) : (N_Bframe-n_Bframe+1.0)/(N_Bframe-n_Bframe+2.0)) );//should add a factor
    }
    
    if (input->PicInterlace == FIELD_CODING) 
    {
      if (img->type == P_SLICE && ref > 1)
      {
        pred_SAD_ref = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][1];
        pred_MV_ref[0] = (int)(pred_MV_ref[0]*((ref>>1)+1)/(float)((ref>>1)));
        pred_MV_ref[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][2];
        pred_MV_ref[1] = (int)(pred_MV_ref[1]*((ref>>1)+1)/(float)((ref>>1)));
      }
      if (img->type == B_SLICE && list==0 && (ref==0 || ref==1) )
      {
        pred_SAD_ref = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] =(int) (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); //should add a factor
        pred_MV_ref[1] =(int) ( all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); 
      }
    }
    else //frame case
    {
      if (ref > 0)
      {//field_mode top_field
        pred_SAD_ref = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][1];
        pred_MV_ref[0] = (int)(pred_MV_ref[0]*(ref+1)/(float)(ref));
        pred_MV_ref[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][2];
        pred_MV_ref[1] = (int)(pred_MV_ref[1]*(ref+1)/(float)(ref));
      }
      if (img->type == B_SLICE && (list==0 && ref==0)) //B frame forward prediction, first ref
      {
        pred_SAD_ref = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] =(int) (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); //should add a factor
        pred_MV_ref[1] =(int) ( all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); 
      }
    }
 }
 

  //===========================================
  //=====   GET MOTION VECTOR PREDICTOR   =====
  //===========================================

  if (input->FMEnable) 
    FME_blocktype=blocktype;

  SetMotionVectorPredictor (pred_mv, enc_picture->ref_idx, enc_picture->mv, ref, list, block_x, block_y, bsx, bsy);

  pred_mv_x = pred_mv[0];
  pred_mv_y = pred_mv[1];
#ifdef WIN32
  _ftime( &tstruct1 );    // start time ms
#else
  ftime(&tstruct1);
#endif

  //==================================
  //=====   INTEGER-PEL SEARCH   =====
  //==================================

  if(input->FMEnable)
  {
    mv_x = pred_mv_x / 4;
    mv_y = pred_mv_y / 4;
    if (!input->rdopt)
    {
      //--- adjust search center so that the (0,0)-vector is inside ---
      mv_x = max (-search_range, min (search_range, mv_x));
      mv_y = max (-search_range, min (search_range, mv_y));
    }
    
    min_mcost = FastIntegerPelBlockMotionSearch(orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
      pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
      min_mcost, lambda);
    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        if(list == 0) 
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][0] = min_mcost;
        else
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][0] = min_mcost; 
      }
    }
  }
  else
  {
#ifndef _FAST_FULL_ME_

    //--- set search center ---
    mv_x = pred_mv_x / 4;
    mv_y = pred_mv_y / 4;
    if (!input->rdopt)
    {
      //--- adjust search center so that the (0,0)-vector is inside ---
      mv_x = max (-search_range, min (search_range, mv_x));
      mv_y = max (-search_range, min (search_range, mv_y));
    }
    
    //--- perform motion search ---
    min_mcost = FullPelBlockMotionSearch     (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                              pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
                                              min_mcost, lambda);

#else

    // comments:   - orig_pic is not used  -> be careful
    //             - search center is automatically determined
    min_mcost = FastFullPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                              pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
                                              min_mcost, lambda);

#endif
  }

#ifdef WIN32
      _ftime(&tstruct2);   // end time ms
#else
      ftime(&tstruct2);    // end time ms
#endif
      
      me_tmp_time=(tstruct2.time*1000+tstruct2.millitm) - (tstruct1.time*1000+tstruct1.millitm); 
      me_tot_time += me_tmp_time;
      me_time += me_tmp_time;

  //==============================
  //=====   SUB-PEL SEARCH   =====
  //==============================
  if (input->hadamard)
  {
    min_mcost = max_value;
  }

  if(input->FMEnable)
  {
    if(blocktype >3)
    {
      min_mcost =  FastSubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                                pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                                min_mcost, lambda, /*useABT*/0);
    }
    else
    {
      min_mcost =  SubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                            pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                            min_mcost, lambda);
    }


    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        if(list == 0)
        {
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][1] = mv_x;
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][2] = mv_y;
        }
        else
        {
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][1] = mv_x;
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][2] = mv_y;
          
        }
      }
    }
  }
  else
  {
    min_mcost =  SubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                          pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                          min_mcost, lambda);
  }


  if (!input->rdopt)
  {
    // Get the skip mode cost
    if (blocktype == 1 && (img->type == P_SLICE||img->type == SP_SLICE))
    {
      int cost;

      FindSkipModeMotionVector ();

      cost  = GetSkipCostMB (lambda);
      cost -= (int)floor(8*lambda+0.4999);

      if (cost < min_mcost)
      {
        min_mcost = cost;
        mv_x      = img->all_mv [0][0][0][0][0][0];
        mv_y      = img->all_mv [0][0][0][0][0][1];
      }
    }
  }

  //===============================================
  //=====   SET MV'S AND RETURN MOTION COST   =====
  //===============================================

  if(input->FMEnable)
  {
    int h4x4blkno = (img->pix_x>>2)+block_x;  
    int v4x4blkno = (img->pix_y>>2)+block_y;
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        all_mv[block_x+i][block_y+j][list][ref][blocktype][0] = mv_x;
        all_mv[block_x+i][block_y+j][list][ref][blocktype][1] = mv_y;
        mv_array[h4x4blkno+i][v4x4blkno+j][0] = mv_x;
        mv_array[h4x4blkno+i][v4x4blkno+j][1] = mv_y;
      }
    }
  }
  else
  {
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        all_mv[block_x+i][block_y+j][list][ref][blocktype][0] = mv_x;
        all_mv[block_x+i][block_y+j][list][ref][blocktype][1] = mv_y;
      }
    }
  }

  return min_mcost;
}


/*!
 ***********************************************************************
 * \brief
 *    Motion Cost for Bidirectional modes
 ***********************************************************************
 */
int BIDPartitionCost (int   blocktype,
                      int   block8x8,
                      int   fw_ref,
                      int   bw_ref,
                      int   lambda_factor)
{
  static int  bx0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,2,0,2}};
  static int  by0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,0,0,0}, {0,0,2,2}};

  int   diff[16];
  int   pic_pix_x, pic_pix_y, block_x, block_y;
  int   v, h, mcost, i, j, k;
  int   mvd_bits  = 0;
  int   parttype  = (blocktype<4?blocktype:4);
  int   step_h0   = (input->blc_size[ parttype][0]>>2);
  int   step_v0   = (input->blc_size[ parttype][1]>>2);
  int   step_h    = (input->blc_size[blocktype][0]>>2);
  int   step_v    = (input->blc_size[blocktype][1]>>2);
  int   bxx, byy;                               // indexing curr_blk

  int   ******all_mv = img->all_mv;
  int   ******  p_mv = img->pred_mv;

  //----- cost for motion vector bits -----
  for (v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; v+=step_v)
  for (h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; h+=step_h)
  {
    mvd_bits += mvbits[ all_mv [h][v][LIST_0][fw_ref][blocktype][0] - p_mv[h][v][LIST_0][fw_ref][blocktype][0] ];
    mvd_bits += mvbits[ all_mv [h][v][LIST_0][fw_ref][blocktype][1] - p_mv[h][v][LIST_0][fw_ref][blocktype][1] ];

    mvd_bits += mvbits[ all_mv [h][v][LIST_1][bw_ref][blocktype][0] - p_mv[h][v][LIST_1][bw_ref][blocktype][0] ];
    mvd_bits += mvbits[ all_mv [h][v][LIST_1][bw_ref][blocktype][1] - p_mv[h][v][LIST_1][bw_ref][blocktype][1] ];
  }

  mcost = WEIGHTED_COST (lambda_factor, mvd_bits);

  //----- cost of residual signal -----
  for (byy=0, v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; byy+=4, v++)
  {
    pic_pix_y = img->opix_y + (block_y = (v<<2));

    for (bxx=0, h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; bxx+=4, h++)
    {
      pic_pix_x = img->opix_x + (block_x = (h<<2));

      LumaPrediction4x4 (block_x, block_y, 2, blocktype, blocktype, fw_ref, bw_ref);

      for (k=j=0; j<4; j++)
      for (  i=0; i<4; i++, k++)
      {
        diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];
      }
      mcost += SATD (diff, input->hadamard);
    }
  }
  return mcost;
}

/*!
 ************************************************************************
 * \brief
 *    Get cost for skip mode for an macroblock
 ************************************************************************
 */
int GetSkipCostMB (double lambda)
{
  int block_y, block_x, pic_pix_y, pic_pix_x, i, j, k;
  int diff[16];
  int cost = 0;

  for (block_y=0; block_y<16; block_y+=4)
  {
    pic_pix_y = img->opix_y + block_y;

    for (block_x=0; block_x<16; block_x+=4)
    {
      pic_pix_x = img->opix_x + block_x;

      //===== prediction of 4x4 block =====
      LumaPrediction4x4 (block_x, block_y, 0, 0, 0, 0, 0);

      //===== get displaced frame difference ======                
      for (k=j=0; j<4; j++)
        for (i=0; i<4; i++, k++)
        {
          diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];
        }
      cost += SATD (diff, input->hadamard);
    }
  }

  return cost;
}

/*!
 ************************************************************************
 * \brief
 *    Find motion vector for the Skip mode
 ************************************************************************
 */
void FindSkipModeMotionVector ()
{
  int bx, by;
  int ******all_mv = img->all_mv;

  int pmv[2];

  int zeroMotionAbove;
  int zeroMotionLeft;
  PixelPos mb_a, mb_b;
  int      a_mv_y = 0;
  int      a_ref_idx = 0;
  int      b_mv_y = 0;
  int      b_ref_idx = 0;

  Macroblock *currMB = &img->mb_data[img->current_mb_nr];
  
  getLuma4x4Neighbour(img->current_mb_nr,0,0,-1, 0,&mb_a);
  getLuma4x4Neighbour(img->current_mb_nr,0,0, 0,-1,&mb_b);
  
  if (mb_a.available)
  {
    a_mv_y    = enc_picture->mv[LIST_0][mb_a.pos_x][mb_a.pos_y][1];
    a_ref_idx = enc_picture->ref_idx[LIST_0][mb_a.pos_x][mb_a.pos_y];
    
    if (currMB->mb_field && !img->mb_data[mb_a.mb_addr].mb_field)
    {
      a_mv_y    /=2;
      a_ref_idx *=2;
    }
    if (!currMB->mb_field && img->mb_data[mb_a.mb_addr].mb_field)
    {
      a_mv_y    *=2;
      a_ref_idx >>=1;
    }
  }
  
  if (mb_b.available)
  {
    b_mv_y    = enc_picture->mv[LIST_0][mb_b.pos_x][mb_b.pos_y][1];
    b_ref_idx = enc_picture->ref_idx[LIST_0][mb_b.pos_x][mb_b.pos_y];
    
    if (currMB->mb_field && !img->mb_data[mb_b.mb_addr].mb_field)
    {
      b_mv_y    /=2;
      b_ref_idx *=2;
    }
    if (!currMB->mb_field && img->mb_data[mb_b.mb_addr].mb_field)
    {
      b_mv_y    *=2;
      b_ref_idx >>=1;
    }
  }
  
  zeroMotionLeft  = !mb_a.available ? 1 : a_ref_idx==0 && enc_picture->mv[LIST_0][mb_a.pos_x][mb_a.pos_y][0]==0 && a_mv_y==0 ? 1 : 0;
  zeroMotionAbove = !mb_b.available ? 1 : b_ref_idx==0 && enc_picture->mv[LIST_0][mb_b.pos_x][mb_b.pos_y][0]==0 && b_mv_y==0 ? 1 : 0;
  
  if (zeroMotionAbove || zeroMotionLeft)
  {
    for (by = 0;by < 4;by++)
      for (bx = 0;bx < 4;bx++)
      {
        all_mv [bx][by][0][0][0][0] = 0;
        all_mv [bx][by][0][0][0][1] = 0;
      }
  }
  else
  {
    SetMotionVectorPredictor (pmv, enc_picture->ref_idx, enc_picture->mv, 0, LIST_0, 0, 0, 16, 16);
    for (by = 0;by < 4;by++)
      for (bx = 0;bx < 4;bx++)
      {
        all_mv [bx][by][0][0][0][0] = pmv[0];
        all_mv [bx][by][0][0][0][1] = pmv[1];
      }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Get cost for direct mode for an 8x8 block
 ************************************************************************
 */
int Get_Direct_Cost8x8 (int block, double lambda)
{
  int block_y, block_x, pic_pix_y, pic_pix_x, i, j, k;
  int diff[16];
  int cost  = 0;
  int mb_y  = (block/2)<<3;
  int mb_x  = (block%2)<<3;

  for (block_y=mb_y; block_y<mb_y+8; block_y+=4)
  {
    pic_pix_y = img->opix_y + block_y;

    for (block_x=mb_x; block_x<mb_x+8; block_x+=4)
    {
      pic_pix_x = img->opix_x + block_x;

      if (direct_pdir[pic_pix_x>>2][pic_pix_y>>2]<0)
      {
        return (1<<30); //mode not allowed
      }

      //===== prediction of 4x4 block =====
      LumaPrediction4x4 (block_x, block_y, direct_pdir[pic_pix_x>>2][pic_pix_y>>2], 0, 0, 
                         direct_ref_idx[LIST_0][pic_pix_x>>2][pic_pix_y>>2], 
                         direct_ref_idx[LIST_1][pic_pix_x>>2][pic_pix_y>>2]);

      //===== get displaced frame difference ======                
      for (k=j=0; j<4; j++)
        for (i=0; i<4; i++, k++)
        {
          diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];

        }
      cost += SATD (diff, input->hadamard);
    }
  }

  return cost;
}



/*!
 ************************************************************************
 * \brief
 *    Get cost for direct mode for an macroblock
 ************************************************************************
 */
int Get_Direct_CostMB (double lambda)
{
  int i;
  int cost = 0;
  
  for (i=0; i<4; i++)
  {
    cost += Get_Direct_Cost8x8 (i, lambda);
    if (cost >= (1<<30)) return cost;
  }
  return cost;
}


/*!
 ************************************************************************
 * \brief
 *    Motion search for a partition
 ************************************************************************
 */
void
PartitionMotionSearch (int    blocktype,
                       int    block8x8,
                       double lambda)
{
  static int  bx0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,2,0,2}};
  static int  by0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,0,0,0}, {0,0,2,2}};

  int   **ref_array, ***mv_array;
  int   ref, v, h, mcost, search_range, i, j;
  int   pic_block_x, pic_block_y;
  int   bslice    = (img->type==B_SLICE);
  int   parttype  = (blocktype<4?blocktype:4);
  int   step_h0   = (input->blc_size[ parttype][0]>>2);
  int   step_v0   = (input->blc_size[ parttype][1]>>2);
  int   step_h    = (input->blc_size[blocktype][0]>>2);
  int   step_v    = (input->blc_size[blocktype][1]>>2);
  int   list;
  int   numlists;
  int   list_offset;

  if (img->mb_data[img->current_mb_nr].mb_field)
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

  numlists=bslice?2:1;

  //===== LOOP OVER REFERENCE FRAMES =====
  for (list=0; list<numlists;list++)
  {
    for (ref=0; ref < listXsize[list+list_offset]; ref++)
    {
        //----- set search range ---
#ifdef _FULL_SEARCH_RANGE_
        if      (input->full_search == 2) search_range = input->search_range;
        else if (input->full_search == 1) search_range = input->search_range /  (min(ref,1)+1);
        else                              search_range = input->search_range / ((min(ref,1)+1) * min(2,blocktype));
#else
        search_range = input->search_range / ((min(ref,1)+1) * min(2,blocktype));
#endif
        
        //----- set arrays -----
        ref_array = enc_picture->ref_idx[list];
        mv_array  = enc_picture->mv[list];
        
        //----- init motion cost -----
        motion_cost[blocktype][list][ref][block8x8] = 0;
        
        //===== LOOP OVER SUB MACRO BLOCK partitions
        for (v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; v+=step_v)
        {
          pic_block_y = img->block_y + v;
          
          for (h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; h+=step_h)
          {
            pic_block_x = img->block_x + h;
            
            //--- motion search for block ---

            mcost = BlockMotionSearch     (ref, list, h<<2, v<<2, blocktype, search_range, lambda);
            motion_cost[blocktype][list][ref][block8x8] += mcost;
            
            //--- set motion vectors and reference frame (for motion vector prediction) ---
            for (j=0; j<step_v; j++)
              for (i=0; i<step_h; i++)
              {
                mv_array  [pic_block_x+i][pic_block_y+j][0] = img->all_mv[h][v][list][ref][blocktype][0];
                mv_array  [pic_block_x+i][pic_block_y+j][1] = img->all_mv[h][v][list][ref][blocktype][1];
                ref_array [pic_block_x+i][pic_block_y+j]    = ref;
              }
          }
        }
    }
  }
}





extern int* last_P_no;
/*********************************************
 *****                                   *****
 *****  Calculate Direct Motion Vectors  *****
 *****                                   *****
 *********************************************/
void Get_Direct_Motion_Vectors ()
{

  int  block_x, block_y, pic_block_x, pic_block_y, opic_block_x, opic_block_y;
  int  ******all_mvs = img->all_mv;
  int  mv_scale;
  byte **    moving_block;
  int ****   co_located_mv;
  int ***    co_located_ref_idx;
  int64 ***    co_located_ref_id;
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  if ((img->MbaffFrameFlag)&&(currMB->mb_field))
  {
    if(img->current_mb_nr%2)
    {
      moving_block = Co_located->bottom_moving_block;
      co_located_mv = Co_located->bottom_mv;
      co_located_ref_idx = Co_located->bottom_ref_idx;
      co_located_ref_id = Co_located->bottom_ref_pic_id;
    }
    else
    {
      moving_block = Co_located->top_moving_block;
      co_located_mv = Co_located->top_mv;
      co_located_ref_idx = Co_located->top_ref_idx;
      co_located_ref_id = Co_located->top_ref_pic_id;
    }
  }
  else
  {
    moving_block = Co_located->moving_block;
    co_located_mv = Co_located->mv;
    co_located_ref_idx = Co_located->ref_idx;
    co_located_ref_id = Co_located->ref_pic_id;
  }

  if (img->direct_type)  //spatial direct mode copy from decoder
  {
    
    int fw_rFrameL, fw_rFrameU, fw_rFrameUL, fw_rFrameUR;
    int bw_rFrameL, bw_rFrameU, bw_rFrameUL, bw_rFrameUR; 
    int fw_rFrame,bw_rFrame;
    int pmvfw[2]={0,0},pmvbw[2]={0,0};

    PixelPos mb_left, mb_up, mb_upleft, mb_upright;              
    
    getLuma4x4Neighbour(img->current_mb_nr,0,0,-1, 0,&mb_left);
    getLuma4x4Neighbour(img->current_mb_nr,0,0, 0,-1,&mb_up);
    getLuma4x4Neighbour(img->current_mb_nr,0,0,16, -1,&mb_upright);
    getLuma4x4Neighbour(img->current_mb_nr,0,0, -1,-1,&mb_upleft);

    if (!img->MbaffFrameFlag)
    {
      fw_rFrameL = mb_left.available ? enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] : -1;
      fw_rFrameU = mb_up.available ? enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : -1;
      fw_rFrameUL = mb_upleft.available ? enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : -1;
      fw_rFrameUR = mb_upright.available ? enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : fw_rFrameUL;      
      
      bw_rFrameL = mb_left.available ? enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : -1;
      bw_rFrameU = mb_up.available ? enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : -1;
      bw_rFrameUL = mb_upleft.available ? enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : -1;
      bw_rFrameUR = mb_upright.available ? enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : bw_rFrameUL;      
    }
    else
    {
      if (currMB->mb_field)
      {
        fw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field  || enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] < 0? 
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] : 
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] * 2: -1;

        fw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] < 0? 
          enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] * 2: -1;

        fw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] < 0?         
          enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] *2: -1;      

        fw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] < 0?
          enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] * 2: fw_rFrameUL;      
        
        bw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] < 0? 
          enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] * 2: -1;

        bw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] < 0? 
          enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] * 2: -1;

        bw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] < 0?         
          enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] *2: -1;      

        bw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] < 0?         
          enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] * 2: bw_rFrameUL;              
      }
      else
      {
        fw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y]  < 0 ?
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y]: -1;
        
        fw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : -1;
        
        fw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y]>> 1 : 
        enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : -1;      
        
        fw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] < 0 ? 
          enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : fw_rFrameUL;      
        
        bw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : -1;
        
        bw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : -1;
        
        bw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : -1;      
        
        bw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] >> 1: 
        enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : bw_rFrameUL;      
      }
    }
    
    fw_rFrame = (fw_rFrameL >= 0 && fw_rFrameU >= 0) ? min(fw_rFrameL,fw_rFrameU): max(fw_rFrameL,fw_rFrameU);
    fw_rFrame = (fw_rFrame >= 0 && fw_rFrameUR >= 0) ? min(fw_rFrame,fw_rFrameUR): max(fw_rFrame,fw_rFrameUR);
    
    bw_rFrame = (bw_rFrameL >= 0 && bw_rFrameU >= 0) ? min(bw_rFrameL,bw_rFrameU): max(bw_rFrameL,bw_rFrameU);
    bw_rFrame = (bw_rFrame >= 0 && bw_rFrameUR >= 0) ? min(bw_rFrame,bw_rFrameUR): max(bw_rFrame,bw_rFrameUR);        
    
    if (fw_rFrame >=0)
      SetMotionVectorPredictor (pmvfw, enc_picture->ref_idx, enc_picture->mv, fw_rFrame, LIST_0, 0, 0, 16, 16);
    
    if (bw_rFrame >=0)
      SetMotionVectorPredictor (pmvbw, enc_picture->ref_idx, enc_picture->mv, bw_rFrame, LIST_1, 0, 0, 16, 16);

    for (block_y=0; block_y<4; block_y++)
    {
      pic_block_y  = (img->pix_y>>2) + block_y;
      opic_block_y = (img->opix_y>>2) + block_y;
      
      for (block_x=0; block_x<4; block_x++)
      {
        pic_block_x  = (img->pix_x>>2) + block_x;
        opic_block_x = (img->opix_x>>2) + block_x;

        if (fw_rFrame >=0)
        {
          if (!fw_rFrame  && !moving_block[opic_block_x][opic_block_y])
          {
            all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
            all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;            
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=0;       
          }
          else
          {
            all_mvs [block_x][block_y][LIST_0][fw_rFrame][0][0] = pmvfw[0];
            all_mvs [block_x][block_y][LIST_0][fw_rFrame][0][1] = pmvfw[1];
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=fw_rFrame;              
          }
        }
        else
        {
          all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=-1;          
        }

        if (bw_rFrame >=0)
        {
          if(bw_rFrame==0 && !moving_block[opic_block_x][opic_block_y])
          {                  
            all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
            all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=bw_rFrame;     
          }
          else
          {
            all_mvs [block_x][block_y][LIST_1][bw_rFrame][0][0] = pmvbw[0];
            all_mvs [block_x][block_y][LIST_1][bw_rFrame][0][1] = pmvbw[1];
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=bw_rFrame;
          }               
        }
        else
        {      
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=-1;

          all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
        }
        
        if (fw_rFrame < 0 && bw_rFrame < 0)
        {
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = 
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
        }

        if      (direct_ref_idx[LIST_1][pic_block_x][pic_block_y]==-1) direct_pdir[pic_block_x][pic_block_y] = 0;
        else if (direct_ref_idx[LIST_0][pic_block_x][pic_block_y]==-1) direct_pdir[pic_block_x][pic_block_y] = 1;
        else                                                           direct_pdir[pic_block_x][pic_block_y] = 2;
      }
    }
  }
  else
  {
    //temporal direct mode copy from decoder
    for (block_y=0; block_y<4; block_y++)
    {
      pic_block_y  = (img->pix_y>>2) + block_y;
      opic_block_y = (img->opix_y>>2) + block_y;
      
      for (block_x=0; block_x<4; block_x++)
      {
        int refList; 
        int ref_idx; 

        int list_offset = ((img->MbaffFrameFlag)&&(currMB->mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

        pic_block_x  = (img->pix_x>>2) + block_x;
        opic_block_x = (img->opix_x>>2) + block_x;
        
        refList = (co_located_ref_idx[LIST_0][opic_block_x][opic_block_y]== -1 ? LIST_1 : LIST_0);
        ref_idx = co_located_ref_idx[refList][opic_block_x][opic_block_y];
              
        // next P is intra mode
        if (ref_idx==-1)
        {
          all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = 0;
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
          direct_pdir[pic_block_x][pic_block_y] = 2;
        }
        // next P is skip or inter mode
        else 
        {
          int mapped_idx=INVALIDINDEX;
          int iref; 

          {
            for (iref=0;iref<min(img->num_ref_idx_l0_active,listXsize[LIST_0+list_offset]);iref++)
            {
              if (enc_picture->ref_pic_num[LIST_0 +list_offset][iref]==co_located_ref_id[refList ][opic_block_x][opic_block_y])
              {
                mapped_idx=iref;
                break;
              }
              else //! invalid index. Default to zero even though this case should not happen
              {                        
                mapped_idx=INVALIDINDEX;
              }
            }
          }

          if (mapped_idx !=INVALIDINDEX)
          {
            mv_scale = img->mvscale[LIST_0+list_offset][mapped_idx];

            if (mv_scale==9999)
            {
              // forward
              all_mvs [block_x][block_y][LIST_0][0][0][0] = co_located_mv[refList][opic_block_x][opic_block_y][0];
              all_mvs [block_x][block_y][LIST_0][0][0][1] = co_located_mv[refList][opic_block_x][opic_block_y][1];
              // backward
              all_mvs [block_x][block_y][LIST_1][       0][0][0] = 0;
              all_mvs [block_x][block_y][LIST_1][       0][0][1] = 0;
            }else
            {
              // forward
              all_mvs [block_x][block_y][LIST_0][mapped_idx][0][0] = (mv_scale * co_located_mv[refList][opic_block_x][opic_block_y][0] + 128) >> 8;
              all_mvs [block_x][block_y][LIST_0][mapped_idx][0][1] = (mv_scale * co_located_mv[refList][opic_block_x][opic_block_y][1] + 128) >> 8;
              // backward
              all_mvs [block_x][block_y][LIST_1][       0][0][0] = ((mv_scale - 256)* co_located_mv[refList][opic_block_x][opic_block_y][0] + 128) >> 8;
              all_mvs [block_x][block_y][LIST_1][       0][0][1] = ((mv_scale - 256)* co_located_mv[refList][opic_block_x][opic_block_y][1] + 128) >> 8;
            }
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = mapped_idx;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
            direct_pdir[pic_block_x][pic_block_y] = 2;
          }
          else
          {
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = -1;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = -1;
            direct_pdir[pic_block_x][pic_block_y] = -1;
          }
        }
      }
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    control the sign of a with b
 ************************************************************************
 */
int sign(int a,int b)
{
  int x;
  x=absm(a);
  if (b >= 0)
    return x;
  else
    return -x;
}

