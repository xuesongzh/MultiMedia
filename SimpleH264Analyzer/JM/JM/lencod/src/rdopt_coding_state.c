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
 * \file rdopt_coding_state.c
 *
 * \brief
 *    Storing/restoring coding state for
 *    Rate-Distortion optimized mode decision
 *
 * \author
 *    Heiko Schwarz
 *
 * \date
 *    17. April 2001
 **************************************************************************/

#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include "rdopt_coding_state.h"
#include "cabac.h"



/*!
 ************************************************************************
 * \brief
 *    delete structure for storing coding state
 ************************************************************************
 */
void
delete_coding_state (CSptr cs)
{
  if (cs != NULL)
  {
    //=== structures of data partition array ===
    if (cs->encenv    != NULL)   free (cs->encenv);
    if (cs->bitstream != NULL)   free (cs->bitstream);

    //=== contexts for binary arithmetic coding ===
    delete_contexts_MotionInfo  (cs->mot_ctx);
    delete_contexts_TextureInfo (cs->tex_ctx);

    //=== coding state structure ===
    free (cs);
    cs=NULL;
  }
}


/*!
 ************************************************************************
 * \brief
 *    create structure for storing coding state
 ************************************************************************
 */
CSptr
create_coding_state ()
{
  CSptr cs;

  //=== coding state structure ===
  if ((cs = (CSptr) calloc (1, sizeof(CSobj))) == NULL)
    no_mem_exit("init_coding_state: cs");

  //=== important variables of data partition array ===
  cs->no_part = input->partition_mode==0?1:3;
  if (input->symbol_mode == CABAC)
  {
    if ((cs->encenv = (EncodingEnvironment*) calloc (cs->no_part, sizeof(EncodingEnvironment))) == NULL)
      no_mem_exit("init_coding_state: cs->encenv");
  }
  else
  {
    cs->encenv = NULL;
  }
  if ((cs->bitstream = (Bitstream*) calloc (cs->no_part, sizeof(Bitstream))) == NULL)
    no_mem_exit("init_coding_state: cs->bitstream");

  //=== context for binary arithmetic coding ===
  cs->symbol_mode = input->symbol_mode;
  if (cs->symbol_mode == CABAC)
  {
    cs->mot_ctx = create_contexts_MotionInfo ();
    cs->tex_ctx = create_contexts_TextureInfo();
  }
  else
  {
    cs->mot_ctx = NULL;
    cs->tex_ctx = NULL;
  }

  return cs;
}


/*!
 ************************************************************************
 * \brief
 *    store coding state (for rd-optimized mode decision)
 ************************************************************************
 */
void
store_coding_state (CSptr cs)
{
  int  i;

  EncodingEnvironment  *ee_src, *ee_dest;
  Bitstream            *bs_src, *bs_dest;

  MotionInfoContexts   *mc_src  = img->currentSlice->mot_ctx;
  TextureInfoContexts  *tc_src  = img->currentSlice->tex_ctx;
  MotionInfoContexts   *mc_dest = cs->mot_ctx;
  TextureInfoContexts  *tc_dest = cs->tex_ctx;
  Macroblock           *currMB  = &(img->mb_data [img->current_mb_nr]);


  if (!input->rdopt)  return;

  if (cs->symbol_mode==CABAC)
  {
  //=== important variables of data partition array ===
	//only one partition for IDR img
  for (i = 0; i <(img->currentPicture->idr_flag? 1:cs->no_part); i++)
  {
    ee_src  = &(img->currentSlice->partArr[i].ee_cabac);
    bs_src  =   img->currentSlice->partArr[i].bitstream;
    ee_dest = &(cs->encenv   [i]);
    bs_dest = &(cs->bitstream[i]);

    memcpy (ee_dest, ee_src, sizeof(EncodingEnvironment));
    memcpy (bs_dest, bs_src, sizeof(Bitstream));
  }

  //=== contexts for binary arithmetic coding ===
    memcpy (mc_dest, mc_src, sizeof(MotionInfoContexts));
    memcpy (tc_dest, tc_src, sizeof(TextureInfoContexts));
  
  }
  else
  {
    //=== important variables of data partition array ===
  for (i = 0; i <(img->currentPicture->idr_flag? 1:cs->no_part); i++)
  {    
    bs_src  =   img->currentSlice->partArr[i].bitstream;   
    bs_dest = &(cs->bitstream[i]);
      memcpy (bs_dest, bs_src, sizeof(Bitstream));
    }
  }
  //=== syntax element number and bitcounters ===
  cs->currSEnr = currMB->currSEnr;
  memcpy (cs->bitcounter, currMB->bitcounter, MAX_BITCOUNTER_MB*sizeof(int));

  //=== elements of current macroblock ===
  memcpy (cs->mvd, currMB->mvd, 2*2*BLOCK_MULTIPLE*BLOCK_MULTIPLE*sizeof(int));
  cs->cbp_bits = currMB->cbp_bits;
}


/*!
 ************************************************************************
 * \brief
 *    restore coding state (for rd-optimized mode decision)
 ************************************************************************
 */
void
reset_coding_state (CSptr cs)
{
  int  i;

  EncodingEnvironment  *ee_src, *ee_dest;
  Bitstream            *bs_src, *bs_dest;

  MotionInfoContexts   *mc_dest = img->currentSlice->mot_ctx;
  TextureInfoContexts  *tc_dest = img->currentSlice->tex_ctx;
  MotionInfoContexts   *mc_src  = cs->mot_ctx;
  TextureInfoContexts  *tc_src  = cs->tex_ctx;
  Macroblock           *currMB  = &(img->mb_data [img->current_mb_nr]);


  if (!input->rdopt)  return;

  if (cs->symbol_mode==CABAC) 
  {
  //=== important variables of data partition array ===
	//only one partition for IDR img
  for (i = 0; i <(img->currentPicture->idr_flag? 1:cs->no_part); i++)
  {
    ee_dest = &(img->currentSlice->partArr[i].ee_cabac);
    bs_dest =   img->currentSlice->partArr[i].bitstream;
    ee_src  = &(cs->encenv   [i]);
    bs_src  = &(cs->bitstream[i]);

    //--- parameters of encoding environments ---
    memcpy (ee_dest, ee_src, sizeof(EncodingEnvironment));
    memcpy (bs_dest, bs_src, sizeof(Bitstream));
  }


  //=== contexts for binary arithmetic coding ===
    memcpy (mc_dest, mc_src, sizeof(MotionInfoContexts));
    memcpy (tc_dest, tc_src, sizeof(TextureInfoContexts));
    
  }
  else
  {
    //=== important variables of data partition array ===
	//only one partition for IDR img
  for (i = 0; i <(img->currentPicture->idr_flag? 1:cs->no_part); i++)

    {
      bs_dest =   img->currentSlice->partArr[i].bitstream;
      bs_src  = &(cs->bitstream[i]);

      //--- parameters of encoding environments ---   
      memcpy (bs_dest, bs_src, sizeof(Bitstream));
    }
  }

  //=== syntax element number and bitcounters ===
  currMB->currSEnr = cs->currSEnr;
  memcpy (currMB->bitcounter, cs->bitcounter, MAX_BITCOUNTER_MB*sizeof(int));

  //=== elements of current macroblock ===
  memcpy (currMB->mvd, cs->mvd, 2*2*BLOCK_MULTIPLE*BLOCK_MULTIPLE*sizeof(int));
  currMB->cbp_bits = cs->cbp_bits;
}

