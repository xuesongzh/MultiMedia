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
 * \file
 *    rdopt_coding_state.h
 *
 * \author
 *    Heiko Schwarz
 *
 * \date
 *    17. April 2001
 *
 * \brief
 *    Headerfile for storing/restoring coding state
 *    (for rd-optimized mode decision)
 **************************************************************************
 */

#ifndef _RD_OPT_CS_H_
#define _RD_OPT_CS_H_

#include "global.h"



typedef struct {

  // important variables of data partition array
  int                   no_part;
  EncodingEnvironment  *encenv;
  Bitstream            *bitstream;

  // contexts for binary arithmetic coding
  int                   symbol_mode;
  MotionInfoContexts   *mot_ctx;
  TextureInfoContexts  *tex_ctx;

  // syntax element number and bitcounters
  int                   currSEnr;
  int                   bitcounter[MAX_BITCOUNTER_MB];

  // elements of current macroblock
  int                   mvd[2][BLOCK_MULTIPLE][BLOCK_MULTIPLE][2];
  unsigned long         cbp_bits;
} CSobj;
typedef CSobj* CSptr;


void  delete_coding_state  (CSptr);  //!< delete structure
CSptr create_coding_state  ();       //!< create structure

void  store_coding_state   (CSptr);  //!< store parameters
void  reset_coding_state   (CSptr);  //!< restore parameters


#endif

