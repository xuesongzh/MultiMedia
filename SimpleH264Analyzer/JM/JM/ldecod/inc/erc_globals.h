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
 ************************************************************************
 * \file erc_globals.h
 *
 * \brief
 *      global header file for error concealment module
 *
 * \author
 *      - Viktor Varsa                     <viktor.varsa@nokia.com>
 *      - Ye-Kui Wang                   <wyk@ieee.org>
 ************************************************************************
 */

#ifndef _ERC_GLOBALS_H_
#define _ERC_GLOBALS_H_


#include <string.h>

/* "block" means an 8x8 pixel area */

/* Region modes */
#define REGMODE_INTER_COPY       0  /* Copy region */
#define REGMODE_INTER_PRED       1  /* Inter region with motion vectors */
#define REGMODE_INTRA            2  /* Intra region */
#define REGMODE_SPLITTED         3  /* Any region mode higher than this indicates that the region 
                                       is splitted which means 8x8 block */
#define REGMODE_INTER_COPY_8x8   4
#define REGMODE_INTER_PRED_8x8   5
#define REGMODE_INTRA_8x8        6

/* YUV pixel domain image arrays for a video frame */
typedef struct
{
  byte *yptr;
  byte *uptr;
  byte *vptr;
} frame;

/* region structure stores information about a region that is needed for concealment */
typedef struct 
{
  byte regionMode;  /* region mode as above */
  int xMin;         /* X coordinate of the pixel position of the top-left corner of the region */
  int yMin;         /* Y coordinate of the pixel position of the top-left corner of the region */
  int32 mv[3];      /* motion vectors in 1/4 pixel units: mvx = mv[0], mvy = mv[1], 
                              and ref_frame = mv[2] */
} objectBuffer_t;

#endif

