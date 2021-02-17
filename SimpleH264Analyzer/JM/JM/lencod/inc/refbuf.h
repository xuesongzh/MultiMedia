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
 * \file refbuf.h
 *
 * \brief
 *    Declarations of the reference frame buffer types and functions
 ************************************************************************
 */
#ifndef _REBUF_H_
#define _REBUF_H_


#include "global.h"


pel_t UMVPelY_14 (pel_t **Pic, int y, int x, int height, int width);
pel_t FastPelY_14 (pel_t **Pic, int y, int x, int height, int width);

pel_t *FastLine16Y_11 (pel_t *Pic, int y, int x, int height, int width);
pel_t *UMVLine16Y_11 (pel_t *Pic, int y, int x, int height, int width);

void PutPel_14 (pel_t **Pic, int y, int x, pel_t val);
void PutPel_11 (pel_t *Pic, int y, int x, pel_t val, int width);

#endif

