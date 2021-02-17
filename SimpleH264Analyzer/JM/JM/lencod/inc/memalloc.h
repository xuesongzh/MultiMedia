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
 * \file  memalloc.h
 *
 * \brief
 *    Memory allocation and free helper funtions
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 ************************************************************************
 */

#ifndef _MEMALLOC_H_
#define _MEMALLOC_H_

#include "global.h"

int  get_mem2D(byte ***array2D, int rows, int columns);
int  get_mem2Dint(int ***array2D, int rows, int columns);
int  get_mem2Dint64(int64 ***array2D, int rows, int columns);
int  get_mem3D(byte ****array2D, int frames, int rows, int columns);
int  get_mem3Dint(int ****array3D, int frames, int rows, int columns);
int  get_mem3Dint64(int64 ****array3D, int frames, int rows, int columns);
int  get_mem4Dint(int *****array4D, int idx, int frames, int rows, int columns );

void free_mem2D(byte **array2D);
void free_mem2Dint(int **array2D);
void free_mem2Dint64(int64 **array2D);
void free_mem3D(byte ***array2D, int frames);
void free_mem3Dint(int ***array3D, int frames);
void free_mem3Dint64(int64 ***array3D, int frames);
void free_mem4Dint(int ****array4D, int idx, int frames);

void no_mem_exit(char *where);

#endif
