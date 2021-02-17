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
 * \file refbuf.c
 *
 * \brief
 *    Declarations of teh reference frame buffer types and functions
 ************************************************************************
 */



#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <assert.h>

#include "refbuf.h"

/*!
 ************************************************************************
 * \brief
 *    Reference buffer write routines
 ************************************************************************
 */
void PutPel_14 (pel_t **Pic, int y, int x, pel_t val)
{
  Pic [IMG_PAD_SIZE*4+y][IMG_PAD_SIZE*4+x] = val;
}

void PutPel_11 (pel_t *Pic, int y, int x, pel_t val, int width)
{
  Pic [y*width+x] = val;
}

/*!
 ************************************************************************
 * \note
 *    The following functions returning line are NOT reentrant!  Use a buffer
 *    provided by the caller to change that (but it costs a memcpy()...
 ************************************************************************
 */
static pel_t line[16];

pel_t *FastLine16Y_11 (pel_t *Pic, int y, int x, int height, int width)
{
  return &Pic [y*width+x];
}


pel_t *UMVLine16Y_11 (pel_t *Pic, int y, int x, int height, int width)
{
  int i, maxx;
  pel_t *Picy;

  Picy = &Pic [max(0,min(height-1,y)) * width];

  if (x < 0) {                    // Left edge ?

    maxx = min(0,x+16);
    for (i = x; i < maxx; i++)
      line[i-x] = Picy [0];       // Replicate left edge pixel

    maxx = x+16;
    for (i = 0; i < maxx; i++)    // Copy non-edge pixels
      line[i-x] = Picy [i];
  }
  else if (x > width-16)  {  // Right edge ?

    maxx = width;
    for (i = x; i < maxx; i++)
      line[i-x] = Picy [i];       // Copy non-edge pixels

    maxx = x+16;
    for (i = max(width,x); i < maxx; i++)
      line[i-x] = Picy [width-1];  // Replicate right edge pixel
  }
  else                            // No edge
    return &Picy [x];

  return line;
}


pel_t *FastLineX (int dummy, pel_t* Pic, int y, int x, int height, int width)
{
  return Pic + y*width + x;
}


pel_t *UMVLineX (int size, pel_t* Pic, int y, int x, int height, int width)
{
  int i, maxx;
  pel_t *Picy;

  Picy = Pic + max(0,min(height-1,y)) * width;

  if (x < 0)                            // Left edge
  {
    maxx = min(0,x+size);
    for (i = x; i < maxx; i++)
    {
      line[i-x] = Picy [0];             // Replicate left edge pixel
    }
    maxx = x+size;
    for (i = 0; i < maxx; i++)          // Copy non-edge pixels
      line[i-x] = Picy [i];
  }
  else if (x > width-size)         // Right edge
  {
    maxx = width;
    for (i = x; i < maxx; i++)
    {
      line[i-x] = Picy [i];             // Copy non-edge pixels
    }
    maxx = x+size;
    for (i = max(width,x); i < maxx; i++)
    {
      line[i-x] = Picy [width-1];  // Replicate right edge pixel
    }
  }
  else                                  // No edge
  {
    return Picy + x;
  }

  return line;
}

/*!
 ************************************************************************
 * \brief
 *    Reference buffer, 1/4 pel
 ************************************************************************
 */
pel_t UMVPelY_14 (pel_t **Pic, int y, int x, int height, int width)
{
  int width4  = ((width+2*IMG_PAD_SIZE-1)<<2);
  int height4 = ((height+2*IMG_PAD_SIZE-1)<<2);

  x = x + IMG_PAD_SIZE*4;
  y = y + IMG_PAD_SIZE*4;

  if (x < 0)
  {
    if (y < 0)
      return Pic [y&3][x&3];
    if (y > height4)
      return Pic [height4+(y&3)][x&3];
    return Pic [y][x&3];
  }

  if (x > width4)
  {
    if (y < 0)
      return Pic [y&3][width4+(x&3)];
    if (y > height4)
      return Pic [height4+(y&3)][width4+(x&3)];
    return Pic [y][width4+(x&3)];
  }

  if (y < 0)    // note: corner pixels were already processed
    return Pic [y&3][x];
  if (y > height4)
    return Pic [height4+(y&3)][x];

  return Pic [y][x];
}

pel_t FastPelY_14 (pel_t **Pic, int y, int x, int height, int width)
{
  return Pic [IMG_PAD_SIZE*4+y][IMG_PAD_SIZE*4+x];
}


