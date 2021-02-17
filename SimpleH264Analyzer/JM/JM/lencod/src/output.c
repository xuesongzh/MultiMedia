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
 * \file output.c
 *
 * \brief
 *    Output an image and Trance support
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Karsten Suehring               <suehring@hhi.de>
 ************************************************************************
 */

#include "contributors.h"

#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "global.h"
#include "mbuffer.h"
#include "image.h"

FrameStore* out_buffer;

/*!
 ************************************************************************
 * \brief
 *    Writes out a storable picture
 * \param p
 *    Picture to be written
 * \param p_out
 *    Output file
 ************************************************************************
 */
void write_picture(StorablePicture *p, FILE *p_out)
{
  int i,j;

  if (p_out)
  {
    for(i=0;i<p->size_y;i++)
      for(j=0;j<p->size_x;j++)
      {
        fputc(p->imgY[i][j],p_out);
      }
    for(i=0;i<p->size_y_cr;i++)
      for(j=0;j<p->size_x_cr;j++)
      {
        fputc(p->imgUV[0][i][j],p_out);
      }
    for(i=0;i<p->size_y_cr;i++)
      for(j=0;j<p->size_x_cr;j++)
      {
        fputc(p->imgUV[1][i][j],p_out);
      }
      fflush(p_out);
  }
}

/*!
 ************************************************************************
 * \brief
 *    Initialize output buffer for direct output
 ************************************************************************
 */
void init_out_buffer()
{
  out_buffer = alloc_frame_store();
}

/*!
 ************************************************************************
 * \brief
 *    Uninitialize output buffer for direct output
 ************************************************************************
 */
void uninit_out_buffer()
{
  free_frame_store(out_buffer);
  out_buffer=NULL;
}

/*!
 ************************************************************************
 * \brief
 *    Initialize picture memory with (Y:0,U:128,V:128)
 ************************************************************************
 */
void clear_picture(StorablePicture *p)
{
  int i;

  for(i=0;i<p->size_y;i++)
    memset(p->imgY[i], 0 ,p->size_x);
  for(i=0;i<p->size_y_cr;i++)
    memset(p->imgUV[0][i], 128 ,p->size_x_cr);
  for(i=0;i<p->size_y_cr;i++)
    memset(p->imgUV[1][i], 128 ,p->size_x_cr);
}

/*!
 ************************************************************************
 * \brief
 *    Write out not paired direct output fields. A second empty field is generated
 *    and combined into the frame buffer.
 * \param fs
 *    FrameStore that contains a single field
 * \param p_out
 *    Output file
 ************************************************************************
 */
void write_unpaired_field(FrameStore* fs, FILE *p_out)
{
  StorablePicture *p;
  assert (fs->is_used<3);
  if(fs->is_used &1)
  {
    // we have a top field
    // construct an empty bottom field
    p = fs->top_field;
    fs->bottom_field = alloc_storable_picture(BOTTOM_FIELD, p->size_x, p->size_y, p->size_x_cr, p->size_y_cr);
    clear_picture(fs->bottom_field);
    dpb_combine_field(fs);
    write_picture (fs->frame, p_out);
  }

  if(fs->is_used &2)
  {
    // we have a bottom field
    // construct an empty top field
    p = fs->bottom_field;
    fs->top_field = alloc_storable_picture(TOP_FIELD, p->size_x, p->size_y, p->size_x_cr, p->size_y_cr);
    clear_picture(fs->bottom_field);
    dpb_combine_field(fs);
    write_picture (fs->frame, p_out);

  }

  fs->is_used=3;
  
}

/*!
 ************************************************************************
 * \brief
 *    Write out unpaired fields from output buffer.
 * \param p_out
 *    Output file
 ************************************************************************
 */
void flush_direct_output(FILE *p_out)
{
  write_unpaired_field(out_buffer, p_out);

  free_storable_picture(out_buffer->frame);
  out_buffer->frame = NULL;
  free_storable_picture(out_buffer->top_field);
  out_buffer->top_field = NULL;
  free_storable_picture(out_buffer->bottom_field);
  out_buffer->bottom_field = NULL;
  out_buffer->is_used = 0;
}


/*!
 ************************************************************************
 * \brief
 *    Write a frame (from FrameStore)
 * \param fs
 *    FrameStore containing the frame
 * \param p_out
 *    Output file
 ************************************************************************
 */
void write_stored_frame( FrameStore *fs,FILE *p_out)
{
  // make sure no direct output field is pending
  flush_direct_output(p_out);

  if (fs->is_used<3)
  {
    write_unpaired_field(fs, p_out);
  }
  else
  {
    write_picture(fs->frame, p_out);
  }

  fs->is_output = 1;
}

/*!
 ************************************************************************
 * \brief
 *    Directly output a picture without storing it in the DPB. Fields 
 *    are buffered before they are written to the file.
 * \param p
 *    Picture for output
 * \param p_out
 *    Output file
 ************************************************************************
 */
void direct_output(StorablePicture *p, FILE *p_out)
{
  if (p->structure==FRAME)
  {
    // we have a frame (or complementary field pair)
    // so output it directly
    flush_direct_output(p_out);
    write_picture (p, p_out);
    free_storable_picture(p);
    return;
  }

  if (p->structure == TOP_FIELD)
  {
    if (out_buffer->is_used &1)
      flush_direct_output(p_out);
    out_buffer->top_field = p;
    out_buffer->is_used |= 1;
  }

  if (p->structure == BOTTOM_FIELD)
  {
    if (out_buffer->is_used &2)
      flush_direct_output(p_out);
    out_buffer->bottom_field = p;
    out_buffer->is_used |= 2;
  }

  if (out_buffer->is_used == 3)
  {
    // we have both fields, so output them
    dpb_combine_field(out_buffer);
    write_picture (out_buffer->frame, p_out);
    free_storable_picture(out_buffer->frame);
    out_buffer->frame = NULL;
    free_storable_picture(out_buffer->top_field);
    out_buffer->top_field = NULL;
    free_storable_picture(out_buffer->bottom_field);
    out_buffer->bottom_field = NULL;
    out_buffer->is_used = 0;
  }
}
