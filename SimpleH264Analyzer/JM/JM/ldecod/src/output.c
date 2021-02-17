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
#include "memalloc.h"

FrameStore* out_buffer;

StorablePicture *pending_output = NULL;
int              pending_output_state = FRAME;


void write_out_picture(StorablePicture *p, FILE *p_out);

#ifdef PAIR_FIELDS_IN_OUTPUT

void clear_picture(StorablePicture *p);

/*!
 ************************************************************************
 * \brief
 *    output the pending frame buffer
 * \param p_out
 *    Output file
 ************************************************************************
 */
void flush_pending_output(FILE *p_out)
{
  if (pending_output_state!=FRAME)
  {
    write_out_picture(pending_output, p_out);
  }
  
  if (pending_output->imgY)
  {
    free_mem2D (pending_output->imgY);
    pending_output->imgY=NULL;
  }
  if (pending_output->imgUV)
  {
    free_mem3D (pending_output->imgUV, 2);
    pending_output->imgUV=NULL;
  }

  pending_output_state = FRAME;
}


/*!
 ************************************************************************
 * \brief
 *    Writes out a storable picture 
 *    If the picture is a field, the output buffers the picture and tries 
 *    to pair it with the next field.
 * \param p
 *    Picture to be written
 * \param p_out
 *    Output file
 ************************************************************************
 */
void write_picture(StorablePicture *p, FILE *p_out, int real_structure)
{
   int i, add;

  if (real_structure==FRAME)
  {
    flush_pending_output(p_out);
    write_out_picture(p, p_out);
    return;
  }
  if (real_structure==pending_output_state)
  {
    flush_pending_output(p_out);
    write_picture(p, p_out, real_structure);
    return;
  }

  if (pending_output_state == FRAME)
  {
    pending_output->size_x = p->size_x;
    pending_output->size_y = p->size_y;
    pending_output->size_x_cr = p->size_x_cr;
    pending_output->size_y_cr = p->size_y_cr;


    pending_output->frame_mbs_only_flag = p->frame_mbs_only_flag;
    pending_output->frame_cropping_flag = p->frame_cropping_flag;
    if (pending_output->frame_cropping_flag)
    {
      pending_output->frame_cropping_rect_left_offset = p->frame_cropping_rect_left_offset;
      pending_output->frame_cropping_rect_right_offset = p->frame_cropping_rect_right_offset;
      pending_output->frame_cropping_rect_top_offset = p->frame_cropping_rect_top_offset;
      pending_output->frame_cropping_rect_bottom_offset = p->frame_cropping_rect_bottom_offset;
    }

    get_mem2D (&(pending_output->imgY), pending_output->size_y, pending_output->size_x);
    get_mem3D (&(pending_output->imgUV), 2, pending_output->size_y_cr, pending_output->size_x_cr );

    clear_picture(pending_output);

    // copy first field
    if (real_structure == TOP_FIELD)
    {
      add = 0;
    }
    else
    {
      add = 1;
    }

    for (i=0; i<pending_output->size_y; i+=2)
    {
      memcpy(pending_output->imgY[(i+add)], p->imgY[(i+add)], p->size_x);
    }
    for (i=0; i<pending_output->size_y_cr; i+=2)
    {
      memcpy(pending_output->imgUV[0][(i+add)], p->imgUV[0][(i+add)], p->size_x_cr);
      memcpy(pending_output->imgUV[1][(i+add)], p->imgUV[1][(i+add)], p->size_x_cr);
    }
    pending_output_state = real_structure;
  }
  else
  {
    if (  (pending_output->size_x!=p->size_x) || (pending_output->size_y!= p->size_y) 
       || (pending_output->frame_mbs_only_flag != p->frame_mbs_only_flag)
       || (pending_output->frame_cropping_flag != p->frame_cropping_flag)
       || ( pending_output->frame_cropping_flag &&
            (  (pending_output->frame_cropping_rect_left_offset   != p->frame_cropping_rect_left_offset)
             ||(pending_output->frame_cropping_rect_right_offset  != p->frame_cropping_rect_right_offset)
             ||(pending_output->frame_cropping_rect_top_offset    != p->frame_cropping_rect_top_offset)
             ||(pending_output->frame_cropping_rect_bottom_offset != p->frame_cropping_rect_bottom_offset)
            )
          )
       )
    {
      flush_pending_output(p_out);
      write_picture (p, p_out, real_structure);
      return;
    }
    // copy second field
    if (real_structure == TOP_FIELD)
    {
      add = 0;
    }
    else
    {
      add = 1;
    }

    for (i=0; i<pending_output->size_y; i+=2)
    {
      memcpy(pending_output->imgY[(i+add)], p->imgY[(i+add)], p->size_x);
    }
    for (i=0; i<pending_output->size_y_cr; i+=2)
    {
      memcpy(pending_output->imgUV[0][(i+add)], p->imgUV[0][(i+add)], p->size_x_cr);
      memcpy(pending_output->imgUV[1][(i+add)], p->imgUV[1][(i+add)], p->size_x_cr);
    }

    flush_pending_output(p_out);
  }
}

#else

/*!
 ************************************************************************
 * \brief
 *    Writes out a storable picture without doing any output modifications
 * \param p
 *    Picture to be written
 * \param p_out
 *    Output file
 ************************************************************************
 */
void write_picture(StorablePicture *p, FILE *p_out, int real_structure)
{
  write_out_picture(p, p_out);
}


#endif

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
void write_out_picture(StorablePicture *p, FILE *p_out)
{
  int i,j;

  int crop_left, crop_right, crop_top, crop_bottom;
  int crop_vert_mult;
  
  if (p->non_existing)
    return;

  if (p->frame_mbs_only_flag)
  {
    crop_vert_mult = 2;
  }
  else
  {
    if (p->structure != FRAME)
    {
      crop_vert_mult = 2;
    }
    else
    {
      crop_vert_mult = 4;
    }
  }

  if (p->frame_cropping_flag)
  {
    crop_left   = 2 * p->frame_cropping_rect_left_offset;
    crop_right  = 2 * p->frame_cropping_rect_right_offset;
    crop_top    = crop_vert_mult * p->frame_cropping_rect_top_offset;
    crop_bottom = crop_vert_mult * p->frame_cropping_rect_bottom_offset;
  }
  else
  {
    crop_left = crop_right = crop_top = crop_bottom = 0;
  }

  //printf ("write frame size: %dx%d\n", p->size_x-crop_left-crop_right,p->size_y-crop_top-crop_bottom );
  
  for(i=crop_top;i<p->size_y-crop_bottom;i++)
    for(j=crop_left;j<p->size_x-crop_right;j++)
    {
      fputc(p->imgY[i][j],p_out);
    }

  crop_left   /= 2;
  crop_right  /= 2;
  crop_top    /= 2;
  crop_bottom /= 2;

  for(i=crop_top;i<p->size_y_cr-crop_bottom;i++)
    for(j=crop_left;j<p->size_x_cr-crop_right;j++)
    {
      fputc(p->imgUV[0][i][j],p_out);
    }
  for(i=crop_top;i<p->size_y_cr-crop_bottom;i++)
    for(j=crop_left;j<p->size_x_cr-crop_right;j++)
    {
      fputc(p->imgUV[1][i][j],p_out);
    }
    
  fflush(p_out);
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
  pending_output = calloc (sizeof(StorablePicture), 1);
  if (NULL==pending_output) no_mem_exit("init_out_buffer");
  pending_output->imgUV = NULL;
  pending_output->imgY  = NULL;
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
#ifdef PAIR_FIELDS_IN_OUTPUT
  flush_pending_output(p_out);
#endif
  free (pending_output);
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
    memset(p->imgY[i], 128 ,p->size_x);
  for(i=0;i<p->size_y_cr;i++)
    memset(p->imgUV[0][i], 128 ,p->size_x_cr);
  for(i=0;i<p->size_y_cr;i++)
    memset(p->imgUV[1][i], 128 ,p->size_x_cr);
  fflush(p_out);
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
    fs->bottom_field = alloc_storable_picture(BOTTOM_FIELD, p->size_x, 2*p->size_y, p->size_x_cr, 2*p->size_y_cr);
    clear_picture(fs->bottom_field);
    dpb_combine_field(fs);
    write_picture (fs->frame, p_out, TOP_FIELD);
  }

  if(fs->is_used &2)
  {
    // we have a bottom field
    // construct an empty top field
    p = fs->bottom_field;
    fs->top_field = alloc_storable_picture(TOP_FIELD, p->size_x, 2*p->size_y, p->size_x_cr, 2*p->size_y_cr);
    clear_picture(fs->top_field);
    fs ->top_field->frame_cropping_flag = fs->bottom_field->frame_cropping_flag;
    if(fs ->top_field->frame_cropping_flag) 
    {
      fs ->top_field->frame_cropping_rect_top_offset = fs->bottom_field->frame_cropping_rect_top_offset;
      fs ->top_field->frame_cropping_rect_bottom_offset = fs->bottom_field->frame_cropping_rect_bottom_offset;
      fs ->top_field->frame_cropping_rect_left_offset = fs->bottom_field->frame_cropping_rect_left_offset;
      fs ->top_field->frame_cropping_rect_right_offset = fs->bottom_field->frame_cropping_rect_right_offset;
    }
    dpb_combine_field(fs);
    write_picture (fs->frame, p_out, BOTTOM_FIELD);
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
    write_picture(fs->frame, p_out, FRAME);
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
    write_picture (p, p_out, FRAME);
    if (p_ref)
      find_snr(snr, p, p_ref);
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
    write_picture (out_buffer->frame, p_out, FRAME);
    if (p_ref)
      find_snr(snr, out_buffer->frame, p_ref);
    free_storable_picture(out_buffer->frame);
    out_buffer->frame = NULL;
    free_storable_picture(out_buffer->top_field);
    out_buffer->top_field = NULL;
    free_storable_picture(out_buffer->bottom_field);
    out_buffer->bottom_field = NULL;
    out_buffer->is_used = 0;
  }
}
