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
 *  \file
 *     parset.c
 *  \brief
 *     Parameter Sets
 *  \author
 *     Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Stephan Wenger          <stewe@cs.tu-berlin.de>
 *
 ***********************************************************************
 */

#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "global.h"
#include "parsetcommon.h"
#include "parset.h"
#include "nalu.h"
#include "memalloc.h"
#include "fmo.h"
#include "cabac.h"
#include "vlc.h"
#include "mbuffer.h"
#include "erc_api.h"

#if TRACE
#define SYMTRACESTRING(s) strncpy(sym->tracestring,s,TRACESTRING_SIZE)
#else
#define SYMTRACESTRING(s) // do nothing
#endif

extern int UsedBits;      // for internal statistics, is adjusted by se_v, ue_v, u_1
extern ColocatedParams *Co_located;

seq_parameter_set_rbsp_t SeqParSet[MAXSPS];
pic_parameter_set_rbsp_t PicParSet[MAXPPS];

extern StorablePicture* dec_picture;

// fill sps with content of p

int InterpretSPS (DataPartition *p, seq_parameter_set_rbsp_t *sps)
{
  unsigned i;
  int reserved_zero;
  Bitstream *s = p->bitstream;

  assert (p != NULL);
  assert (p->bitstream != NULL);
  assert (p->bitstream->streamBuffer != 0);
  assert (sps != NULL);

  UsedBits = 0;

  sps->profile_idc                            = u_v  (8, "SPS: profile_idc"                           , s);

  sps->constrained_set0_flag                  = u_1  (   "SPS: constrained_set0_flag"                 , s);
  sps->constrained_set1_flag                  = u_1  (   "SPS: constrained_set1_flag"                 , s);
  sps->constrained_set2_flag                  = u_1  (   "SPS: constrained_set2_flag"                 , s);
  reserved_zero                               = u_v  (5, "SPS: reserved_zero_5bits"                   , s);
  assert (reserved_zero==0);

  sps->level_idc                              = u_v  (8, "SPS: level_idc"                             , s);
  

  sps->seq_parameter_set_id                   = ue_v ("SPS: seq_parameter_set_id"                     , s);
  sps->log2_max_frame_num_minus4              = ue_v ("SPS: log2_max_frame_num_minus4"                , s);
  sps->pic_order_cnt_type                     = ue_v ("SPS: pic_order_cnt_type"                       , s);

  if (sps->pic_order_cnt_type == 0)
    sps->log2_max_pic_order_cnt_lsb_minus4 = ue_v ("SPS: log2_max_pic_order_cnt_lsb_minus4"           , s);
  else if (sps->pic_order_cnt_type == 1)
  {
    sps->delta_pic_order_always_zero_flag      = u_1  ("SPS: delta_pic_order_always_zero_flag"       , s);
    sps->offset_for_non_ref_pic                = se_v ("SPS: offset_for_non_ref_pic"                 , s);
    sps->offset_for_top_to_bottom_field        = se_v ("SPS: offset_for_top_to_bottom_field"         , s);
    sps->num_ref_frames_in_pic_order_cnt_cycle = ue_v ("SPS: num_ref_frames_in_pic_order_cnt_cycle"  , s);
    for(i=0; i<sps->num_ref_frames_in_pic_order_cnt_cycle; i++)
      sps->offset_for_ref_frame[i]               = se_v ("SPS: offset_for_ref_frame[i]"              , s);
  }
  sps->num_ref_frames                        = ue_v ("SPS: num_ref_frames"                         , s);
  sps->gaps_in_frame_num_value_allowed_flag  = u_1  ("SPS: gaps_in_frame_num_value_allowed_flag"   , s);
  sps->pic_width_in_mbs_minus1               = ue_v ("SPS: pic_width_in_mbs_minus1"                , s);
  sps->pic_height_in_map_units_minus1        = ue_v ("SPS: pic_height_in_map_units_minus1"         , s);
  sps->frame_mbs_only_flag                   = u_1  ("SPS: frame_mbs_only_flag"                    , s);
  if (!sps->frame_mbs_only_flag)
  {
    sps->mb_adaptive_frame_field_flag          = u_1  ("SPS: mb_adaptive_frame_field_flag"           , s);
  }
  sps->direct_8x8_inference_flag             = u_1  ("SPS: direct_8x8_inference_flag"              , s);
  sps->frame_cropping_flag                   = u_1  ("SPS: frame_cropping_flag"                , s);

  if (sps->frame_cropping_flag)
  {
    sps->frame_cropping_rect_left_offset      = ue_v ("SPS: frame_cropping_rect_left_offset"           , s);
    sps->frame_cropping_rect_right_offset     = ue_v ("SPS: frame_cropping_rect_right_offset"          , s);
    sps->frame_cropping_rect_top_offset       = ue_v ("SPS: frame_cropping_rect_top_offset"            , s);
    sps->frame_cropping_rect_bottom_offset    = ue_v ("SPS: frame_cropping_rect_bottom_offset"         , s);
  }
  sps->vui_parameters_present_flag           = u_1  ("SPS: vui_parameters_present_flag"            , s);
  if (sps->vui_parameters_present_flag)
  {
    printf ("VUI sequence parameters present but not supported, ignored, proceeding to next NALU\n");
  }
  sps->Valid = TRUE;
  return UsedBits;
}


int InterpretPPS (DataPartition *p, pic_parameter_set_rbsp_t *pps)
{
  unsigned i;
  int NumberBitsPerSliceGroupId;
  Bitstream *s = p->bitstream;
  
  assert (p != NULL);
  assert (p->bitstream != NULL);
  assert (p->bitstream->streamBuffer != 0);
  assert (pps != NULL);

  UsedBits = 0;

  pps->pic_parameter_set_id                  = ue_v ("PPS: pic_parameter_set_id"                   , s);
  pps->seq_parameter_set_id                  = ue_v ("PPS: seq_parameter_set_id"                   , s);
  pps->entropy_coding_mode_flag              = u_1  ("PPS: entropy_coding_mode_flag"               , s);

  //! Note: as per JVT-F078 the following bit is unconditional.  If F078 is not accepted, then
  //! one has to fetch the correct SPS to check whether the bit is present (hopefully there is
  //! no consistency problem :-(
  //! The current encoder code handles this in the same way.  When you change this, don't forget
  //! the encoder!  StW, 12/8/02
  pps->pic_order_present_flag                = u_1  ("PPS: pic_order_present_flag"                 , s);

  pps->num_slice_groups_minus1               = ue_v ("PPS: num_slice_groups_minus1"                , s);

  // FMO stuff begins here
  if (pps->num_slice_groups_minus1 > 0)
  {
    pps->slice_group_map_type               = ue_v ("PPS: slice_group_map_type"                , s);
    if (pps->slice_group_map_type == 0)
    {
      for (i=0; i<=pps->num_slice_groups_minus1; i++)
        pps->run_length_minus1 [i]                  = ue_v ("PPS: run_length_minus1 [i]"              , s);
    }
    else if (pps->slice_group_map_type == 2)
    {
      for (i=0; i<pps->num_slice_groups_minus1; i++)
      {
        //! JVT-F078: avoid reference of SPS by using ue(v) instead of u(v)
        pps->top_left [i]                          = ue_v ("PPS: top_left [i]"                        , s);
        pps->bottom_right [i]                      = ue_v ("PPS: bottom_right [i]"                    , s);
      }
    }
    else if (pps->slice_group_map_type == 3 ||
             pps->slice_group_map_type == 4 ||
             pps->slice_group_map_type == 5)
    {
      pps->slice_group_change_direction_flag     = u_1  ("PPS: slice_group_change_direction_flag"      , s);
      pps->slice_group_change_rate_minus1        = ue_v ("PPS: slice_group_change_rate_minus1"         , s);
    }
    else if (pps->slice_group_map_type == 6)
    {
      if (pps->num_slice_groups_minus1+1 >4)
        NumberBitsPerSliceGroupId = 3;
      else if (pps->num_slice_groups_minus1+1 > 2)
        NumberBitsPerSliceGroupId = 2;
      else
        NumberBitsPerSliceGroupId = 1;
      //! JVT-F078, exlicitly signal number of MBs in the map
      pps->num_slice_group_map_units_minus1      = ue_v ("PPS: num_slice_group_map_units_minus1"               , s);
      for (i=0; i<=pps->num_slice_group_map_units_minus1; i++)
        pps->slice_group_id[i] = u_v (NumberBitsPerSliceGroupId, "slice_group_id[i]", s);
    }
  }

  // End of FMO stuff

  pps->num_ref_idx_l0_active_minus1          = ue_v ("PPS: num_ref_idx_l0_active_minus1"           , s);
  pps->num_ref_idx_l1_active_minus1          = ue_v ("PPS: num_ref_idx_l1_active_minus1"           , s);
  pps->weighted_pred_flag                    = u_1  ("PPS: weighted prediction flag"               , s);
  pps->weighted_bipred_idc                   = u_v  ( 2, "PPS: weighted_bipred_idc"                , s);
  pps->pic_init_qp_minus26                   = se_v ("PPS: pic_init_qp_minus26"                    , s);
  pps->pic_init_qs_minus26                   = se_v ("PPS: pic_init_qs_minus26"                    , s);
  pps->chroma_qp_index_offset                = se_v ("PPS: chroma_qp_index_offset"                 , s);
  pps->deblocking_filter_control_present_flag = u_1 ("PPS: deblocking_filter_control_present_flag" , s);
  pps->constrained_intra_pred_flag           = u_1  ("PPS: constrained_intra_pred_flag"            , s);
  pps->redundant_pic_cnt_present_flag        = u_1  ("PPS: redundant_pic_cnt_present_flag"         , s);

  pps->Valid = TRUE;
  return UsedBits;
}


void PPSConsistencyCheck (pic_parameter_set_rbsp_t *pps)
{
  printf ("Consistency checking a picture parset, to be implemented\n");
//  if (pps->seq_parameter_set_id invalid then do something)
}

void SPSConsistencyCheck (seq_parameter_set_rbsp_t *sps)
{
  printf ("Consistency checking a sequence parset, to be implemented\n");
}

void MakePPSavailable (int id, pic_parameter_set_rbsp_t *pps)
{
  assert (pps->Valid == TRUE);

  if (PicParSet[id].Valid == TRUE && PicParSet[id].slice_group_id != NULL)
    free (PicParSet[id].slice_group_id);

  memcpy (&PicParSet[id], pps, sizeof (pic_parameter_set_rbsp_t));

  if ((PicParSet[id].slice_group_id = calloc (PicParSet[id].num_slice_group_map_units_minus1+1, sizeof(int))) == NULL)
    no_mem_exit ("MakePPSavailable: Cannot calloc slice_group_id");
  
  memcpy (PicParSet[id].slice_group_id, pps->slice_group_id, (pps->num_slice_group_map_units_minus1+1)*sizeof(int));
}

void MakeSPSavailable (int id, seq_parameter_set_rbsp_t *sps)
{
  assert (sps->Valid == TRUE);
  memcpy (&SeqParSet[id], sps, sizeof (seq_parameter_set_rbsp_t));
}


void ProcessSPS (NALU_t *nalu)
{
  DataPartition *dp = AllocPartition(1);
  seq_parameter_set_rbsp_t *sps = AllocSPS();
  int dummy;
  
  memcpy (dp->bitstream->streamBuffer, &nalu->buf[1], nalu->len-1);
  dp->bitstream->code_len = dp->bitstream->bitstream_length = RBSPtoSODB (dp->bitstream->streamBuffer, nalu->len-1);
  dp->bitstream->ei_flag = 0;
  dp->bitstream->read_len = dp->bitstream->frame_bitoffset = 0;
  dummy = InterpretSPS (dp, sps);

  if (active_sps)
  {
    if (sps->seq_parameter_set_id == active_sps->seq_parameter_set_id)
    {
      if (!sps_is_equal(sps, active_sps))
      {
        if (dec_picture)
        {
          // this may only happen on slice loss
          exit_picture();
        }
        active_sps=NULL;
      }
    }
  }
  // SPSConsistencyCheck (pps);
  MakeSPSavailable (sps->seq_parameter_set_id, sps);

  FreePartition (dp, 1);
  FreeSPS (sps);
}


void ProcessPPS (NALU_t *nalu)
{
  DataPartition *dp;
  pic_parameter_set_rbsp_t *pps;
  int dummy;

  dp = AllocPartition(1);
  pps = AllocPPS();
  memcpy (dp->bitstream->streamBuffer, &nalu->buf[1], nalu->len-1);
  dp->bitstream->code_len = dp->bitstream->bitstream_length = RBSPtoSODB (dp->bitstream->streamBuffer, nalu->len-1);
  dp->bitstream->ei_flag = 0;
  dp->bitstream->read_len = dp->bitstream->frame_bitoffset = 0;
  dummy = InterpretPPS (dp, pps);
  // PPSConsistencyCheck (pps);
  if (active_pps)
  {
    if (pps->pic_parameter_set_id == active_pps->pic_parameter_set_id)
    {
      if (!pps_is_equal(pps, active_pps))
      {
        if (dec_picture)
        {
          // this may only happen on slice loss
          exit_picture();
        }
        active_pps = NULL;
      }
    }
  }
  MakePPSavailable (pps->pic_parameter_set_id, pps);
  FreePartition (dp, 1);
  FreePPS (pps);
}

void activate_sps (seq_parameter_set_rbsp_t *sps)
{
  if (active_sps != sps)
  {
    if (dec_picture)
    {
      // this may only happen on slice loss
      exit_picture();
    }
    active_sps = sps;
    
    img->MaxFrameNum = 1<<(sps->log2_max_frame_num_minus4+4);
    img->PicWidthInMbs = (sps->pic_width_in_mbs_minus1 +1);
    img->PicHeightInMapUnits = (sps->pic_height_in_map_units_minus1 +1);
    img->FrameHeightInMbs = ( 2 - sps->frame_mbs_only_flag ) * img->PicHeightInMapUnits;
    img->FrameSizeInMbs = img->PicWidthInMbs * img->FrameHeightInMbs;
    
    img->width = img->PicWidthInMbs * MB_BLOCK_SIZE;
    img->width_cr = img->width /2;
    img->height = img->FrameHeightInMbs * MB_BLOCK_SIZE;
    img->height_cr = img->height / 2;
    
    init_global_buffers();
    if (!img->no_output_of_prior_pics_flag)
    {
      flush_dpb();
    }
    init_dpb();
    
    if (NULL!=Co_located)
    {
      free_collocated(Co_located);
    }
    Co_located = alloc_colocated (img->width, img->height,sps->mb_adaptive_frame_field_flag);
    ercInit(img->width, img->height, 1);
  }
}

void activate_pps(pic_parameter_set_rbsp_t *pps)
{
  if (active_pps != pps)
  {
    if (dec_picture)
    {
      // this may only happen on slice loss
      exit_picture();
    }
    active_pps = pps;
  }

}

void UseParameterSet (int PicParsetId)
{
  seq_parameter_set_rbsp_t *sps = &SeqParSet[PicParSet[PicParsetId].seq_parameter_set_id];
  pic_parameter_set_rbsp_t *pps = &PicParSet[PicParsetId];
  int i;


  if (PicParSet[PicParsetId].Valid != TRUE)
    printf ("Trying to use an invalid (uninitialized) Picture Parameter Set with ID %d, expect the unexpected...\n", PicParsetId);
  if (SeqParSet[PicParSet[PicParsetId].seq_parameter_set_id].Valid != TRUE)
    printf ("PicParset %d references an invalid (uninitialized) Sequence Parameter Set with ID %d, expect the unexpected...\n", PicParsetId, PicParSet[PicParsetId].seq_parameter_set_id);

  sps =  &SeqParSet[PicParSet[PicParsetId].seq_parameter_set_id];

  
  // In theory, and with a well-designed software, the lines above
  // are everything necessary.  In practice, we need to patch many values
  // in img-> (but no more in inp-> -- these have been taken care of)

  // Sequence Parameter Set Stuff first

//  printf ("Using Picture Parameter set %d and associated Sequence Parameter Set %d\n", PicParsetId, PicParSet[PicParsetId].seq_parameter_set_id);

  if (sps->pic_order_cnt_type < 0 || sps->pic_order_cnt_type > 2)  // != 1
  {
    printf ("invalid sps->pic_order_cnt_type = %d\n", sps->pic_order_cnt_type);
    error ("pic_order_cnt_type != 1", -1000);
  }

  if (sps->pic_order_cnt_type == 1)
  {
    if(sps->num_ref_frames_in_pic_order_cnt_cycle >= MAXnum_ref_frames_in_pic_order_cnt_cycle)
    {
      error("num_ref_frames_in_pic_order_cnt_cycle too large",-1011);
    }
  }
  
  activate_sps(sps);
  activate_pps(pps);

  // currSlice->dp_mode is set by read_new_slice (NALU first byte available there)
  if (pps->entropy_coding_mode_flag == UVLC)
  {
    nal_startcode_follows = uvlc_startcode_follows;
    for (i=0; i<3; i++)
    {
      img->currentSlice->partArr[i].readSyntaxElement = readSyntaxElement_UVLC;
    }
  }
  else
  {
    nal_startcode_follows = cabac_startcode_follows;
    for (i=0; i<3; i++)
    {
      img->currentSlice->partArr[i].readSyntaxElement = readSyntaxElement_CABAC;
    }
  }
}

