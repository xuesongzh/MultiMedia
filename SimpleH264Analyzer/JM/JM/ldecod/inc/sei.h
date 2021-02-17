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
 * \file sei.h
 *
 * \brief
 *    Prototypes for sei.h
 *************************************************************************************
 */

#ifndef SEI_H
#define SEI_H

//! definition of SEI payload type
typedef enum {
  SEI_BUFFERING_PERIOD = 0,
  SEI_PIC_TIMING,
  SEI_PAN_SCAN_RECT,
  SEI_FILLER_PAYLOAD,
  SEI_USER_DATA_REGISTERED_ITU_T_T35,
  SEI_USER_DATA_UNREGISTERED,
  SEI_RANDOM_ACCESS_POINT,
  SEI_DEC_REF_PIC_MARKING_REPETITION,
  SEI_SPARE_PIC,
  SEI_SCENE_INFO,
  SEI_SUB_SEQ_INFO,
  SEI_SUB_SEQ_LAYER_CHARACTERISTICS,
  SEI_SUB_SEQ_CHARACTERISTICS,
  SEI_FULL_FRAME_FREEZE,
  SEI_FULL_FRAME_FREEZE_RELEASE,
  SEI_FULL_FRAME_SNAPSHOT,
  SEI_PROGRESSIVE_REFINEMENT_SEGMENT_START,
  SEI_PROGRESSIVE_REFINEMENT_SEGMENT_END,
  SEI_MOTION_CONSTRAINED_SLICE_GROUP_SET,

  SEI_MAX_ELEMENTS  //!< number of maximum syntax elements
} SEI_type;

#define MAX_FN 256

void InterpretSEIMessage(byte* msg, int size, ImageParameters *img);
void interpret_spare_pic( byte* payload, int size, ImageParameters *img );
void interpret_subsequence_info( byte* payload, int size, ImageParameters *img );
void interpret_subsequence_layer_characteristics_info( byte* payload, int size, ImageParameters *img );
void interpret_subsequence_characteristics_info( byte* payload, int size, ImageParameters *img );
void interpret_scene_information( byte* payload, int size, ImageParameters *img ); // JVT-D099
void interpret_user_data_registered_itu_t_t35_info( byte* payload, int size, ImageParameters *img );
void interpret_user_data_unregistered_info( byte* payload, int size, ImageParameters *img );
void interpret_pan_scan_rect_info( byte* payload, int size, ImageParameters *img );
void interpret_random_access_info( byte* payload, int size, ImageParameters *img );
void interpret_filler_payload_info( byte* payload, int size, ImageParameters *img );
void interpret_dec_ref_pic_marking_repetition_info( byte* payload, int size, ImageParameters *img );
void interpret_full_frame_freeze_info( byte* payload, int size, ImageParameters *img );
void interpret_full_frame_freeze_release_info( byte* payload, int size, ImageParameters *img );
void interpret_full_frame_snapshot_info( byte* payload, int size, ImageParameters *img );
void interpret_progressive_refinement_start_info( byte* payload, int size, ImageParameters *img );
void interpret_progressive_refinement_end_info( byte* payload, int size, ImageParameters *img );
void interpret_motion_constrained_slice_group_set_info( byte* payload, int size, ImageParameters *img );
void interpret_reserved_info( byte* payload, int size, ImageParameters *img );
void interpret_buffering_period_info( byte* payload, int size, ImageParameters *img );
void interpret_picture_timing_info( byte* payload, int size, ImageParameters *img );

#endif
