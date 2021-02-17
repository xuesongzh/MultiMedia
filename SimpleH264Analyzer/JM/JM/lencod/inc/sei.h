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
 *     sei.h
 *  \brief
 *     definitions for Supplemental Enhanced Information
 *  \author(s)
 *      - Dong Tian                             <tian@cs.tut.fi>
 *      - TBD
 *
 * ************************************************************************
 */

#ifndef SEI_H
#define SEI_H

#define MAX_LAYER_NUMBER 2
#define MAX_DEPENDENT_SUBSEQ 5


//! definition of SEI payload type
typedef enum {
  SEI_ZERO,        //!< 0 is undefined, useless
  SEI_TEMPORAL_REF,
  SEI_CLOCK_TIMESTAMP,
  SEI_PANSCAN_RECT,
  SEI_BUFFERING_PERIOD,
  SEI_HRD_PICTURE,
  SEI_FILLER_PAYLOAD,
  SEI_USER_DATA_REGISTERED_ITU_T_T35,
  SEI_USER_DATA_UNREGISTERED,
  SEI_RANDOM_ACCESS_POINT,
  SEI_REF_PIC_BUFFER_MANAGEMENT_REPETITION,
  SEI_SPARE_PICTURE,
  SEI_SCENE_INFORMATION,
  SEI_SUBSEQ_INFORMATION,
  SEI_SUBSEQ_LAYER_CHARACTERISTICS,
  SEI_SUBSEQ_CHARACTERISTICS,
  SEI_MAX_ELEMENTS  //!< number of maximum syntax elements
} SEI_type;

#define MAX_FN 256

#define AGGREGATION_PACKET_TYPE 4
#define SEI_PACKET_TYPE 5  // Tian Dong: See VCEG-N72, it need updates

#define NORMAL_SEI 0
#define AGGREGATION_SEI 1

//! SEI structure
typedef struct
{
  Boolean available;
  int payloadSize;
  unsigned char subPacketType;
  byte* data;
} sei_struct;

//!< sei_message[0]: this struct is to store the sei message packtized independently 
//!< sei_message[1]: this struct is to store the sei message packtized together with slice data
extern sei_struct sei_message[2];

void InitSEIMessages();
void CloseSEIMessages();
Boolean HaveAggregationSEI();
void write_sei_message(int id, byte* payload, int payload_size, int payload_type);
void finalize_sei_message(int id);
void clear_sei_message(int id);
void AppendTmpbits2Buf( Bitstream* dest, Bitstream* source );

void PrepareAggregationSEIMessage();


//! Spare Picture
typedef struct
{
  int target_frame_num;
  int num_spare_pics;
  int payloadSize;
  Bitstream* data;
} spare_picture_struct;

extern Boolean seiHasSparePicture;
//extern Boolean sei_has_sp;
extern spare_picture_struct seiSparePicturePayload;

void InitSparePicture();
void CloseSparePicture();
void CalculateSparePicture();
void ComposeSparePictureMessage(int delta_spare_frame_num, int ref_area_indicator, Bitstream *tmpBitstream);
Boolean CompressSpareMBMap(unsigned char **map_sp, Bitstream *bitstream);
void FinalizeSpareMBMap();

//! Subseq Information
typedef struct
{
  int subseq_layer_num;
  int subseq_id;
  unsigned int last_picture_flag;
  unsigned int stored_frame_cnt;

  int payloadSize;
  Bitstream* data;
} subseq_information_struct;

extern Boolean seiHasSubseqInfo;
extern subseq_information_struct seiSubseqInfo[MAX_LAYER_NUMBER];

void InitSubseqInfo(int currLayer);
void UpdateSubseqInfo(int currLayer);
void FinalizeSubseqInfo(int currLayer);
void ClearSubseqInfoPayload(int currLayer);
void CloseSubseqInfo(int currLayer);

//! Subseq Layer Information
typedef struct
{
  unsigned short bit_rate[MAX_LAYER_NUMBER];
  unsigned short frame_rate[MAX_LAYER_NUMBER];
  byte data[4*MAX_LAYER_NUMBER];
  int layer_number;
  int payloadSize;
} subseq_layer_information_struct;

extern Boolean seiHasSubseqLayerInfo;
extern subseq_layer_information_struct seiSubseqLayerInfo;

void InitSubseqLayerInfo();
void CloseSubseqLayerInfo();
void FinalizeSubseqLayerInfo();

//! Subseq Characteristics
typedef struct
{
  int subseq_layer_num;
  int subseq_id;
  int duration_flag;
  unsigned int subseq_duration;
  unsigned int average_rate_flag;
  unsigned int average_bit_rate;
  unsigned int average_frame_rate;
  int num_referenced_subseqs;
  int ref_subseq_layer_num[MAX_DEPENDENT_SUBSEQ];
  int ref_subseq_id[MAX_DEPENDENT_SUBSEQ];

  Bitstream* data;
  int payloadSize;
} subseq_char_information_struct;

extern Boolean seiHasSubseqChar;
extern subseq_char_information_struct seiSubseqChar;

void InitSubseqChar();
void ClearSubseqCharPayload();
void UpdateSubseqChar();
void FinalizeSubseqChar();
void CloseSubseqChar();


//! JVT-D099 Scene information SEI message
typedef struct
{
  int scene_id;
  int scene_transition_type;
  int second_scene_id;

  Bitstream* data;
  int payloadSize;
} scene_information_struct;

extern Boolean seiHasSceneInformation;
extern scene_information_struct seiSceneInformation;

void InitSceneInformation();
void CloseSceneInformation();
void UpdateSceneInformation(Boolean HasSceneInformation, int sceneID, int sceneTransType, int secondSceneID);
void FinalizeSceneInformation();
//! End JVT-D099 Scene information SEI message

//! Shankar Regunathan Oct 2002
//! PanScanRect Information
typedef struct
{
  int pan_scan_rect_id; 
  int pan_scan_rect_left_offset;
  int pan_scan_rect_right_offset;
  int pan_scan_rect_top_offset;
  int pan_scan_rect_bottom_offset;

  Bitstream *data;
  int payloadSize;
} panscanrect_information_struct;

extern Boolean seiHasPanScanRectInfo;
extern panscanrect_information_struct seiPanScanRectInfo;

void InitPanScanRectInfo();
void ClearPanScanRectInfoPayload();
void UpdatePanScanRectInfo();
void FinalizePanScanRectInfo();
void ClosePanScanRectInfo();

//! User_data_unregistered Information
typedef struct
{
  char *byte;
  int total_byte;
  Bitstream *data;
  int payloadSize;
} user_data_unregistered_information_struct;
Boolean seiHasUser_data_unregistered_info;
user_data_unregistered_information_struct seiUser_data_unregistered;

void InitUser_data_unregistered();
void ClearUser_data_unregistered();
void UpdateUser_data_unregistered();
void FinalizeUser_data_unregistered();
void CloseUser_data_unregistered();

//! User_data_registered_itu_t_t35 Information
typedef struct
{
  char *byte;
  int total_byte;
  int itu_t_t35_country_code;
  int itu_t_t35_country_code_extension_byte;
  Bitstream *data;
  int payloadSize;
} user_data_registered_itu_t_t35_information_struct;
Boolean seiHasUser_data_registered_itu_t_t35_info;
user_data_registered_itu_t_t35_information_struct seiUser_data_registered_itu_t_t35;

void InitUser_data_registered_itu_t_t35();
void ClearUser_data_registered_itu_t_t35();
void UpdateUser_data_registered_itu_t_t35();
void FinalizeUser_data_registered_itu_t_t35();
void CloseUser_data_registered_itu_t_t35();

//! RandomAccess Information
typedef struct
{
  unsigned char recovery_point_flag;
  unsigned char exact_match_flag;
  unsigned char broken_link_flag;

  Bitstream *data;
  int payloadSize;
} randomaccess_information_struct;
Boolean seiHasRandomAccess_info;
randomaccess_information_struct seiRandomAccess;

void InitRandomAccess();
void ClearRandomAccess();
void UpdateRandomAccess();
void FinalizeRandomAccess();
void CloseRandomAccess();

#endif
