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
 ***********************************************************************
 *  \file
 *     configfile.h
 *  \brief
 *     Prototypes for configfile.c and definitions of used structures.
 ***********************************************************************
 */

#ifndef _CONFIGFILE_H_
#define _CONFIGFILE_H_


#define DEFAULTCONFIGFILENAME "encoder.cfg"

#define PROFILE_IDC     88
#define LEVEL_IDC       21


typedef struct {
  char *TokenName;
  void *Place;
  int Type;
} Mapping;



InputParameters configinput;


#ifdef INCLUDED_BY_CONFIGFILE_C

Mapping Map[] = {
    {"ProfileIDC",               &configinput.ProfileIDC,              0},
    {"LevelIDC",                 &configinput.LevelIDC,                0},
    {"FrameRate",                &configinput.FrameRate,               0},
    {"IDRIntraEnable",           &configinput.idr_enable,              0},
    {"StartFrame",               &configinput.start_frame,             0},
    {"IntraPeriod",              &configinput.intra_period,            0},
    {"FramesToBeEncoded",        &configinput.no_frames,               0},
    {"QPFirstFrame",             &configinput.qp0,                     0},
    {"QPRemainingFrame",         &configinput.qpN,                     0},
    {"FrameSkip",                &configinput.jumpd,                   0},
    {"UseHadamard",              &configinput.hadamard,                0},
    {"SearchRange",              &configinput.search_range,            0},
    {"NumberReferenceFrames",    &configinput.num_reference_frames,    0},
    {"PList0References",         &configinput.P_List0_refs,            0},
    {"BList0References",         &configinput.B_List0_refs,            0},
    {"BList1References",         &configinput.B_List1_refs,            0},
    {"SourceWidth",              &configinput.img_width,               0},
    {"SourceHeight",             &configinput.img_height,              0},
    {"MbLineIntraUpdate",        &configinput.intra_upd,               0},
    {"SliceMode",                &configinput.slice_mode,              0},
    {"SliceArgument",            &configinput.slice_argument,          0},
    {"UseConstrainedIntraPred",  &configinput.UseConstrainedIntraPred, 0},
    {"InputFile",                &configinput.infile,                  1},
    {"InputHeaderLength",        &configinput.infile_header,           0},
    {"OutputFile",               &configinput.outfile,                 1},
    {"ReconFile",                &configinput.ReconFile,               1},
    {"TraceFile",                &configinput.TraceFile,               1},
    {"NumberBFrames",            &configinput.successive_Bframe,       0},
    {"QPBPicture",               &configinput.qpB,                     0},
    {"DirectModeType",           &configinput.direct_type,             0},
    {"DirectInferenceFlag",      &configinput.directInferenceFlag,     0},
    {"SPPicturePeriodicity",     &configinput.sp_periodicity,          0},
    {"QPSPPicture",              &configinput.qpsp,                    0},
    {"QPSP2Picture",             &configinput.qpsp_pred,               0},
    {"SymbolMode",               &configinput.symbol_mode,             0},
    {"OutFileMode",              &configinput.of_mode,                 0},
    {"PartitionMode",            &configinput.partition_mode,          0},
    {"PictureTypeSequence",      &configinput.PictureTypeSequence,     1},
    {"InterSearch16x16",         &configinput.InterSearch16x16,        0},
    {"InterSearch16x8",          &configinput.InterSearch16x8 ,        0},
    {"InterSearch8x16",          &configinput.InterSearch8x16,         0},
    {"InterSearch8x8",           &configinput.InterSearch8x8 ,         0},
    {"InterSearch8x4",           &configinput.InterSearch8x4,          0},
    {"InterSearch4x8",           &configinput.InterSearch4x8,          0},
    {"InterSearch4x4",           &configinput.InterSearch4x4,          0},
#ifdef _FULL_SEARCH_RANGE_
    {"RestrictSearchRange",      &configinput.full_search,             0},
#endif
#ifdef _ADAPT_LAST_GROUP_
    {"LastFrameNumber",          &configinput.last_frame,              0},
#endif
#ifdef _CHANGE_QP_
    {"ChangeQPI",                &configinput.qp02,                    0},
    {"ChangeQPP",                &configinput.qpN2,                    0},
    {"ChangeQPB",                &configinput.qpB2,                    0},
    {"ChangeQPStart",            &configinput.qp2start,                0},
#endif
    {"RDOptimization",           &configinput.rdopt,                   0},
    {"LossRateA",                &configinput.LossRateA,               0},
    {"LossRateB",                &configinput.LossRateB,               0},
    {"LossRateC",                &configinput.LossRateC,               0},
    {"NumberOfDecoders",         &configinput.NoOfDecoders,            0},
    {"RestrictRefFrames",        &configinput.RestrictRef ,            0},
#ifdef _LEAKYBUCKET_
    {"NumberofLeakyBuckets",     &configinput.NumberLeakyBuckets,      0},
    {"LeakyBucketRateFile",      &configinput.LeakyBucketRateFile,     1},
    {"LeakyBucketParamFile",     &configinput.LeakyBucketParamFile,    1},
#endif
    {"PicInterlace",             &configinput.PicInterlace,            0},
    {"MbInterlace",              &configinput.MbInterlace,             0},

    {"IntraBottom",              &configinput.IntraBottom,             0},

    {"NumberFramesInEnhancementLayerSubSequence", &configinput.NumFramesInELSubSeq, 0},
    {"NumberOfFrameInSecondIGOP",&configinput.NumFrameIn2ndIGOP, 0},
    {"RandomIntraMBRefresh",     &configinput.RandomIntraMBRefresh,    0},
		
		
    {"WeightedPrediction",       &configinput.WeightedPrediction,      0},
    {"WeightedBiprediction",     &configinput.WeightedBiprediction,    0},
    {"StoredBPictures",          &configinput.StoredBPictures,         0},
    {"LoopFilterParametersFlag", &configinput.LFSendParameters,        0},
    {"LoopFilterDisable",        &configinput.LFDisableIdc,            0},
    {"LoopFilterAlphaC0Offset",  &configinput.LFAlphaC0Offset,         0},
    {"LoopFilterBetaOffset",     &configinput.LFBetaOffset,            0},
    {"SparePictureOption",       &configinput.SparePictureOption,      0},
    {"SparePictureDetectionThr", &configinput.SPDetectionThreshold,    0},
    {"SparePicturePercentageThr",&configinput.SPPercentageThreshold,   0},

    {"num_slice_groups_minus1",           &configinput.num_slice_groups_minus1,           0},
    {"slice_group_map_type",              &configinput.slice_group_map_type,              0},
    {"slice_group_change_direction_flag", &configinput.slice_group_change_direction_flag, 0},
    {"slice_group_change_rate_minus1",    &configinput.slice_group_change_rate_minus1,    0},
    {"SliceGroupConfigFileName",          &configinput.SliceGroupConfigFileName,          1},
		

    {"UseRedundantSlice",        &configinput.redundant_slice_flag,    0},
    {"PicOrderCntType",          &configinput.pic_order_cnt_type,      0},

    {"ContextInitMethod",        &configinput.context_init_method,     0},
    {"FixedModelNumber",         &configinput.model_number,            0},

    // Rate Control
    {"RateControlEnable",        &configinput.RCEnable,                0},
    {"Bitrate",                  &configinput.bit_rate,                0},
    {"InitialQP",                &configinput.SeinitialQP,             0},
    {"BasicUnit",                &configinput.basicunit,               0},
    {"ChannelType",              &configinput.channel_type,            0},

    // Fast ME enable
    {"UseFME",                   &configinput.FMEnable,                0},
    
    {"ChromaQPOffset",           &configinput.chroma_qp_index_offset,  0},    
    {NULL,                       NULL,                                -1}
};

#endif

#ifndef INCLUDED_BY_CONFIGFILE_C
extern Mapping Map[];
#endif


void Configure (int ac, char *av[]);
void PatchInputNoFrames();

#endif

