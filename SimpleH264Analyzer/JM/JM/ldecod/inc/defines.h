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
 **************************************************************************
 * \file defines.h
 *
 * \brief
 *    Headerfile containing some useful global definitions
 *
 * \author
 *    Detlev Marpe  
 *    Copyright (C) 2000 HEINRICH HERTZ INSTITUTE All Rights Reserved.
 *
 * \date
 *    21. March 2001
 **************************************************************************
 */

#ifndef _DEFINES_H_
#define _DEFINES_H_

#if defined _DEBUG
#define TRACE           0                   //!< 0:Trace off 1:Trace on
#else
#define TRACE           0                   //!< 0:Trace off 1:Trace on
#endif

//#define PAIR_FIELDS_IN_OUTPUT

//#define MAX_NUM_SLICES 150
#define MAX_NUM_SLICES 50

// CAVLC
#define LUMA              0
#define LUMA_INTRA16x16DC 1
#define LUMA_INTRA16x16AC 2

#define TOTRUN_NUM    15
#define RUNBEFORE_NUM  7


//--- block types for CABAC ----
#define LUMA_16DC       0
#define LUMA_16AC       1
#define LUMA_8x8        2
#define LUMA_8x4        3
#define LUMA_4x8        4
#define LUMA_4x4        5
#define CHROMA_DC       6
#define CHROMA_AC       7
#define NUM_BLOCK_TYPES 8


#define MAX_CODED_FRAME_SIZE 8000000         //!< bytes for one frame

// #define _LEAKYBUCKET_

#define absm(A) ((A)<(0) ? (-(A)):(A))      //!< abs macro, faster than procedure

#define Clip1(a)            ((a)>255?255:((a)<0?0:(a)))
#define Clip3(min,max,val) (((val)<(min))?(min):(((val)>(max))?(max):(val)))

#define P8x8    8
#define I4MB    9
#define I16MB   10
#define IBLOCK  11
#define SI4MB   12
#define MAXMODE 13
#define IPCM    14

#define IS_INTRA(MB)    ((MB)->mb_type==I4MB  || (MB)->mb_type==I16MB ||(MB)->mb_type==IPCM || (MB)->mb_type==SI4MB)
#define IS_NEWINTRA(MB) ((MB)->mb_type==I16MB  || (MB)->mb_type==IPCM)

#define IS_OLDINTRA(MB) ((MB)->mb_type==I4MB)
#define IS_INTER(MB)    ((MB)->mb_type!=I4MB  && (MB)->mb_type!=I16MB  && (MB)->mb_type!=IPCM)
#define IS_INTERMV(MB)  ((MB)->mb_type!=I4MB  && (MB)->mb_type!=I16MB  && (MB)->mb_type!=0 && (MB)->mb_type!=IPCM)
#define IS_DIRECT(MB)   ((MB)->mb_type==0     && (img->type==B_SLICE ))
#define IS_COPY(MB)     ((MB)->mb_type==0     && (img->type==P_SLICE || img->type==SP_SLICE))
#define IS_P8x8(MB)     ((MB)->mb_type==P8x8)


// Quantization parameter range

#define MIN_QP          0
#define MAX_QP          51

#define BLOCK_SIZE      4
#define MB_BLOCK_SIZE   16


#define NO_INTRA_PMODE  9        //!< #intra prediction modes
/* 4x4 intra prediction modes */
#define VERT_PRED             0
#define HOR_PRED              1
#define DC_PRED               2
#define DIAG_DOWN_LEFT_PRED   3
#define DIAG_DOWN_RIGHT_PRED  4
#define VERT_RIGHT_PRED       5
#define HOR_DOWN_PRED         6
#define VERT_LEFT_PRED        7
#define HOR_UP_PRED           8

// 16x16 intra prediction modes
#define VERT_PRED_16    0
#define HOR_PRED_16     1
#define DC_PRED_16      2
#define PLANE_16        3

// 8x8 chroma intra prediction modes
#define DC_PRED_8       0
#define HOR_PRED_8      1
#define VERT_PRED_8     2
#define PLANE_8         3

#define EOS             1                       //!< End Of Sequence
#define SOP             2                       //!< Start Of Picture
#define SOS             3                       //!< Start Of Slice

#define DECODING_OK     0
#define SEARCH_SYNC     1
#define PICTURE_DECODED 2

#define MAX_REFERENCE_PICTURES 32               //!< H264 allows 32 fields

#define INVALIDINDEX  (-135792468)

#ifndef WIN32
#define max(a, b)      ((a) > (b) ? (a) : (b))  //!< Macro returning max value
#define min(a, b)      ((a) < (b) ? (a) : (b))  //!< Macro returning min value
#endif


#define MVPRED_MEDIAN   0
#define MVPRED_L        1
#define MVPRED_U        2
#define MVPRED_UR       3

#define DECODE_COPY_MB  0
#define DECODE_MB       1
//#define DECODE_MB_BFRAME 2

#define BLOCK_MULTIPLE      (MB_BLOCK_SIZE/BLOCK_SIZE)

//Start code and Emulation Prevention need this to be defined in identical manner at encoder and decoder
#define ZEROBYTES_SHORTSTARTCODE 2 //indicates the number of zero bytes in the short start-code prefix

#endif

