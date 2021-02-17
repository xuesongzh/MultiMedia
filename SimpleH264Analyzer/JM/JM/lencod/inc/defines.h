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

typedef unsigned char byte;    //!< byte type definition

#define LIST_0 0
#define LIST_1 1
#define ZEROSNR 1
// CAVLC
#define LUMA              0
#define LUMA_INTRA16x16DC 1
#define LUMA_INTRA16x16AC 2

#define LEVEL_NUM      6
#define TOTRUN_NUM    15
#define RUNBEFORE_NUM  7

//--- block types for CABAC
#define LUMA_16DC       0
#define LUMA_16AC       1
#define LUMA_8x8        2
#define LUMA_8x4        3
#define LUMA_4x8        4
#define LUMA_4x4        5
#define CHROMA_DC       6
#define CHROMA_AC       7
#define NUM_BLOCK_TYPES 8


#define _FAST_FULL_ME_

#define _FULL_SEARCH_RANGE_
#define _ADAPT_LAST_GROUP_
#define _CHANGE_QP_
#define _LEAKYBUCKET_

// ---------------------------------------------------------------------------------
// FLAGS and DEFINES for new chroma intra prediction, Dzung Hoang
// Threshold values to zero out quantized transform coefficients.
// Recommend that _CHROMA_COEFF_COST_ be low to improve chroma quality
#define _LUMA_COEFF_COST_       4 //!< threshold for luma coeffs
#define _CHROMA_COEFF_COST_     4 //!< threshold for chroma coeffs, used to be 7

#define IMG_PAD_SIZE    4   //!< Number of pixels padded around the reference frame (>=4)

#define absm(A) ((A)<(0) ? (-(A)):(A)) //!< abs macro, faster than procedure
#define MAX_VALUE       999999   //!< used for start value for some variables

#define INVALIDINDEX  (-135792468)

#define Clip1(a)            ((a)>255?255:((a)<0?0:(a)))
#define Clip3(min,max,val) (((val)<(min))?(min):(((val)>(max))?(max):(val)))

#define P8x8    8
#define I4MB    9
#define I16MB   10
#define IBLOCK  11
#define SI4MB   12
#define MAXMODE 15
#define IPCM    14


#define  LAMBDA_ACCURACY_BITS         16
#define  LAMBDA_FACTOR(lambda)        ((int)((double)(1<<LAMBDA_ACCURACY_BITS)*lambda+0.5))
#define  WEIGHTED_COST(factor,bits)   (((factor)*(bits))>>LAMBDA_ACCURACY_BITS)
#define  MV_COST(f,s,cx,cy,px,py)     (WEIGHTED_COST(f,mvbits[((cx)<<(s))-px]+mvbits[((cy)<<(s))-py]))
//#define  REF_COST(f,ref)              (WEIGHTED_COST(f,refbits[(ref)]))

#define  REF_COST(f,ref,list_offset) (WEIGHTED_COST(f,((listXsize[list_offset]<=1)? 0:refbits[(ref)])))

#define IS_INTRA(MB)    ((MB)->mb_type==I4MB  || (MB)->mb_type==I16MB)
#define IS_NEWINTRA(MB) ((MB)->mb_type==I16MB)
#define IS_OLDINTRA(MB) ((MB)->mb_type==I4MB)
#define IS_INTER(MB)    ((MB)->mb_type!=I4MB  && (MB)->mb_type!=I16MB)
#define IS_INTERMV(MB)  ((MB)->mb_type!=I4MB  && (MB)->mb_type!=I16MB  && (MB)->mb_type!=0)
#define IS_DIRECT(MB)   ((MB)->mb_type==0     && (img ->   type==B_SLICE))
#define IS_COPY(MB)     ((MB)->mb_type==0     && (img ->type==P_SLICE||img ->type==SP_SLICE));
#define IS_P8x8(MB)     ((MB)->mb_type==P8x8)

// Quantization parameter range

#define MIN_QP          0
#define MAX_QP          51
#define SHIFT_QP        12

#define LOG2_MAX_FRAME_NUM_MINUS4   7           // POC200301 moved from defines.h
#define LOG2_MAX_PIC_ORDER_CNT_LSB_MINUS4 7     // POC200301 newly added

// Direct Mode types
#define DIR_TEMPORAL    0   //!< Temporal Direct Mode
#define DIR_SPATIAL     1    //!< Spatial Direct Mode

#define MAX_REFERENCE_PICTURES 15

#define BLOCK_SIZE      4
#define MB_BLOCK_SIZE   16

#define NO_INTRA_PMODE  9        //!< #intra prediction modes
//!< 4x4 intra prediction modes
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

#define INIT_FRAME_RATE 30
#define EOS             1         //!< End Of Sequence


#define MVPRED_MEDIAN   0
#define MVPRED_L        1
#define MVPRED_U        2
#define MVPRED_UR       3


#define BLOCK_MULTIPLE      (MB_BLOCK_SIZE/BLOCK_SIZE)

#define MAX_SYMBOLS_PER_MB  1200  //!< Maximum number of different syntax elements for one MB
                                  // CAVLC needs more symbols per MB


#define MAX_PART_NR     3 /*!< Maximum number of different data partitions.
                               Some reasonable number which should reflect
                               what is currently defined in the SE2Partition map (elements.h) */

#define MAXPICTURETYPESEQUENCELEN 100   /*!< Maximum size of the string that defines the picture
                                             types to be coded, e.g. "IBBPBBPBB" */
//Start code and Emulation Prevention need this to be defined in identical manner at encoder and decoder
#define ZEROBYTES_SHORTSTARTCODE 2 //indicates the number of zero bytes in the short start-code prefix
#define BIPRED_SIMPLE 1

#endif

