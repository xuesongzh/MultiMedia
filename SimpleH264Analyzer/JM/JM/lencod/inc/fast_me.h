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
 * \brief
 * Macro definitions and global variables for fast integer pel motion 
 * estimation and fractional pel motio estimation
 * \ Main contributors: (see contributors.h for copyright, address and affiliation details)
 *   Zhibo Chen         <chenzhibo@tsinghua.org.cn>
 *   JianFeng Xu        <fenax@video.mdc.tsinghua.edu.cn>  
 * \date   : 2003.4
 ************************************************************************
 */
#ifndef _FAST_ME_H_
#define _FAST_ME_H_
#include "global.h"
#include "mbuffer.h"

#define EARLY_TERMINATION  if(ref>0)  \
  {                                                                    \
  if ((min_mcost-pred_SAD_ref)<pred_SAD_ref*betaThird)             \
  goto third_step;                                             \
  else if((min_mcost-pred_SAD_ref)<pred_SAD_ref*betaSec)           \
  goto sec_step;                                               \
  }                                                                    \
  else if(blocktype>1)                                                 \
  {                                                                    \
  if ((min_mcost-pred_SAD_uplayer)<pred_SAD_uplayer*betaThird)     \
    {                                                                \
    goto third_step;                                             \
    }                                                                \
    else if((min_mcost-pred_SAD_uplayer)<pred_SAD_uplayer*betaSec)   \
    goto sec_step;                                               \
  }                                                                    \
  else                                                                 \
  {                                                                    \
  if ((min_mcost-pred_SAD_space)<pred_SAD_space*betaThird)         \
    {                                                                \
    goto third_step;                                             \
    }                                                                \
    else if((min_mcost-pred_SAD_space)<pred_SAD_space*betaSec)       \
    goto sec_step;                                               \
  }


#define SEARCH_ONE_PIXEL  if(abs(cand_x - center_x) <=search_range && abs(cand_y - center_y)<= search_range) \
    { \
    if(!McostState[cand_y-center_y+search_range][cand_x-center_x+search_range]) \
    { \
    mcost = MV_COST (lambda_factor, mvshift, cand_x, cand_y, pred_x, pred_y); \
    mcost = PartCalMad(ref_pic, orig_pic, get_ref_line,blocksize_y,blocksize_x,blocksize_x4,mcost,min_mcost,cand_x,cand_y); \
    McostState[cand_y-center_y+search_range][cand_x-center_x+search_range] = mcost; \
    if (mcost < min_mcost) \
    { \
    best_x = cand_x; \
    best_y = cand_y; \
    min_mcost = mcost; \
    } \
    } \
    }
#define SEARCH_ONE_PIXEL1(value_iAbort) if(abs(cand_x - center_x) <=search_range && abs(cand_y - center_y)<= search_range) \
      { \
      if(!McostState[cand_y-center_y+search_range][cand_x-center_x+search_range]) \
        { \
        mcost = MV_COST (lambda_factor, mvshift, cand_x, cand_y, pred_x, pred_y); \
        mcost = PartCalMad(ref_pic, orig_pic, get_ref_line,blocksize_y,blocksize_x,blocksize_x4,mcost,min_mcost,cand_x,cand_y); \
        McostState[cand_y-center_y+search_range][cand_x-center_x+search_range] = mcost; \
        if (mcost < min_mcost) \
          { \
          best_x = cand_x; \
          best_y = cand_y; \
          min_mcost = mcost; \
          iAbort = value_iAbort; \
          } \
        } \
      }


int **McostState; //state for integer pel search

int *****all_mincost;//store the MV and SAD information needed;
int *****all_bwmincost;//store for backward prediction
int pred_SAD_space,pred_SAD_time,pred_SAD_ref,pred_SAD_uplayer;//SAD prediction
int FME_blocktype;  //blocktype for FME SetMotionVectorPredictor
int pred_MV_time[2],pred_MV_ref[2],pred_MV_uplayer[2];//pred motion vector by space or tempral correlation,Median is provided

//for early termination
float Quantize_step;
float  Bsize[8];
int Thresh4x4;
float AlphaSec[8];
float AlphaThird[8];
int  flag_intra[124];//HD enough
int  flag_intra_SAD;
void DefineThreshold();
void DefineThresholdMB();

char **SearchState; //state for fractional pel search
void DefineThreshold();
void DefineThresholdMB();
int get_mem_mincost (int****** mv);
int get_mem_bwmincost (int****** mv);
int get_mem_FME();
void free_mem_mincost (int***** mv);
void free_mem_bwmincost (int***** mv);
void free_mem_FME();
void   decide_intrabk_SAD();
void skip_intrabk_SAD(int best_mode, int ref_max);

int                                     //  ==> minimum motion cost after search
FastIntegerPelBlockMotionSearch  (pel_t**   orig_pic,     // <--  not used
                                  int       ref,          // <--  reference frame (0... or -1 (backward))
                                  int       list,
                                  int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB block
                                  int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
                                  int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
                                  int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
                                  int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
                                  int*      mv_x,         //  --> motion vector (x) - in pel units
                                  int*      mv_y,         //  --> motion vector (y) - in pel units
                                  int       search_range, // <--  1-d search range in pel units                         
                                  int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
                                  double    lambda) ;      // <--  lagrangian parameter for determining motion cost

int AddUpSADQuarter(int pic_pix_x,int pic_pix_y,int blocksize_x,int blocksize_y,
                    int cand_mv_x,int cand_mv_y, StorablePicture *ref_picture, pel_t**   orig_pic, 
                    int Mvmcost, int min_mcost,int useABT);

int                                                   //  ==> minimum motion cost after search
FastSubPelBlockMotionSearch (pel_t**   orig_pic,      // <--  original pixel values for the AxB block
                             int       ref,           // <--  reference frame (0... or -1 (backward))
                             int       list,
                             int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                             int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                             int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                             int       pred_mv_x,     // <--  motion vector predictor (x) in sub-pel units
                             int       pred_mv_y,     // <--  motion vector predictor (y) in sub-pel units
                             int*      mv_x,          // <--> in: search center (x) / out: motion vector (x) - in pel units
                             int*      mv_y,          // <--> in: search center (y) / out: motion vector (y) - in pel units
                             int       search_pos2,   // <--  search positions for    half-pel search  (default: 9)
                             int       search_pos4,   // <--  search positions for quarter-pel search  (default: 9)
                             int       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                             double    lambda,
                             int  useABT);        // <--  lagrangian parameter for determining motion cost

int                                               //  ==> minimum motion cost after search
SubPelBlockMotionSearch (pel_t**   orig_pic,      // <--  original pixel values for the AxB block
                         int       ref,           // <--  reference frame (0... or -1 (backward))
                         int       list,
                         int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                         int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                         int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                         int       pred_mv_x,     // <--  motion vector predictor (x) in sub-pel units
                         int       pred_mv_y,     // <--  motion vector predictor (y) in sub-pel units
                         int*      mv_x,          // <--> in: search center (x) / out: motion vector (x) - in pel units
                         int*      mv_y,          // <--> in: search center (y) / out: motion vector (y) - in pel units
                         int       search_pos2,   // <--  search positions for    half-pel search  (default: 9)
                         int       search_pos4,   // <--  search positions for quarter-pel search  (default: 9)
                         int       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                         double    lambda         // <--  lagrangian parameter for determining motion cost
                         );

int                                         //  ==> minimum motion cost after search
FME_BlockMotionSearch (int       ref,           // <--  reference frame (0... or -1 (backward))
                       int       list,
                       int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                       int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                       int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                       int       search_range,  // <--  1-d search range for integer-position search
                       double    lambda         // <--  lagrangian parameter for determining motion cost
                       );

int                                              //!< minimum motion cost after search
noFME_BlockMotionSearch (int       ref,          //!< reference idx
                         int       list,         //!< reference pciture list
                         int       mb_x,         //!< x-coordinate inside macroblock
                         int       mb_y,         //!< y-coordinate inside macroblock
                         int       blocktype,    //!< block type (1-16x16 ... 7-4x4)
                         int       search_range, //!< 1-d search range for integer-position search
                         double    lambda        //!< lagrangian parameter for determining motion cost
                         );
#endif

