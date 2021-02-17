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
 * \file image.c
 *
 * \brief
 *    Code one image/slice
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *     - Inge Lille-Langøy               <inge.lille-langoy@telenor.com>
 *     - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *     - Jani Lainema                    <jani.lainema@nokia.com>
 *     - Sebastian Purreiter             <sebastian.purreiter@mch.siemens.de>
 *     - Byeong-Moon Jeon                <jeonbm@lge.com>
 *     - Yoon-Seong Soh                  <yunsung@lge.com>
 *     - Thomas Stockhammer              <stockhammer@ei.tum.de>
 *     - Detlev Marpe                    <marpe@hhi.de>
 *     - Guido Heising                   <heising@hhi.de>
 *     - Thomas Wedi                     <wedi@tnt.uni-hannover.de>
 *     - Ragip Kurceren                  <ragip.kurceren@nokia.com>
 *     - Antti Hallapuro                 <antti.hallapuro@nokia.com>
 *     - Alexandros Tourapis                     <alexismt@ieee.org> 
 *************************************************************************************
 */
#include "contributors.h"

#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <string.h>
#include <memory.h>
#include <assert.h>

#include "global.h"
#include "image.h"
#include "refbuf.h"
#include "mbuffer.h"
#include "header.h"
#include "intrarefresh.h"
#include "fmo.h"
#include "sei.h"
#include "memalloc.h"
#include "nalu.h"
#include "ratectl.h"
#include "mb_access.h"

void code_a_picture(Picture *pic);
void frame_picture (Picture *frame);
void field_picture(Picture *top, Picture *bottom);

static int  writeout_picture(Picture *pic);

static int  picture_structure_decision(Picture *frame, Picture *top, Picture *bot);
static void distortion_fld (float *dis_fld_y, float *dis_fld_u, float *dis_fld_v);
static void find_snr();
static void find_distortion();

static void field_mode_buffer(int bit_field, float snr_field_y, float snr_field_u, float snr_field_v);
static void frame_mode_buffer (int bit_frame, float snr_frame_y, float snr_frame_u, float snr_frame_v);

static void init_frame();
static void init_field();

static void put_buffer_frame();
static void put_buffer_top();
static void put_buffer_bot();

static void copy_motion_vectors_MB();

static void CopyFrameToOldImgOrgVariables (Sourceframe *sf);
static void CopyTopFieldToOldImgOrgVariables (Sourceframe *sf);
static void CopyBottomFieldToOldImgOrgVariables (Sourceframe *sf);
static Sourceframe *AllocSourceframe (int xs, int ys);
static void FreeSourceframe (Sourceframe *sf);
static void ReadOneFrame (int FrameNoInFile, int HeaderSize, int xs, int ys, Sourceframe *sf);
static void writeUnit(Bitstream* currStream ,int partition);

#ifdef _ADAPT_LAST_GROUP_
int *last_P_no;
int *last_P_no_frm;
int *last_P_no_fld;
#endif

static void ReportFirstframe(int tmp_time, int me_time);
static void ReportIntra(int tmp_time, int me_time);
static void ReportSP(int tmp_time, int me_time);
static void ReportBS(int tmp_time, int me_time);
static void ReportP(int tmp_time, int me_time);
static void ReportB(int tmp_time, int me_time);
static void ReportNALNonVLCBits(int tmp_time, int me_time);

/*
static void ReportFirstframe(int tmp_time);
static void ReportIntra(int tmp_time);
static void ReportSP(int tmp_time);
static void ReportBS(int tmp_time);
static void ReportP(int tmp_time);
static void ReportB(int tmp_time);
*/

static int CalculateFrameNumber();  // Calculates the next frame number
static int FrameNumberInFile;       // The current frame number in the input file
static Sourceframe *srcframe;

StorablePicture *enc_picture;
StorablePicture *enc_frame_picture;
StorablePicture *enc_top_picture;
StorablePicture *enc_bottom_picture;
//Rate control
int    QP;

const int ONE_FOURTH_TAP[3][2] =
{
  {20,20},
  {-5,-4},
  { 1, 0},
};


void MbAffPostProc()
{
  byte temp[16][32];

  byte ** imgY  = enc_picture->imgY;
  byte ***imgUV = enc_picture->imgUV;

  int i, x, y, x0, y0, uv;
  for (i=0; i<(int)img->PicSizeInMbs; i+=2)
  {
    if (enc_picture->mb_field[i])
    {
      get_mb_pos(i, &x0, &y0);
      for (y=0; y<(2*MB_BLOCK_SIZE);y++)
        for (x=0; x<MB_BLOCK_SIZE; x++)
          temp[x][y] = imgY[y0+y][x0+x];

      for (y=0; y<MB_BLOCK_SIZE;y++)
        for (x=0; x<MB_BLOCK_SIZE; x++)
        {
          imgY[y0+(2*y)][x0+x]   = temp[x][y];
          imgY[y0+(2*y+1)][x0+x] = temp[x][y+MB_BLOCK_SIZE];
        }

      x0 = x0/2;
      y0 = y0/2;

      for (uv=0; uv<2; uv++)
      {
        for (y=0; y<(2*MB_BLOCK_SIZE/2);y++)
          for (x=0; x<MB_BLOCK_SIZE/2; x++)
            temp[x][y] = imgUV[uv][y0+y][x0+x];
          
        for (y=0; y<MB_BLOCK_SIZE/2;y++)
          for (x=0; x<MB_BLOCK_SIZE/2; x++)
          {
            imgUV[uv][y0+(2*y)][x0+x]   = temp[x][y];
            imgUV[uv][y0+(2*y+1)][x0+x] = temp[x][y+MB_BLOCK_SIZE/2];
          }
      }
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Encodes a picture
 *
 *    This is the main picture coding loop.. It is called by all this
 *    frame and field coding stuff after the img-> elements have been
 *    set up.  Not sure whether it is useful for MB-adaptive frame/field
 *    coding
 ************************************************************************
 */
void code_a_picture(Picture *pic)
{
  int NumberOfCodedMBs = 0;
  int SliceGroup = 0;
  int j;

  img->currentPicture = pic;

  img->currentPicture->idr_flag = ((!IMG_NUMBER) && (!(img->structure==BOTTOM_FIELD))) || (input->idr_enable && (img->type == I_SLICE || img->type==SP_SLICE || img->type==SI_SLICE)&& (!(img->structure==BOTTOM_FIELD)));

  pic->no_slices = 0;
  pic->distortion_u = pic->distortion_v = pic->distortion_y = 0.0;

  // restrict list 1 size
  img->num_ref_idx_l0_active = max(1, (img->type==B_SLICE ? active_pps->num_ref_idx_l0_active_minus1 + 1: active_pps->num_ref_idx_l0_active_minus1 +1 )); 
  img->num_ref_idx_l1_active = (img->type==B_SLICE ? active_pps->num_ref_idx_l1_active_minus1 + 1 : 0);


  // generate reference picture lists
  init_lists(img->type, img->structure);

  // assign list 0 size from list size
  img->num_ref_idx_l0_active = listXsize[0];
  img->num_ref_idx_l1_active = listXsize[1];

  //if (!img->MbaffFrameFlag)
  {
    if ((img->type == P_SLICE || img->type == SP_SLICE) && input->P_List0_refs)
    {
      img->num_ref_idx_l0_active = min(img->num_ref_idx_l0_active, input->P_List0_refs);
      listXsize[0] = min(listXsize[0], input->P_List0_refs);  
    }
    if (img->type == B_SLICE )
    {
      
      if (input->B_List0_refs)
      {
        img->num_ref_idx_l0_active = min(img->num_ref_idx_l0_active, input->B_List0_refs);
        listXsize[0] = min(listXsize[0], input->B_List0_refs);  
      }
      if (input->B_List1_refs)
      {
        
        img->num_ref_idx_l1_active = min(img->num_ref_idx_l1_active, input->B_List1_refs);
        listXsize[1] = min(listXsize[1], input->B_List1_refs);  
      }
    }
  } 


  //if (img->MbaffFrameFlag)
  if (img->structure==FRAME)
    init_mbaff_lists();

  if (img->type != I_SLICE && (input->WeightedPrediction == 1 || (input->WeightedBiprediction > 0 && (img->type == B_SLICE))))
  {
  	if (img->type==P_SLICE || img->type==SP_SLICE)
       estimate_weighting_factor_P_slice ();
    else
       estimate_weighting_factor_B_slice ();
  }

  
  RandomIntraNewPicture ();     //! Allocates forced INTRA MBs (even for fields!)

  // The slice_group_change_cycle can be changed here.
  // FmoInit() is called before coding each picture, frame or field
  img->slice_group_change_cycle=1;
  FmoInit(img, active_pps, active_sps);
	
  FmoStartPicture ();           //! picture level initialization of FMO

  while (NumberOfCodedMBs < img->total_number_mb)       // loop over slices
  {
    // Encode one SLice Group
    while (!FmoSliceGroupCompletelyCoded (SliceGroup))
    {
      // Encode the current slice
      NumberOfCodedMBs += encode_one_slice (SliceGroup, pic);
      FmoSetLastMacroblockInSlice (img->current_mb_nr);
      // Proceed to next slice
      img->current_slice_nr++;
      stat->bit_slice = 0;
    }
    // Proceed to next SliceGroup
    SliceGroup++;
  }
  FmoEndPicture ();

  if (input->rdopt == 2 && (img->type != B_SLICE))
    for (j = 0; j < input->NoOfDecoders; j++)
      DeblockFrame (img, decs->decY_best[j], NULL);

  DeblockFrame (img, enc_picture->imgY, enc_picture->imgUV);

  if (img->MbaffFrameFlag)
    MbAffPostProc();

}



/*!
 ************************************************************************
 * \brief
 *    Encodes one frame
 ************************************************************************
 */
int encode_one_frame ()
{
  static int prev_frame_no = 0; // POC200301
  static int consecutive_non_reference_pictures = 0; // POC200301

#ifdef _LEAKYBUCKET_
  extern long Bit_Buffer[10000];
  extern unsigned long total_frame_buffer;
#endif

  time_t ltime1;
  time_t ltime2;

#ifdef WIN32
  struct _timeb tstruct1;
  struct _timeb tstruct2;
#else
  struct timeb tstruct1;
  struct timeb tstruct2;
#endif

  int tmp_time;
  int bits_frm = 0, bits_fld = 0;
  float dis_frm = 0, dis_frm_y = 0, dis_frm_u = 0, dis_frm_v = 0;
  float dis_fld = 0, dis_fld_y = 0, dis_fld_u = 0, dis_fld_v = 0;

  //Rate control
  int pic_type, bits = 0; 

  me_time=0;
  
#ifdef WIN32
  _ftime (&tstruct1);           // start time ms
#else
  ftime (&tstruct1);
#endif
  time (&ltime1);               // start time s

  //Rate control 
  img->write_macroblock = 0;
/*
  //Shankar Regunathan (Oct 2002)
  //Prepare Panscanrect SEI payload
  UpdatePanScanRectInfo ();
  //Prepare Arbitrarydata SEI Payload
  UpdateUser_data_unregistered ();
  //Prepare Registered data SEI Payload
  UpdateUser_data_registered_itu_t_t35 ();
  //Prepare RandomAccess SEI Payload
  UpdateRandomAccess ();
*/

  put_buffer_frame ();      // sets the pointers to the frame structures 
                            // (and not to one of the field structures)
  init_frame ();
  FrameNumberInFile = CalculateFrameNumber();

  srcframe = AllocSourceframe (img->width, img->height);
  ReadOneFrame (FrameNumberInFile, input->infile_header, img->width, img->height, srcframe);
  CopyFrameToOldImgOrgVariables (srcframe);

  // Set parameters for directmode and Deblocking filter
  img->direct_type     = input->direct_type;
  img->LFDisableIdc    = input->LFDisableIdc;
  img->LFAlphaC0Offset = input->LFAlphaC0Offset;
  img->LFBetaOffset    = input->LFBetaOffset;

  if (img->type == B_SLICE)
    Bframe_ctr++;         // Bframe_ctr only used for statistics, should go to stat->

  if (input->PicInterlace == FIELD_CODING)
  {
    //Rate control
    img->FieldControl=1;

    img->field_picture = 1;  // we encode fields
    field_picture (top_pic, bottom_pic);
    img->fld_flag = 1;
  }
  else
  {
    //Rate control
    img->FieldControl=0;

    // For frame coding, turn MB level field/frame coding flag on
    if (input->MbInterlace)
      mb_adaptive = 1;

    img->field_picture = 0; // we encode a frame

    //Rate control
    if(input->RCEnable)
    { 
    /*update the number of MBs in the basic unit for MB adaptive 
      f/f coding*/
      if((input->MbInterlace)&&(input->basicunit<img->Frame_Total_Number_MB)\
        &&(img->type==P_SLICE)&&(img->IFLAG==0))
        img->BasicUnit=input->basicunit*2;
      else
        img->BasicUnit=input->basicunit;
      
      rc_init_pict(1,0,1); 
      img->qp  = updateQuantizationParameter(0); 
      
      
      pic_type = img->type;
      QP =0;
    }

    if( active_sps->frame_mbs_only_flag)
      img->TopFieldFlag=0;

    frame_picture (frame_pic);
   
    // For field coding, turn MB level field/frame coding flag off
    if (input->MbInterlace)
      mb_adaptive = 0;
    
    if (input->PicInterlace == ADAPTIVE_CODING)
    {
      //Rate control
      img->FieldControl=1;
      img->write_macroblock = 0;
      img->bot_MB = 0;

      img->field_picture = 1;  // we encode fields
      field_picture (top_pic, bottom_pic);
      
      //! Note: the distortion for a field coded picture is stored in the top field
      //! the distortion values in the bottom field are dummies
      dis_fld = top_pic->distortion_y + top_pic->distortion_u + top_pic->distortion_v;
      dis_frm = frame_pic->distortion_y + frame_pic->distortion_u + frame_pic->distortion_v;
      
      img->fld_flag = picture_structure_decision (frame_pic, top_pic, bottom_pic);
      update_field_frame_contexts (img->fld_flag);

      //Rate control
      if(img->fld_flag==0)
        img->FieldFrame=1;
      /*the current choice is field coding*/
      else
        img->FieldFrame=0;
    }
    else
   
      img->fld_flag = 0;
  }

  if (img->fld_flag)
    stat->bit_ctr_emulationprevention += stat->em_prev_bits_fld;
  else
    stat->bit_ctr_emulationprevention += stat->em_prev_bits_frm;

  if (img->type != B_SLICE)
  {
    img->pstruct_next_P = img->fld_flag;
  }

  // Here, img->structure may be either FRAME or BOTTOM FIELD depending on whether AFF coding is used
  // The picture structure decision changes really only the fld_flag

  if (img->fld_flag)            // field mode (use field when fld_flag=1 only)
  {
    field_mode_buffer (bits_fld, dis_fld_y, dis_fld_u, dis_fld_v);
    writeout_picture (top_pic);
    writeout_picture (bottom_pic);
  }
  else                          //frame mode
  {
    frame_mode_buffer (bits_frm, dis_frm_y, dis_frm_u, dis_frm_v);
    writeout_picture (frame_pic);
  }

  if (frame_pic)
    free_slice_list(frame_pic);
  if (top_pic)
    free_slice_list(top_pic);
  if (bottom_pic)
    free_slice_list(bottom_pic);

  /*
  // Tian Dong (Sept 2002)
  // in frame mode, the newly reconstructed frame has been inserted to the mem buffer
  // and it is time to prepare the spare picture SEI payload.
  if (input->InterlaceCodingOption == FRAME_CODING
      && input->SparePictureOption && img->type != B_SLICE)
    CalculateSparePicture ();
*/

  //Rate control
  if(input->RCEnable)
  {
    bits = stat->bit_ctr-stat->bit_ctr_n;
    rc_update_pict_frame(bits);
  }

/*
    
  if (input->InterlaceCodingOption == FRAME_CODING)
  {
    if (input->rdopt == 2 && img->type != B_SLICE)
      UpdateDecoders ();      // simulate packet losses and move decoded image to reference buffers
    
    if (input->RestrictRef)
      UpdatePixelMap ();
  }
*/

  find_snr ();

  time (&ltime2);               // end time sec
#ifdef WIN32
  _ftime (&tstruct2);           // end time ms
#else
  ftime (&tstruct2);            // end time ms
#endif

  tmp_time = (ltime2 * 1000 + tstruct2.millitm) - (ltime1 * 1000 + tstruct1.millitm);
  tot_time = tot_time + tmp_time;

  if (input->PicInterlace == ADAPTIVE_CODING)
  {
    if (img->fld_flag)
    {
      // store bottom field
      store_picture_in_dpb(enc_bottom_picture);
      free_storable_picture(enc_frame_picture);
    }
    else
    {
      // replace top with frame
      replace_top_pic_with_frame(enc_frame_picture);
      free_storable_picture(enc_bottom_picture);
    }
  }
  else
  {
    if (img->fld_flag)
    {
      store_picture_in_dpb(enc_bottom_picture);
    }
    else
    {
      store_picture_in_dpb(enc_frame_picture);
    }
  }


#ifdef _LEAKYBUCKET_
  // Store bits used for this frame and increment counter of no. of coded frames
  Bit_Buffer[total_frame_buffer] = stat->bit_ctr - stat->bit_ctr_n;
  total_frame_buffer++;
#endif

  // POC200301: Verify that POC coding type 2 is not used if more than one consecutive 
  // non-reference frame is requested or if decoding order is different from output order
  if (img->pic_order_cnt_type == 2)
  {
    if (!img->nal_reference_idc) consecutive_non_reference_pictures++;
    else consecutive_non_reference_pictures = 0;

    if (frame_no < prev_frame_no || consecutive_non_reference_pictures>1)
      error("POC type 2 cannot be applied for the coding pattern where the encoding /decoding order of pictures are different from the output order.\n", -1);
    prev_frame_no = frame_no;
  }

  if (stat->bit_ctr_parametersets_n!=0)
    ReportNALNonVLCBits(tmp_time, me_time);

  if (IMG_NUMBER == 0)
    ReportFirstframe(tmp_time,me_time);
    //ReportFirstframe(tmp_time);
  else
  {
    //Rate control
    if(input->RCEnable)
    {
      if((!input->PicInterlace)&&(!input->MbInterlace))
        bits=stat->bit_ctr-stat->bit_ctr_n;
      else
      {
        bits = stat->bit_ctr -Pprev_bits; // used for rate control update */
        Pprev_bits = stat->bit_ctr;
      }
    }

    switch (img->type)
    {
    case I_SLICE:
      stat->bit_ctr_P += stat->bit_ctr - stat->bit_ctr_n;
	  ReportIntra(tmp_time,me_time);
      //ReportIntra(tmp_time);
      break;
    case SP_SLICE:
      stat->bit_ctr_P += stat->bit_ctr - stat->bit_ctr_n;
      ReportSP(tmp_time,me_time);
      //ReportSP(tmp_time);
      break;
    case B_SLICE:
      stat->bit_ctr_B += stat->bit_ctr - stat->bit_ctr_n;
      if (img->nal_reference_idc>0)
        ReportBS(tmp_time,me_time);
        //ReportBS(tmp_time);
      else
        ReportB(tmp_time,me_time);
        //ReportB(tmp_time);

      break;
    default:      // P
      stat->bit_ctr_P += stat->bit_ctr - stat->bit_ctr_n;
      ReportP(tmp_time,me_time);
      //ReportP(tmp_time);
    }
  }
  stat->bit_ctr_n = stat->bit_ctr;

  //Rate control
  if(input->RCEnable) 
  {
    rc_update_pict(bits);
      /*update the parameters of quadratic R-D model*/
    if((img->type==P_SLICE)&&(active_sps->frame_mbs_only_flag))
      updateRCModel();
    else if((img->type==P_SLICE)&&(!active_sps->frame_mbs_only_flag)\
      &&(img->IFLAG==0))
      updateRCModel();
  }

  stat->bit_ctr_parametersets_n=0;

  FreeSourceframe (srcframe);

  if (IMG_NUMBER == 0)
    return 0;
  else
    return 1;
}


/*!
 ************************************************************************
 * \brief
 *    This function write out a picture
 * \return
 *    0 if OK,                                                         \n
 *    1 in case of error
 *
 ************************************************************************
 */
static int writeout_picture(Picture *pic)
{
  Bitstream *currStream;
  int partition, slice;
  Slice *currSlice;

  img->currentPicture=pic;

  for (slice=0; slice<pic->no_slices; slice++)
  {
    currSlice = pic->slices[slice];
    for (partition=0; partition<currSlice->max_part_nr; partition++)
    {
      currStream = (currSlice->partArr[partition]).bitstream;
      assert (currStream->bits_to_go == 8);    //! should always be the case, the 
                                               //! byte alignment is done in terminate_slice
			writeUnit (currSlice->partArr[partition].bitstream,partition);

    }           // partition loop
  }           // slice loop
  return 0;   
}


/*!
 ************************************************************************
 * \brief
 *    Encodes a frame picture
 ************************************************************************
 */
void frame_picture (Picture *frame)
{

  img->structure = FRAME;
  img->PicSizeInMbs = img->FrameSizeInMbs;

  enc_frame_picture  = alloc_storable_picture (img->structure, img->width, img->height, img->width_cr, img->height_cr);
  img->ThisPOC=enc_frame_picture->poc=img->framepoc;
  enc_frame_picture->top_poc    = img->toppoc;
  enc_frame_picture->bottom_poc = img->bottompoc;

  enc_frame_picture->frame_poc = img->framepoc;

  enc_frame_picture->pic_num = img->frame_num;
  enc_frame_picture->coded_frame = 1;

  enc_frame_picture->MbaffFrameFlag = img->MbaffFrameFlag = (input->MbInterlace != FRAME_CODING);

  enc_picture=enc_frame_picture;

  stat->em_prev_bits_frm = 0;
  stat->em_prev_bits = &stat->em_prev_bits_frm;

  if (img->MbaffFrameFlag)
  {
    CopyTopFieldToOldImgOrgVariables (srcframe);
    CopyBottomFieldToOldImgOrgVariables (srcframe);
  }

  img->fld_flag = 0;
  code_a_picture(frame);

  frame->bits_per_picture = 8 * ((((img->currentSlice)->partArr[0]).bitstream)->byte_pos);
  
  if (img->structure==FRAME)
  {
    find_distortion (snr, img);      
    frame->distortion_y = snr->snr_y;
    frame->distortion_u = snr->snr_u;
    frame->distortion_v = snr->snr_v;
  }
}


/*!
 ************************************************************************
 * \brief
 *    Encodes a field picture, consisting of top and bottom field
 ************************************************************************
 */
void field_picture (Picture *top, Picture *bottom)
{
  //Rate control
  int old_pic_type;              // picture type of top field used for rate control    
  int TopFieldBits;
  
  //Rate control
  old_pic_type = img->type;

  stat->em_prev_bits_fld = 0;
  stat->em_prev_bits = &stat->em_prev_bits_fld;
  img->number *= 2;
  img->buf_cycle *= 2;
  img->height = input->img_height / 2;
  img->height_cr = input->img_height / 4;
  img->fld_flag = 1;
  img->PicSizeInMbs = img->FrameSizeInMbs/2;
  // Top field
  
//  img->bottom_field_flag = 0;
  enc_top_picture  = alloc_storable_picture (img->structure, img->width, img->height, img->width_cr, img->height_cr);
  enc_top_picture->poc=img->toppoc;
  enc_top_picture->frame_poc = img->toppoc;
  enc_top_picture->pic_num = img->frame_num;
  enc_top_picture->coded_frame = 0;
  enc_top_picture->MbaffFrameFlag = img->MbaffFrameFlag = FALSE;
  img->ThisPOC = img->toppoc;
  
  img->structure = TOP_FIELD;
  enc_picture = enc_top_picture;
  put_buffer_top ();
  init_field ();
  if (img->type == B_SLICE)       //all I- and P-frames
    nextP_tr_fld--;

  CopyTopFieldToOldImgOrgVariables (srcframe);
  
  img->fld_flag = 1;
//  img->bottom_field_flag = 0;
 
  //Rate control
  if(input->RCEnable)
  {
    img->BasicUnit=input->basicunit;

    if(input->PicInterlace==FIELD_CODING)
      rc_init_pict(0,1,1); 
    else
      rc_init_pict(0,1,0);

    img->qp  = updateQuantizationParameter(1); 
   }
  img->TopFieldFlag=1;

  code_a_picture(top_pic);
  enc_picture->structure = 1;
    
  store_picture_in_dpb(enc_top_picture);

  top->bits_per_picture = 8 * ((((img->currentSlice)->partArr[0]).bitstream)->byte_pos);

  //Rate control
  TopFieldBits=top->bits_per_picture;

  //  Bottom field
//  img->bottom_field_flag = 0;
  enc_bottom_picture  = alloc_storable_picture (img->structure, img->width, img->height, img->width_cr, img->height_cr);
  enc_bottom_picture->poc=img->bottompoc;
  enc_bottom_picture->frame_poc = img->bottompoc;
  enc_bottom_picture->pic_num = img->frame_num;
  enc_bottom_picture->coded_frame = 0;
  enc_bottom_picture->MbaffFrameFlag = img->MbaffFrameFlag = FALSE;
  img->ThisPOC = img->bottompoc;
  img->structure = BOTTOM_FIELD;
  enc_picture = enc_bottom_picture;
  put_buffer_bot ();
  img->number++;

  init_field ();

  if (img->type == B_SLICE)       //all I- and P-frames
    nextP_tr_fld++;             //check once coding B field

 if (img->type == I_SLICE && input->IntraBottom!=1)
    img->type = P_SLICE;

  CopyBottomFieldToOldImgOrgVariables (srcframe);
  img->fld_flag = 1;
//  img->bottom_field_flag = 1;

  //Rate control
  if(input->RCEnable)  setbitscount(TopFieldBits);
  if(input->RCEnable)
  {
    rc_init_pict(0,0,0); 
    img->qp  = updateQuantizationParameter(0); 
  }
  img->TopFieldFlag=0;

  enc_picture->structure = 2;
  code_a_picture(bottom_pic);

  bottom->bits_per_picture = 8 * ((((img->currentSlice)->partArr[0]).bitstream)->byte_pos);

  // the distortion for a field coded frame (consisting of top and bottom field)
  // lives in the top->distortion varaibles, thye bottom-> are dummies
  distortion_fld (&top->distortion_y, &top->distortion_u, &top->distortion_v);

}


/*!
 ************************************************************************
 * \brief
 *    Distortion Field
 ************************************************************************
 */
static void distortion_fld (float *dis_fld_y, float *dis_fld_u, float *dis_fld_v)
{

  img->number /= 2;
  img->buf_cycle /= 2;
  img->height = input->img_height;
  img->height_cr = input->img_height / 2;
  img->total_number_mb =
    (img->width * img->height) / (MB_BLOCK_SIZE * MB_BLOCK_SIZE);

  combine_field ();

  imgY_org = imgY_org_frm;
  imgUV_org = imgUV_org_frm;

  find_distortion (snr, img);   // find snr from original frame picture

  *dis_fld_y = snr->snr_y;
  *dis_fld_u = snr->snr_u;
  *dis_fld_v = snr->snr_v;
}


/*!
 ************************************************************************
 * \brief
 *    Picture Structure Decision
 ************************************************************************
 */
static int picture_structure_decision (Picture *frame, Picture *top, Picture *bot)
{
  double lambda_picture;
  int spframe = (img->type == SP_SLICE);
  int bframe = (img->type == B_SLICE);
  float snr_frame, snr_field;
  int bit_frame, bit_field;

  lambda_picture = 0.85 * pow (2, (img->qp - SHIFT_QP) / 3.0) * (bframe || spframe ? 4 : 1);

  snr_frame = frame->distortion_y + frame->distortion_u + frame->distortion_v;
  //! all distrortions of a field picture are accumulated in the top field
  snr_field = top->distortion_y + top->distortion_u + top->distortion_v;
  bit_field = top->bits_per_picture + bot->bits_per_picture;
  bit_frame = frame->bits_per_picture;

  return decide_fld_frame (snr_frame, snr_field, bit_field, bit_frame, lambda_picture);
}


/*!
 ************************************************************************
 * \brief
 *    Field Mode Buffer
 ************************************************************************
 */
static void field_mode_buffer (int bit_field, float snr_field_y, float snr_field_u, float snr_field_v)
{
  put_buffer_frame ();

  snr->snr_y = snr_field_y;
  snr->snr_u = snr_field_u;
  snr->snr_v = snr_field_v;
}


/*!
 ************************************************************************
 * \brief
 *    Frame Mode Buffer
 ************************************************************************
 */
static void frame_mode_buffer (int bit_frame, float snr_frame_y, float snr_frame_u, float snr_frame_v)
{
  put_buffer_frame ();

  if ((input->PicInterlace != FRAME_CODING)||(input->MbInterlace != FRAME_CODING))
  {
    img->height = img->height / 2;
    img->height_cr = img->height_cr / 2;
    img->number *= 2;
    
    put_buffer_top ();
     
    img->number++;
    put_buffer_bot ();
    
    img->number /= 2;         // reset the img->number to field
    img->height = input->img_height;
    img->height_cr = input->img_height / 2;
    img->total_number_mb =
      (img->width * img->height) / (MB_BLOCK_SIZE * MB_BLOCK_SIZE);
    
    snr->snr_y = snr_frame_y;
    snr->snr_u = snr_frame_u;
    snr->snr_v = snr_frame_v;
    put_buffer_frame ();
    
  }
}


/*!
 ************************************************************************
 * \brief
 *    mmco initializations should go here
 ************************************************************************
 */
static void init_dec_ref_pic_marking_buffer()
{
  img->dec_ref_pic_marking_buffer=NULL;
}


/*!
 ************************************************************************
 * \brief
 *    Initializes the parameters for a new frame
 ************************************************************************
 */
static void init_frame ()
{
  int i;
  int prevP_no, nextP_no;
  
  last_P_no = last_P_no_frm;

  img->current_mb_nr = 0;
  img->current_slice_nr = 0;
  stat->bit_slice = 0;

  img->mb_y = img->mb_x = 0;
  img->block_y = img->pix_y = img->pix_c_y = 0; 
  img->block_x = img->pix_x = img->block_c_x = img->pix_c_x = 0;

  // The 'slice_nr' of each macroblock is set to -1 here, to guarantee the correct encoding 
  // with FMO (if no FMO, encoding is correct without following assignment), 
  // for which MBs may not be encoded with scan order
  for(i=0;i< ((img->width/MB_BLOCK_SIZE)*(img->height/MB_BLOCK_SIZE));i++)
    img->mb_data[i].slice_nr=-1;
	
  if (img->type != B_SLICE)
  {
    img->tr = start_tr_in_this_IGOP + IMG_NUMBER * (input->jumpd + 1);
    
    img->imgtr_last_P_frm = img->imgtr_next_P_frm;
    img->imgtr_next_P_frm = img->tr;
    
#ifdef _ADAPT_LAST_GROUP_
    if (input->last_frame && img->number + 1 == input->no_frames)
      img->tr = input->last_frame;
#endif
    
    if (IMG_NUMBER != 0 && input->successive_Bframe != 0)     // B pictures to encode
      nextP_tr_frm = img->tr;
    
    //Rate control
    if(!input->RCEnable)                  // without using rate control
    {
      if (img->type == I_SLICE)
#ifdef _CHANGE_QP_
        if (input->qp2start > 0 && img->tr >= input->qp2start)
          img->qp = input->qp02;
        else
#endif    
        img->qp = input->qp0;   // set quant. parameter for I-frame
      else
      {
#ifdef _CHANGE_QP_
        if (input->qp2start > 0 && img->tr >= input->qp2start)
          img->qp = input->qpN2;
        else
#endif
          img->qp = input->qpN;
        
        if (img->type == SP_SLICE)
        {
          img->qp = input->qpsp;
          img->qpsp = input->qpsp_pred;
        }   
      }
    }

    img->mb_y_intra = img->mb_y_upd;  //  img->mb_y_intra indicates which GOB to intra code for this frame
    
    if (input->intra_upd > 0) // if error robustness, find next GOB to update
    {
      img->mb_y_upd = (IMG_NUMBER / input->intra_upd) % (img->height / MB_BLOCK_SIZE);
    }
  }
  else
  {
    img->p_interval = input->jumpd + 1;
    prevP_no = start_tr_in_this_IGOP + (IMG_NUMBER - 1) * img->p_interval;
    nextP_no = start_tr_in_this_IGOP + (IMG_NUMBER) * img->p_interval;
    
#ifdef _ADAPT_LAST_GROUP_
    last_P_no[0] = prevP_no;
    for (i = 1; i < img->buf_cycle; i++)
      last_P_no[i] = last_P_no[i - 1] - img->p_interval;
    
    if (input->last_frame && img->number + 1 == input->no_frames)
    {
      nextP_no = input->last_frame;
      img->p_interval = nextP_no - prevP_no;
    }
#endif
    
    img->b_interval =
      (int) ((float) (input->jumpd + 1) / (input->successive_Bframe + 1.0) +
      0.49999);
    
    img->tr = prevP_no + img->b_interval * img->b_frame_to_code;      // from prev_P

    if (img->tr >= nextP_no)
      img->tr = nextP_no - 1;
    //Rate control
    if(!input->RCEnable)                  // without using rate control
    {    
#ifdef _CHANGE_QP_
    if (input->qp2start > 0 && img->tr >= input->qp2start)
      img->qp = input->qpB2;
    else
#endif
      img->qp = input->qpB;
    }

  }
  
  UpdateSubseqInfo (img->layer);        // Tian Dong (Sept 2002)
  UpdateSceneInformation (0, 0, 0, -1); // JVT-D099, scene information SEI, nothing included by default

  //! Commented out by StW, needs fixing in SEI.h to keep the trace file clean
  //  PrepareAggregationSEIMessage ();

  img->total_number_mb = (img->width * img->height) / (MB_BLOCK_SIZE * MB_BLOCK_SIZE);

  img->no_output_of_prior_pics_flag = 0;
  img->long_term_reference_flag = 0;

  init_dec_ref_pic_marking_buffer();
}

/*!
 ************************************************************************
 * \brief
 *    Initializes the parameters for a new field
 ************************************************************************
 */
static void init_field ()
{
  int i;
  int prevP_no, nextP_no;

  last_P_no = last_P_no_fld;

  img->current_mb_nr = 0;
  img->current_slice_nr = 0;
  stat->bit_slice = 0;

  input->jumpd *= 2;
  input->successive_Bframe *= 2;
  img->number /= 2;
  img->buf_cycle /= 2;

  img->mb_y = img->mb_x = 0;
  img->block_y = img->pix_y = img->pix_c_y = 0; // define vertical positions
  img->block_x = img->pix_x = img->block_c_x = img->pix_c_x = 0;        // define horizontal positions

  if (img->type != B_SLICE)
    {
      img->tr = img->number * (input->jumpd + 2) + img->fld_type;

      if (!img->fld_type)
        {
          img->imgtr_last_P_fld = img->imgtr_next_P_fld;
          img->imgtr_next_P_fld = img->tr;
        }

#ifdef _ADAPT_LAST_GROUP_
      if (input->last_frame && img->number + 1 == input->no_frames)
        img->tr = input->last_frame;
#endif
      if (img->number != 0 && input->successive_Bframe != 0)    // B pictures to encode
        nextP_tr_fld = img->tr;
      
      //Rate control
      if(!input->RCEnable)                  // without using rate control
      {
        if (img->type == I_SLICE)
#ifdef _CHANGE_QP_
        if (input->qp2start > 0 && img->tr >= input->qp2start)
          img->qp = input->qp02;
        else
#endif    
          img->qp = input->qp0;   // set quant. parameter for I-frame
        else
        {
#ifdef _CHANGE_QP_
          if (input->qp2start > 0 && img->tr >= input->qp2start)
            img->qp = input->qpN2;
          else
#endif
            img->qp = input->qpN;
          if (img->type == SP_SLICE)
          {
            img->qp = input->qpsp;
            img->qpsp = input->qpsp_pred;
          }
        }
      }

      img->mb_y_intra = img->mb_y_upd;  //  img->mb_y_intra indicates which GOB to intra code for this frame

      if (input->intra_upd > 0) // if error robustness, find next GOB to update
        {
          img->mb_y_upd =
            (img->number / input->intra_upd) % (img->width / MB_BLOCK_SIZE);
        }
    }
  else
    {
      img->p_interval = input->jumpd + 2;
      prevP_no = (img->number - 1) * img->p_interval + img->fld_type;
      nextP_no = img->number * img->p_interval + img->fld_type;
#ifdef _ADAPT_LAST_GROUP_
      if (!img->fld_type)       // top field
        {
          last_P_no[0] = prevP_no + 1;
          last_P_no[1] = prevP_no;
          for (i = 1; i <= img->buf_cycle; i++)
            {
              last_P_no[2 * i] = last_P_no[2 * i - 2] - img->p_interval;
              last_P_no[2 * i + 1] = last_P_no[2 * i - 1] - img->p_interval;
            }
        }
      else                      // bottom field
        {
          last_P_no[0] = nextP_no - 1;
          last_P_no[1] = prevP_no;
          for (i = 1; i <= img->buf_cycle; i++)
            {
              last_P_no[2 * i] = last_P_no[2 * i - 2] - img->p_interval;
              last_P_no[2 * i + 1] = last_P_no[2 * i - 1] - img->p_interval;
            }
        }

      if (input->last_frame && img->number + 1 == input->no_frames)
        {
          nextP_no = input->last_frame;
          img->p_interval = nextP_no - prevP_no;
        }
#endif

      img->b_interval =
      (int) ((float) (input->jumpd + 1) / (input->successive_Bframe + 1.0) + 0.49999);

      img->tr = prevP_no + (img->b_interval + 1) * img->b_frame_to_code;        // from prev_P
      if (img->tr >= nextP_no)
        img->tr = nextP_no - 1; // ?????
      //Rate control
      if(!input->RCEnable)                  // without using rate control
      {
#ifdef _CHANGE_QP_
      if (input->qp2start > 0 && img->tr >= input->qp2start)
        img->qp = input->qpB2;
      else
#endif
        img->qp = input->qpB;
      }
    }
  input->jumpd /= 2;
  input->successive_Bframe /= 2;
  img->buf_cycle *= 2;
  img->number = 2 * img->number + img->fld_type;
  img->total_number_mb = (img->width * img->height) / (MB_BLOCK_SIZE * MB_BLOCK_SIZE);
}


#define Clip(min,max,val) (((val)<(min))?(min):(((val)>(max))?(max):(val)))




/*!
 ************************************************************************
 * \brief
 *    Generate Full Pel Representation
 ************************************************************************
 */
static void GenerateFullPelRepresentation (pel_t ** Fourthpel,
                                           pel_t * Fullpel, int xsize,
                                           int ysize)
{
  int x, y;
  
  for (y = 0; y < ysize; y++)
    for (x = 0; x < xsize; x++)
      PutPel_11 (Fullpel, y, x, FastPelY_14 (Fourthpel, y * 4, x * 4, ysize, xsize), xsize);
}


/*!
 ************************************************************************
 * \brief
 *    Upsample 4 times, store them in out4x.  Color is simply copied
 *
 * \par Input:
 *    srcy, srcu, srcv, out4y, out4u, out4v
 *
 * \par Side Effects_
 *    Uses (writes) img4Y_tmp.  This should be moved to a static variable
 *    in this module
 ************************************************************************/
void UnifiedOneForthPix (StorablePicture *s)
{
  int is;
  int i, j, j4;
  int ie2, je2, jj, maxy;
  
  byte **out4Y;
  byte  *ref11;
  byte  **imgY = s->imgY;

  int img_width =s->size_x;
  int img_height=s->size_y;
  
  // don't upsample twice
  if (s->imgY_ups || s->imgY_11)
    return;

  s->imgY_11 = malloc ((s->size_x * s->size_y) * sizeof (byte));
  if (NULL == s->imgY_11)
    no_mem_exit("alloc_storable_picture: s->imgY_11");
  
  get_mem2D (&(s->imgY_ups), (2*IMG_PAD_SIZE + s->size_y)*4, (2*IMG_PAD_SIZE + s->size_x)*4);

  if (input->WeightedPrediction || input->WeightedBiprediction)
  {
      s->imgY_11_w = malloc ((s->size_x * s->size_y) * sizeof (byte));
      get_mem2D (&(s->imgY_ups_w), (2*IMG_PAD_SIZE + s->size_y)*4, (2*IMG_PAD_SIZE + s->size_x)*4);
  }
  out4Y = s->imgY_ups;
  ref11 = s->imgY_11;

  for (j = -IMG_PAD_SIZE; j < s->size_y + IMG_PAD_SIZE; j++)
  {
    for (i = -IMG_PAD_SIZE; i < s->size_x + IMG_PAD_SIZE; i++)
    {
      jj = max (0, min (s->size_y - 1, j));
      is =
              (ONE_FOURTH_TAP[0][0] *
               (imgY[jj][max (0, min (s->size_x - 1, i))] +
                imgY[jj][max (0, min (s->size_x - 1, i + 1))]) +
               ONE_FOURTH_TAP[1][0] *
               (imgY[jj][max (0, min (s->size_x - 1, i - 1))] +
                imgY[jj][max (0, min (s->size_x - 1, i + 2))]) +
               ONE_FOURTH_TAP[2][0] *
               (imgY[jj][max (0, min (s->size_x - 1, i - 2))] +
                imgY[jj][max (0, min (s->size_x - 1, i + 3))]));
            img4Y_tmp[j + IMG_PAD_SIZE][(i + IMG_PAD_SIZE) * 2] = imgY[jj][max (0, min (s->size_x - 1, i))] * 1024;    // 1/1 pix pos
            img4Y_tmp[j + IMG_PAD_SIZE][(i + IMG_PAD_SIZE) * 2 + 1] = is * 32;  // 1/2 pix pos
    }
  }
  
  for (i = 0; i < (s->size_x + 2 * IMG_PAD_SIZE) * 2; i++)
  {
    for (j = 0; j < s->size_y + 2 * IMG_PAD_SIZE; j++)
    {
      j4 = j * 4;
      maxy = s->size_y + 2 * IMG_PAD_SIZE - 1;
      // change for TML4, use 6 TAP vertical filter
      is =
              (ONE_FOURTH_TAP[0][0] *
               (img4Y_tmp[j][i] + img4Y_tmp[min (maxy, j + 1)][i]) +
               ONE_FOURTH_TAP[1][0] * (img4Y_tmp[max (0, j - 1)][i] +
                                       img4Y_tmp[min (maxy, j + 2)][i]) +
               ONE_FOURTH_TAP[2][0] * (img4Y_tmp[max (0, j - 2)][i] +
                                       img4Y_tmp[min (maxy, j + 3)][i])) / 32;
      
      PutPel_14 (out4Y, (j - IMG_PAD_SIZE) * 4, (i - IMG_PAD_SIZE * 2) * 2, (pel_t) max (0, min (255, (int) ((img4Y_tmp[j][i] + 512) / 1024))));  // 1/2 pix
      PutPel_14 (out4Y, (j - IMG_PAD_SIZE) * 4 + 2, (i - IMG_PAD_SIZE * 2) * 2, (pel_t) max (0, min (255, (int) ((is + 512) / 1024))));   // 1/2 pix
    }
  }
  
  /* 1/4 pix */
  /* luma */
  ie2 = (s->size_x + 2 * IMG_PAD_SIZE - 1) * 4;
  je2 = (s->size_y + 2 * IMG_PAD_SIZE - 1) * 4;
  
  for (j = 0; j < je2 + 4; j += 2)
    for (i = 0; i < ie2 + 3; i += 2)
    {
      /*  '-'  */
          PutPel_14 (out4Y, j - IMG_PAD_SIZE * 4, i - IMG_PAD_SIZE * 4 + 1,
                     (pel_t) (max (0, min (255,(int) (FastPelY_14 (out4Y, j - IMG_PAD_SIZE * 4,
                      i - IMG_PAD_SIZE * 4, img_height, img_width) + FastPelY_14 (out4Y,
                      j - IMG_PAD_SIZE * 4, min (ie2 + 2, i + 2) - IMG_PAD_SIZE * 4, img_height, img_width)+1) / 2))));
    }
    for (i = 0; i < ie2 + 4; i++)
    {
      for (j = 0; j < je2 + 3; j += 2)
      {
        if (i % 2 == 0)
        {
          /*  '|'  */
          PutPel_14 (out4Y, j - IMG_PAD_SIZE * 4 + 1, i - IMG_PAD_SIZE * 4,
                    (pel_t) (max (0, min (255, (int) (FastPelY_14 (out4Y, j - IMG_PAD_SIZE * 4,
                     i - IMG_PAD_SIZE * 4, img_height, img_width) + FastPelY_14 (out4Y,
                     min (je2 + 2, j + 2) - IMG_PAD_SIZE * 4, i - IMG_PAD_SIZE * 4, img_height, img_width)+1) / 2))));
        }
        else if ((j % 4 == 0 && i % 4 == 1) || (j % 4 == 2 && i % 4 == 3))
        {
                /*  '/'  */
                PutPel_14 (out4Y, j - IMG_PAD_SIZE * 4 + 1, i - IMG_PAD_SIZE * 4,
                           (pel_t) (max (0, min (255, (int) (FastPelY_14 (out4Y, j - IMG_PAD_SIZE * 4,
                            min (ie2 + 2, i + 1) - IMG_PAD_SIZE * 4, img_height, img_width) + FastPelY_14 (out4Y,
                            min (je2 + 2, j + 2) - IMG_PAD_SIZE * 4, i - IMG_PAD_SIZE * 4 - 1, img_height, img_width) + 1) / 2))));
        }
        else
        {
                /*  '\'  */
                PutPel_14 (out4Y, j - IMG_PAD_SIZE * 4 + 1, i - IMG_PAD_SIZE * 4,
                           (pel_t) (max (0, min (255, (int) (FastPelY_14 (out4Y, j - IMG_PAD_SIZE * 4,
                           i - IMG_PAD_SIZE * 4 - 1, img_height, img_width) + FastPelY_14 (out4Y,
                           min (je2 + 2, j + 2) - IMG_PAD_SIZE * 4, 
                           min (ie2 + 2, i + 1) - IMG_PAD_SIZE * 4, img_height, img_width) + 1) / 2))));
              }
          }
      }

    /*  Chroma: */
/*    for (j = 0; j < img->height_cr; j++)
      {
        memcpy (outU[j], imgU[j], img->width_cr);       // just copy 1/1 pix, interpolate "online" 
        memcpy (outV[j], imgV[j], img->width_cr);
      }
*/
    // Generate 1/1th pel representation (used for integer pel MV search)
    GenerateFullPelRepresentation (out4Y, ref11, s->size_x, s->size_y);

}


/*!
 ************************************************************************
 * \brief
 *    Find SNR for all three components
 ************************************************************************
 */
static void find_snr ()
{
  int i, j;
  int diff_y, diff_u, diff_v;
  int impix;
  
  //  Calculate  PSNR for Y, U and V.
  
  //     Luma.
  impix = img->height * img->width;
  
  if (img->fld_flag != 0)
  {
      
    diff_y = 0;
    for (i = 0; i < img->width; ++i)
    {
      for (j = 0; j < img->height; ++j)
      {
        diff_y += img->quad[imgY_org[j][i] - imgY_com[j][i]];
      }
    }
    
    //     Chroma.
    diff_u = 0;
    diff_v = 0;
    
    for (i = 0; i < img->width_cr; i++)
    {
      for (j = 0; j < img->height_cr; j++)
      {
        diff_u += img->quad[imgUV_org[0][j][i] - imgUV_com[0][j][i]];
        diff_v += img->quad[imgUV_org[1][j][i] - imgUV_com[1][j][i]];
       }
    }
  }
  else
  { 
    imgY_org  = imgY_org_frm;
    imgUV_org = imgUV_org_frm;

    if(input->PicInterlace==ADAPTIVE_CODING)
    {
      enc_picture = enc_frame_picture;
    }  

    diff_y = 0;
    for (i = 0; i < img->width; ++i)
    {
      for (j = 0; j < img->height; ++j)
      {
        diff_y += img->quad[imgY_org[j][i] - enc_picture->imgY[j][i]];
      }
    }
    
    //     Chroma.
    diff_u = 0;
    diff_v = 0;
   
    for (i = 0; i < img->width_cr; i++)
    {
      for (j = 0; j < img->height_cr; j++)
      {
        diff_u += img->quad[imgUV_org[0][j][i] - enc_picture->imgUV[0][j][i]];
        diff_v += img->quad[imgUV_org[1][j][i] - enc_picture->imgUV[1][j][i]];
      }
    }
  }

#if ZEROSNR
  if (diff_y == 0)
    diff_y = 1;
  if (diff_u == 0)
    diff_u = 1;
  if (diff_v == 0)
    diff_v = 1; 
#endif

  //  Collecting SNR statistics
  if (diff_y != 0)
  {
    snr->snr_y = (float) (10 * log10 (65025 * (float) impix / (float) diff_y));         // luma snr for current frame
    snr->snr_u = (float) (10 * log10 (65025 * (float) impix / (float) (4 * diff_u)));   // u croma snr for current frame, 1/4 of luma samples
    snr->snr_v = (float) (10 * log10 (65025 * (float) impix / (float) (4 * diff_v)));   // v croma snr for current frame, 1/4 of luma samples
  }
  
  if (img->number == 0)
  {
    snr->snr_y1 = (float) (10 * log10 (65025 * (float) impix / (float) diff_y));        // keep luma snr for first frame
    snr->snr_u1 = (float) (10 * log10 (65025 * (float) impix / (float) (4 * diff_u)));  // keep croma u snr for first frame
    snr->snr_v1 = (float) (10 * log10 (65025 * (float) impix / (float) (4 * diff_v)));  // keep croma v snr for first frame
    snr->snr_ya = snr->snr_y1;
    snr->snr_ua = snr->snr_u1;
    snr->snr_va = snr->snr_v1;
  }
  // B pictures
  else
  {
    snr->snr_ya = (float) (snr->snr_ya * (img->number + Bframe_ctr) + snr->snr_y) / (img->number + Bframe_ctr + 1); // average snr lume for all frames inc. first
    snr->snr_ua = (float) (snr->snr_ua * (img->number + Bframe_ctr) + snr->snr_u) / (img->number + Bframe_ctr + 1); // average snr u croma for all frames inc. first
    snr->snr_va = (float) (snr->snr_va * (img->number + Bframe_ctr) + snr->snr_v) / (img->number + Bframe_ctr + 1); // average snr v croma for all frames inc. first

  }
}

/*!
 ************************************************************************
 * \brief
 *    Find distortion for all three components
 ************************************************************************
 */
static void find_distortion ()
{
  int i, j;
  int diff_y, diff_u, diff_v;
  int impix;
  
  //  Calculate  PSNR for Y, U and V.
  
  //     Luma.
  impix = img->height * img->width;
  
  if (img->structure!=FRAME)
  {

    diff_y = 0;
    for (i = 0; i < img->width; ++i)
    {
      for (j = 0; j < img->height; ++j)
      {
        diff_y += img->quad[abs (imgY_org[j][i] - imgY_com[j][i])];
      }
    }
    
    //     Chroma.
    
    diff_u = 0;
    diff_v = 0;
    
    for (i = 0; i < img->width_cr; i++)
    {
      for (j = 0; j < img->height_cr; j++)
      {
        diff_u += img->quad[abs (imgUV_org[0][j][i] - imgUV_com[0][j][i])];
        diff_v += img->quad[abs (imgUV_org[1][j][i] - imgUV_com[1][j][i])];
      }
    }
  }else
  {
      imgY_org   = imgY_org_frm;
      imgUV_org = imgUV_org_frm;

      diff_y = 0;
      for (i = 0; i < img->width; ++i)
      {
        for (j = 0; j < img->height; ++j)
        {
          diff_y += img->quad[abs (imgY_org[j][i] - enc_picture->imgY[j][i])];
        }
      }
      
      //     Chroma.
      
      diff_u = 0;
      diff_v = 0;
      
      for (i = 0; i < img->width_cr; i++)
      {
        for (j = 0; j < img->height_cr; j++)
        {
          diff_u += img->quad[abs (imgUV_org[0][j][i] - enc_picture->imgUV[0][j][i])];
          diff_v += img->quad[abs (imgUV_org[1][j][i] - enc_picture->imgUV[1][j][i])];
        }
      }
  }
  // Calculate real PSNR at find_snr_avg()
  snr->snr_y = (float) diff_y;
  snr->snr_u = (float) diff_u;
  snr->snr_v = (float) diff_v;
}

  
  /*!
 ************************************************************************
 * \brief
 *    Just a placebo
 ************************************************************************
 */
Boolean dummy_slice_too_big (int bits_slice)
{
  return FALSE;
}


/*! 
***************************************************************************
// For MB level field/frame coding
***************************************************************************
*/
void copy_rdopt_data (int bot_block)
{
  int mb_nr = img->current_mb_nr;
  Macroblock *currMB = &img->mb_data[mb_nr];
  int i, j, k, l;

  int bframe = (img->type == B_SLICE);
  int mode;
  int b8mode, b8pdir;

  int list_offset = ((img->MbaffFrameFlag)&&(currMB->mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  mode             = rdopt->mode;
  currMB->mb_type  = rdopt->mb_type;   // copy mb_type 
  currMB->cbp      = rdopt->cbp;   // copy cbp
  currMB->cbp_blk  = rdopt->cbp_blk;   // copy cbp_blk
  img->i16offset   = rdopt->i16offset;

  currMB->prev_qp=rdopt->prev_qp;
  currMB->prev_delta_qp=rdopt->prev_delta_qp;
  currMB->qp=rdopt->qp;

  currMB->c_ipred_mode = rdopt->c_ipred_mode;

  for (i = 0; i < 6; i++)
    for (j = 0; j < 4; j++)
      for (k = 0; k < 2; k++)
        for (l = 0; l < 18; l++)
          img->cofAC[i][j][k][l] = rdopt->cofAC[i][j][k][l];

  for (i = 0; i < 3; i++)
    for (k = 0; k < 2; k++)
      for (l = 0; l < 18; l++)
        img->cofDC[i][k][l] = rdopt->cofDC[i][k][l];

  for (j = 0; j < 4; j++)
    for (i = 0; i < 4; i++)
    {
      enc_picture->ref_idx[LIST_0][img->block_x + i][img->block_y + j] = rdopt->refar[LIST_0][j][i];
      enc_picture->ref_pic_id [LIST_0][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_0 + list_offset][enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]];
      if (bframe)
      {
        enc_picture->ref_idx[LIST_1][img->block_x + i][img->block_y + j] = rdopt->refar[LIST_1][j][i];
        enc_picture->ref_pic_id [LIST_1][img->block_x+i][img->block_y+j] = enc_picture->ref_pic_num[LIST_1 + list_offset][enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j]];
      }
    }
    
    //===== reconstruction values =====
  for (j = 0; j < 16; j++)
    for (i = 0; i < 16; i++)
    {
      enc_picture->imgY[img->pix_y + j][img->pix_x + i] = rdopt->rec_mbY[j][i];
    }
      
      
  for (j = 0; j < 8; j++)
    for (i = 0; i < 8; i++)
    {
      enc_picture->imgUV[0][img->pix_c_y + j][img->pix_c_x + i] = rdopt->rec_mbU[j][i];
      enc_picture->imgUV[1][img->pix_c_y + j][img->pix_c_x + i] = rdopt->rec_mbV[j][i];
    }


  for (i = 0; i < 4; i++)
  {
    currMB->b8mode[i] = rdopt->b8mode[i];
    currMB->b8pdir[i] = rdopt->b8pdir[i];
  }

  //==== intra prediction modes ====
  if (mode == P8x8)
  {
    for (k = 0, j = img->block_y; j < img->block_y + 4; j++)
      for (i = img->block_x; i < img->block_x + 4; i++, k++)
      {
        img->ipredmode[i][j]        = rdopt->ipredmode[i][j];
        currMB->intra_pred_modes[k] = rdopt->intra_pred_modes[k];
      }
  }
  else if (mode != I4MB)
  {
    for (k = 0, j = img->block_y; j < img->block_y + 4; j++)
      for (     i = img->block_x; i < img->block_x + 4; i++, k++)
      {
        img->ipredmode[i][j]        = DC_PRED;
        currMB->intra_pred_modes[k] = DC_PRED;
      }
  }
  else if (mode == I4MB)
  {
    for (k = 0, j = img->block_y; j < img->block_y + 4; j++)
      for (     i = img->block_x; i < img->block_x + 4; i++, k++)
      {
        img->ipredmode[i][j]        = rdopt->ipredmode[i][j];
        currMB->intra_pred_modes[k] = rdopt->intra_pred_modes[k];
        
      }
      
  }

  if (img->MbaffFrameFlag)
  {
    // motion vectors
    copy_motion_vectors_MB ();
    
    
    if (!IS_INTRA(currMB))
    {
      for (j = 0; j < 4; j++)
        for (i = 0; i < 4; i++)
        {
          b8mode = currMB->b8mode[i/2+2*(j/2)];
          b8pdir = currMB->b8pdir[i/2+2*(j/2)];

          if (b8pdir!=1)
          {
            enc_picture->mv[LIST_0][i+img->block_x][j+img->block_y][0] = rdopt->all_mv[i][j][LIST_0][rdopt->refar[LIST_0][j][i]][b8mode][0];
            enc_picture->mv[LIST_0][i+img->block_x][j+img->block_y][1] = rdopt->all_mv[i][j][LIST_0][rdopt->refar[LIST_0][j][i]][b8mode][1];
          }
          else
          {
            enc_picture->mv[LIST_0][i+img->block_x][j+img->block_y][0] = 0;
            enc_picture->mv[LIST_0][i+img->block_x][j+img->block_y][1] = 0;
          }
          if (bframe)
          {
            if (b8pdir!=0)
            {
              enc_picture->mv[LIST_1][i+img->block_x][j+img->block_y][0] = rdopt->all_mv[i][j][LIST_1][rdopt->refar[LIST_1][j][i]][b8mode][0];
              enc_picture->mv[LIST_1][i+img->block_x][j+img->block_y][1] = rdopt->all_mv[i][j][LIST_1][rdopt->refar[LIST_1][j][i]][b8mode][1];
            }
            else
            {
              enc_picture->mv[LIST_1][i+img->block_x][j+img->block_y][0] = 0;
              enc_picture->mv[LIST_1][i+img->block_x][j+img->block_y][1] = 0;
            }
          }
        }
    }
    else
    {
      for (j = 0; j < 4; j++)
        for (i = 0; i < 4; i++)
        {
          enc_picture->mv[LIST_0][i+img->block_x][j+img->block_y][0] = 0;
          enc_picture->mv[LIST_0][i+img->block_x][j+img->block_y][1] = 0;
          
          if (bframe)
          {
            enc_picture->mv[LIST_1][i+img->block_x][j+img->block_y][0] = 0;
            enc_picture->mv[LIST_1][i+img->block_x][j+img->block_y][1] = 0;
          }
        }
    }
  }
  
}                             // end of copy_rdopt_data
  
static void copy_motion_vectors_MB ()
{
  int i,j,k,l;

  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < 4; j++)
    {
      for (k = 0; k < img->max_num_references; k++)
      {
        for (l = 0; l < 9; l++)
        {
          img->all_mv[i][j][LIST_0][k][l][0] = rdopt->all_mv[i][j][LIST_0][k][l][0];
          img->all_mv[i][j][LIST_0][k][l][1] = rdopt->all_mv[i][j][LIST_0][k][l][1];

          img->all_mv[i][j][LIST_1][k][l][0] = rdopt->all_mv[i][j][LIST_1][k][l][0];
          img->all_mv[i][j][LIST_1][k][l][1] = rdopt->all_mv[i][j][LIST_1][k][l][1];

          img->pred_mv[i][j][LIST_0][k][l][0] = rdopt->pred_mv[i][j][LIST_0][k][l][0];
          img->pred_mv[i][j][LIST_0][k][l][1] = rdopt->pred_mv[i][j][LIST_0][k][l][1];
          
          img->pred_mv[i][j][LIST_1][k][l][0] = rdopt->pred_mv[i][j][LIST_1][k][l][0];
          img->pred_mv[i][j][LIST_1][k][l][1] = rdopt->pred_mv[i][j][LIST_1][k][l][1];
          
        }
      }
    }
  }
}
  

static void ReportNALNonVLCBits(int tmp_time, int me_time)
{
  //! Need to add type (i.e. SPS, PPS, SEI etc).
    printf ("%04d(NVB)%8d \n", frame_no, stat->bit_ctr_parametersets_n);

}
static void ReportFirstframe(int tmp_time,int me_time)
{
  //Rate control
  int bits;
  printf ("%04d(IDR)%8d %1d %2d %7.3f %7.3f %7.3f  %7d   %5d     %3s   %3d\n",
    frame_no, stat->bit_ctr - stat->bit_ctr_n,0,
    img->qp, snr->snr_y, snr->snr_u, snr->snr_v, tmp_time, me_time,
    img->fld_flag ? "FLD" : "FRM", intras);

  //Rate control
  if(input->RCEnable)
  {
    if((!input->PicInterlace)&&(!input->MbInterlace))
        bits = stat->bit_ctr-stat->bit_ctr_n; // used for rate control update 
    else
    {
      bits = stat->bit_ctr - Iprev_bits; // used for rate control update 
      Iprev_bits = stat->bit_ctr;
    }
  }

  stat->bitr0 = stat->bitr;
  stat->bit_ctr_0 = stat->bit_ctr;
  stat->bit_ctr = 0;
}


static void ReportIntra(int tmp_time, int me_time)
{
	
  if (img->currentPicture->idr_flag == 1)
    printf ("%04d(IDR)%8d %1d %2d %7.3f %7.3f %7.3f  %7d   %5d     %3s   %3d\n",
    frame_no, stat->bit_ctr - stat->bit_ctr_n, 0,
    img->qp, snr->snr_y, snr->snr_u, snr->snr_v, tmp_time, me_time,
    img->fld_flag ? "FLD" : "FRM", intras); 
  else
    printf ("%04d(I)  %8d %1d %2d %7.3f %7.3f %7.3f  %7d   %5d     %3s   %3d\n",
    frame_no, stat->bit_ctr - stat->bit_ctr_n, 0,
    img->qp, snr->snr_y, snr->snr_u, snr->snr_v, tmp_time, me_time,
    img->fld_flag ? "FLD" : "FRM", intras);

}

static void ReportSP(int tmp_time, int me_time)
{
  printf ("%04d(SP) %8d %1d %2d %7.3f %7.3f %7.3f  %7d   %5d     %3s   %3d\n",
    frame_no, stat->bit_ctr - stat->bit_ctr_n, active_pps->weighted_pred_flag, img->qp, snr->snr_y,
    snr->snr_u, snr->snr_v, tmp_time, me_time,
    img->fld_flag ? "FLD" : "FRM", intras);
}

static void ReportBS(int tmp_time, int me_time)
{
  printf ("%04d(BS) %8d %1d %2d %7.3f %7.3f %7.3f  %7d   %5d     %3s   %3d %1d\n",
    frame_no, stat->bit_ctr - stat->bit_ctr_n, active_pps->weighted_bipred_idc, img->qp, snr->snr_y,
    snr->snr_u, snr->snr_v, tmp_time, me_time,
    img->fld_flag ? "FLD" : "FRM", intras,img->direct_type);
}

static void ReportB(int tmp_time, int me_time)
{
    printf ("%04d(B)  %8d %1d %2d %7.3f %7.3f %7.3f  %7d   %5d     %3s   %3d %1d\n",
    frame_no, stat->bit_ctr - stat->bit_ctr_n, active_pps->weighted_bipred_idc,img->qp,
    snr->snr_y, snr->snr_u, snr->snr_v, tmp_time,me_time,
    img->fld_flag ? "FLD" : "FRM",intras,img->direct_type);
}


static void ReportP(int tmp_time, int me_time)
{            
    printf ("%04d(P)  %8d %1d %2d %7.3f %7.3f %7.3f  %7d   %5d     %3s   %3d\n",
    frame_no, stat->bit_ctr - stat->bit_ctr_n, active_pps->weighted_pred_flag, img->qp, snr->snr_y,
    snr->snr_u, snr->snr_v, tmp_time, me_time,
    img->fld_flag ? "FLD" : "FRM", intras);

}

/*!
 ************************************************************************
 * \brief
 *    Copies contents of a Sourceframe structure into the old-style
 *    variables imgY_org_frm and imgUV_org_frm.  No other side effects
 * \param sf
 *    the source frame the frame is to be taken from
 ************************************************************************
 */

static void CopyFrameToOldImgOrgVariables (Sourceframe *sf)
{
  int x, y;

  for (y=0; y<sf->y_framesize; y++)
    for (x=0; x<sf->x_size; x++)
      imgY_org_frm [y][x] = sf->yf[y*sf->x_size+x];
  for (y=0; y<sf->y_framesize/2; y++)
    for (x=0; x<sf->x_size/2; x++)
    {
      imgUV_org_frm[0][y][x] = sf->uf[y*sf->x_size/2+x];
      imgUV_org_frm[1][y][x] = sf->vf[y*sf->x_size/2+x];
    }
}

/*!
 ************************************************************************
 * \brief
 *    Copies contents of a Sourceframe structure into the old-style
 *    variables imgY_org_top and imgUV_org_top.  No other side effects
 * \param sf
 *    the source frame the field is to be taken from
 ************************************************************************
 */

static void CopyTopFieldToOldImgOrgVariables (Sourceframe *sf)
{
  int x, y;

  for (y=0; y<sf->y_fieldsize; y++)
    for (x=0; x<sf->x_size; x++)
      imgY_org_top [y][x] = sf->yt[y*sf->x_size+x];
  for (y=0; y<sf->y_fieldsize/2; y++)
    for (x=0;x<sf->x_size/2; x++)
    {
      imgUV_org_top[0][y][x] = sf->ut[y*sf->x_size/2+x];
      imgUV_org_top[1][y][x] = sf->vt[y*sf->x_size/2+x];
    }
}
/*!
 ************************************************************************
 * \brief
 *    Copies contents of a Sourceframe structure into the old-style
 *    variables imgY_org_bot and imgUV_org_bot.  No other side effects
 * \param sf
 *    the source frame the field is to be taken from
 ************************************************************************
 */

static void CopyBottomFieldToOldImgOrgVariables (Sourceframe *sf)
{
  int x, y;

  for (y=0; y<sf->y_fieldsize; y++)
    for (x=0; x<sf->x_size; x++)
      imgY_org_bot [y][x] = sf->yb[y*sf->x_size+x];
  for (y=0; y<sf->y_fieldsize/2; y++)
    for (x=0;x<sf->x_size/2; x++)
    {
      imgUV_org_bot[0][y][x] = sf->ub[y*sf->x_size/2+x];
      imgUV_org_bot[1][y][x] = sf->vb[y*sf->x_size/2+x];
    }
}


/*!
 ************************************************************************
 * \brief
 *    Allocates Sourceframe structure
 * \param xs
 *    horizontal size of frame in pixels
 * \param ys
 *    vertical size of frame in pixels, must be divisible by 2
 * \return
 *    pointer to initialized source frame structure
 ************************************************************************
 */

static Sourceframe *AllocSourceframe (int xs, int ys)
{
  Sourceframe *sf = NULL;
  const unsigned int bytes_y = xs*ys;
  const unsigned int bytes_uv = (xs*ys)/4;

  if ((sf = calloc (1, sizeof (Sourceframe))) == NULL) no_mem_exit ("ReadOneFrame: sf");
  if (sf->yf == NULL) if ((sf->yf = calloc (1, bytes_y)) == NULL) no_mem_exit ("ReadOneFrame: sf->yf");
  if (sf->yt == NULL) if ((sf->yt = calloc (1, bytes_y/2)) == NULL) no_mem_exit ("ReadOneFrame: sf->yt");
  if (sf->yb == NULL) if ((sf->yb = calloc (1, bytes_y/2)) == NULL) no_mem_exit ("ReadOneFrame: sf->yb");
  if (sf->uf == NULL) if ((sf->uf = calloc (1, bytes_uv)) == NULL) no_mem_exit ("ReadOneFrame: sf->uf");
  if (sf->ut == NULL) if ((sf->ut = calloc (1, bytes_uv/2)) == NULL) no_mem_exit ("ReadOneFrame: sf->ut");
  if (sf->ub == NULL) if ((sf->ub = calloc (1, bytes_uv/2)) == NULL) no_mem_exit ("ReadOneFrame: sf->ub");
  if (sf->vf == NULL) if ((sf->vf = calloc (1, bytes_uv)) == NULL) no_mem_exit ("ReadOneFrame: sf->vf");
  if (sf->vt == NULL) if ((sf->vt = calloc (1, bytes_uv/2)) == NULL) no_mem_exit ("ReadOneFrame: sf->vt");
  if (sf->vb == NULL) if ((sf->vb = calloc (1, bytes_uv/2)) == NULL) no_mem_exit ("ReadOneFrame: sf->vb");
  sf->x_size = xs;
  sf->y_framesize = ys;
  sf->y_fieldsize = ys/2;

  return sf;
}



/*!
 ************************************************************************
 * \brief
 *    Frees Sourceframe structure
 * \param sf
 *    pointer to Sourceframe previoously allocated with ALlocSourceframe()
 * \return
 *    none
 ************************************************************************
 */

static void FreeSourceframe (Sourceframe *sf)
{
  if (sf!=NULL) 
  {
    if (sf->yf != NULL) free (sf->yf);
    if (sf->yt != NULL) free (sf->yt);
    if (sf->yb != NULL) free (sf->yb);
    if (sf->uf != NULL) free (sf->uf);
    if (sf->ut != NULL) free (sf->ut);
    if (sf->ub != NULL) free (sf->ub);
    if (sf->vf != NULL) free (sf->vf);
    if (sf->vt != NULL) free (sf->vt);
    if (sf->vb != NULL) free (sf->vb);
    free (sf);
  }
}

/*!
 ************************************************************************
 * \brief
 *    Calculates the absolute frame number in the source file out
 *    of various variables in img-> and input->
 * \return
 *    frame number in the file to be read
 * \par side effects
 *    global variable frame_no updated -- dunno, for what this one is necessary
 ************************************************************************
 */
static int CalculateFrameNumber()
{
  if (img->type == B_SLICE)
    frame_no = start_tr_in_this_IGOP + (IMG_NUMBER - 1) * (input->jumpd + 1) + img->b_interval * img->b_frame_to_code;
  else
    {
      frame_no = start_tr_in_this_IGOP + IMG_NUMBER * (input->jumpd + 1);
#ifdef _ADAPT_LAST_GROUP_
      if (input->last_frame && img->number + 1 == input->no_frames)
        frame_no = input->last_frame;
#endif
    }
  return frame_no;
}


/*!
 ************************************************************************
 * \brief
 *    Generate Field Component from Frame Components by copying
 * \param src
 *    source frame component
 * \param top
 *    destination top field component
 * \param bot
 *    destination bottom field component
 * \param xs
 *    horizontal size of frame in pixels
 * \param ys
 *    vertical size of frame in pixels, must be divisible by 2
 ************************************************************************
 */
static void GenerateFieldComponent (char *src, char *top, char *bot, int xs, int ys)
{
  int fieldline;
  assert (ys % 2 == 0);

  for (fieldline = 0; fieldline < ys/2; fieldline++)
  {
    memcpy (&top[xs * fieldline], &src[xs * (fieldline * 2 + 0)], xs);
    memcpy (&bot[xs * fieldline], &src[xs * (fieldline * 2 + 1)], xs);
  }
}


/*!
 ************************************************************************
 * \brief
 *    Reads one new frame from file
 * \param FrameNoInFile
 *    Frame number in the source file
 * \param HeaderSize
 *    Number of bytes in the source file to be skipped
 * \param xs
 *    horizontal size of frame in pixels, must be divisible by 16
 * \param ys
 *    vertical size of frame in pixels, must be divisible by 16 or
 *    32 in case of MB-adaptive frame/field coding
 * \param sf
 *    Sourceframe structure to which the frame is written
 ************************************************************************
 */
static void ReadOneFrame (int FrameNoInFile, int HeaderSize, int xs, int ys, Sourceframe *sf)
{
  int i;

  const unsigned int bytes_y = xs*ys;
  const unsigned int bytes_uv = (xs*ys)/4;
  const int framesize_in_bytes = bytes_y + 2*bytes_uv;

  assert (xs % MB_BLOCK_SIZE == 0);
  assert (ys % MB_BLOCK_SIZE == 0);
  assert (p_in != NULL);
  assert (sf != NULL);
  assert (sf->yf != NULL);

  assert (FrameNumberInFile == FrameNoInFile);
  // printf ("ReadOneFrame: frame_no %d xs %d ys %d\n", FrameNoInFile, xs, ys);

  if (fseek (p_in, HeaderSize, SEEK_SET) != 0)
    error ("ReadOneFrame: cannot fseek to (Header size) in p_in", -1);

  // the reason for the following loop is to support source files bigger than
  // MAXINT.  In most operating systems, including Windows, it is possible to
  // fseek to file positions bigger than MAXINT by using this relative seeking
  // technique.  StW, 12/30/02
  // Skip starting frames
  for (i=0; i<input->start_frame; i++)
    if (fseek (p_in, framesize_in_bytes, SEEK_CUR) != 0) 
    {
      printf ("ReadOneFrame: cannot advance file pointer in p_in beyond frame %d, looping to picture zero\n", i);
      if (fseek (p_in, HeaderSize, SEEK_SET) != 0)
        report_stats_on_error();
        exit (-1);
    } 

  for (i=0; i<FrameNoInFile; i++)
    if (fseek (p_in, framesize_in_bytes, SEEK_CUR) != 0) 
    {
      printf ("ReadOneFrame: cannot advance file pointer in p_in beyond frame %d, looping to picture zero\n", i);
      if (fseek (p_in, HeaderSize, SEEK_SET) != 0)
        error ("ReadOneFrame: cannot fseek to (Header size) in p_in", -1);
    }

  // Here we are at the correct position for the source frame in the file.  Now
  // read it.
  if (fread (sf->yf, 1, bytes_y, p_in) != bytes_y)
  {
    printf ("ReadOneFrame: cannot read %d bytes from input file, unexpected EOF?, exiting", bytes_y);
    report_stats_on_error();
    exit (-1);
  }
  if (fread (sf->uf, 1, bytes_uv, p_in) != bytes_uv)
  {
    printf ("ReadOneFrame: cannot read %d bytes from input file, unexpected EOF?, exiting", bytes_uv);
    report_stats_on_error();
    exit (-1);
  }
  if (fread (sf->vf, 1, bytes_uv, p_in) != bytes_uv)
  {
    printf ("ReadOneFrame: cannot read %d bytes from input file, unexpected EOF?, exiting", bytes_uv);
    report_stats_on_error();
    exit (-1);
  }

  // Complete frame is read into sf->?f, now setup 
  // top and bottom field (sf->?t and sf->?b)

  GenerateFieldComponent (sf->yf, sf->yt, sf->yb, xs, ys);
  GenerateFieldComponent (sf->uf, sf->ut, sf->ub, xs/2, ys/2);
  GenerateFieldComponent (sf->vf, sf->vt, sf->vb, xs/2, ys/2);
}


/*!
 ************************************************************************
 * \brief
 *    point to frame coding variables 
 ************************************************************************
 */
static void put_buffer_frame()
{
  imgY_org  = imgY_org_frm;
  imgUV_org = imgUV_org_frm;  
}

/*!
 ************************************************************************
 * \brief
 *    point to top field coding variables 
 ************************************************************************
 */
static void put_buffer_top()
{
  img->fld_type = 0;

  imgY_org = imgY_org_top;
  imgUV_org = imgUV_org_top;
}

/*!
 ************************************************************************
 * \brief
 *    point to bottom field coding variables 
 ************************************************************************
 */
static void put_buffer_bot()
{
  img->fld_type = 1;

  imgY_org = imgY_org_bot;
  imgUV_org = imgUV_org_bot;
}

/*!
 ************************************************************************
 * \brief
 *    Writes a NAL unit of a partition or slice
 ************************************************************************
 */

static void writeUnit(Bitstream* currStream,int partition)
{
  NALU_t *nalu;
  assert (currStream->bits_to_go == 8);
  nalu = AllocNALU(img->width*img->height*4);
  nalu->startcodeprefix_len = 2+(img->current_mb_nr == 0?ZEROBYTES_SHORTSTARTCODE+1:ZEROBYTES_SHORTSTARTCODE);
//printf ("nalu->startcodeprefix_len %d\n", nalu->startcodeprefix_len);
  nalu->len = currStream->byte_pos +1;            // add one for the first byte of the NALU
//printf ("nalu->len %d\n", nalu->len);
  memcpy (&nalu->buf[1], currStream->streamBuffer, nalu->len-1);
  if (img->currentPicture->idr_flag)
  {
    nalu->nal_unit_type = NALU_TYPE_IDR;
    nalu->nal_reference_idc = NALU_PRIORITY_HIGHEST;
  }
  else if (img->type == B_SLICE)
  {
		//different nal header for different partitions
    if(input->partition_mode == 0)
		{
      nalu->nal_unit_type = NALU_TYPE_SLICE;
		}
	  else
		{
      nalu->nal_unit_type = NALU_TYPE_DPA +  partition;
		}
    
    if (img->nal_reference_idc !=0)
		{
			nalu->nal_reference_idc = NALU_PRIORITY_HIGH;
		}
    else
    {
      nalu->nal_reference_idc = NALU_PRIORITY_DISPOSABLE;
    }
  }
  else   // non-b frame, non IDR slice
  {
		//different nal header for different partitions
    if(input->partition_mode == 0)
    {
      nalu->nal_unit_type = NALU_TYPE_SLICE;
		}
	  else
		{
     nalu->nal_unit_type = NALU_TYPE_DPA +  partition;
		}
    if (img->nal_reference_idc !=0)
    {
      nalu->nal_reference_idc = NALU_PRIORITY_HIGH;
    }
    else
    {
      nalu->nal_reference_idc = NALU_PRIORITY_DISPOSABLE;
    }
  }
  nalu->forbidden_bit = 0;
  stat->bit_ctr += WriteNALU (nalu);
  
  FreeNALU(nalu);
}


