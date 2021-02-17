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
 * \file header.c
 *
 * \brief
 *    H.264 Slice and Sequence headers
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *************************************************************************************
 */

#include <math.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>

#include "elements.h"
#include "header.h"
#include "rtp.h"
#include "mbuffer.h"
#include "defines.h"
#include "vlc.h"
#include "parset.h"

// A little trick to avoid those horrible #if TRACE all over the source code
#if TRACE
#define SYMTRACESTRING(s) strncpy(sym->tracestring,s,TRACESTRING_SIZE)
#else
#define SYMTRACESTRING(s) // do nothing
#endif

int * assignSE2partition[2] ;
int assignSE2partition_NoDP[SE_MAX_ELEMENTS] =
  {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int assignSE2partition_DP[SE_MAX_ELEMENTS] =
  {  0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 2, 2, 2, 2, 0, 0, 0, 0 } ;

static int ref_pic_list_reordering();
static int dec_ref_pic_marking();
static int pred_weight_table();

/*!
 ********************************************************************************************
 * \brief 
 *    Write a slice header
 ********************************************************************************************
*/
int SliceHeader()
{
  int dP_nr = assignSE2partition[input->partition_mode][SE_HEADER];
  DataPartition *partition = &((img->currentSlice)->partArr[dP_nr]);
  Slice* currSlice = img->currentSlice;
  int len = 0;
  unsigned int field_pic_flag = 0, bottom_field_flag = 0;    // POC200301

  int num_bits_slice_group_change_cycle;
  float numtmp;	
	
  if (img->MbaffFrameFlag)
    len  = ue_v("SH: first_mb_in_slice", img->current_mb_nr >> 1,   partition);
  else
    len  = ue_v("SH: first_mb_in_slice", img->current_mb_nr,   partition);

  len += ue_v("SH: slice_type",        get_picture_type (),   partition);

  // Note: Encoder supports only one pic/seq parameter set, hence value is
  // hard coded to zero
//  len += ue_v("SH: pic_parameter_set_id" , 0 ,partition);
  len += ue_v("SH: pic_parameter_set_id" , active_pps->pic_parameter_set_id ,partition);

  // frame_num
//  if(input->no_frames >= 1<<(log2_max_frame_num_minus4+4))
//    error ("Too many frames.  Increase log2_max_frame_num_minus4",-999);  

  len += u_v (log2_max_frame_num_minus4 + 4,"SH: frame_num", img->frame_num, partition);

  if (!active_sps->frame_mbs_only_flag)
  {
    // field_pic_flag    u(1)
    field_pic_flag = (img->structure ==TOP_FIELD || img->structure ==BOTTOM_FIELD)?1:0;
    assert( field_pic_flag == img->fld_flag );
    len += u_1("SH: field_pic_flag", field_pic_flag, partition);

    if (field_pic_flag)
    {
      //bottom_field_flag     u(1)
      bottom_field_flag = (img->structure == BOTTOM_FIELD)?1:0;
      len += u_1("SH: bottom_field_flag" , bottom_field_flag ,partition);
    }
  }

  if (img->currentPicture->idr_flag)
  {
    // idr_pic_id
    // hard coded to zero because we don't have proper IDR handling at the moment
    len += ue_v ("SH: idr_pic_id", 0, partition);
  }

  // POC200301
  if (img->pic_order_cnt_type == 0)
  {
    if (active_sps->frame_mbs_only_flag)
    {
      img->pic_order_cnt_lsb = (img->toppoc & ~((((unsigned int)(-1)) << (log2_max_pic_order_cnt_lsb_minus4+4))) );
    }
    else
    {
      if (!field_pic_flag || img->structure == TOP_FIELD)
        img->pic_order_cnt_lsb = (img->toppoc & ~((((unsigned int)(-1)) << (log2_max_pic_order_cnt_lsb_minus4+4))) );
      else if ( img->structure == BOTTOM_FIELD )
        img->pic_order_cnt_lsb = (img->bottompoc & ~((((unsigned int)(-1)) << (log2_max_pic_order_cnt_lsb_minus4+4))) );
    }

    len += u_v (log2_max_pic_order_cnt_lsb_minus4+4, "SH: pic_order_cnt_lsb", img->pic_order_cnt_lsb, partition);

    if (img->pic_order_present_flag && !field_pic_flag)  // img->fld_flag
    {
      len += se_v ("SH: delta_pic_order_cnt_bottom", img->delta_pic_order_cnt_bottom, partition);
    }
  }
  if (img->pic_order_cnt_type == 1 && !img->delta_pic_order_always_zero_flag)
  {
    len += se_v ("SH: delta_pic_order_cnt[0]", img->delta_pic_order_cnt[0], partition);

    if (img->pic_order_present_flag && !field_pic_flag)  // img->fld_flag
    {
      len += se_v ("SH: delta_pic_order_cnt[1]", img->delta_pic_order_cnt[1], partition);
    }
  }

  // redundant slice info redundant_pic_cnt is missing here
  if (input->redundant_slice_flag)
  {
    len += ue_v ("SH: redundant_pic_cnt", img->redundant_pic_cnt, partition);
  }

  // Direct Mode Type selection for B pictures
  if (img->type==B_SLICE)
  {
    len +=  u_1 ("SH: direct_spatial_mv_pred_flag", img->direct_type, partition);  	
    //len +=  u_1 ("SH: direct_spatial_mv_pred_flag", input->direct_type, partition);
  }

  if ((img->type == P_SLICE) || (img->type == B_SLICE) || (img->type==SP_SLICE))
  {
    int override_flag;
    if ((img->type == P_SLICE) || (img->type==SP_SLICE))
    {
      override_flag = (img->num_ref_idx_l0_active != (active_pps->num_ref_idx_l0_active_minus1 +1)) ? 1 : 0;
    }
    else
    {
      override_flag = ((img->num_ref_idx_l0_active != (active_pps->num_ref_idx_l0_active_minus1 +1)) 
                      || (img->num_ref_idx_l1_active != (active_pps->num_ref_idx_l1_active_minus1 +1))) ? 1 : 0;
    }
    // num_ref_idx_active_override_flag here always 1
    len +=  u_1 ("SH: num_ref_idx_active_override_flag", override_flag, partition);
    
    if (override_flag) 
    {
      len += ue_v ("SH: num_ref_idx_l0_active_minus1", img->num_ref_idx_l0_active-1, partition);
      if (img->type==B_SLICE)
      {
        len += ue_v ("SH: num_ref_idx_l1_active_minus1", img->num_ref_idx_l1_active-1, partition);
      }
    }

  }
  len += ref_pic_list_reordering();

  //if (((img->type == P_SLICE || img->type == SP_SLICE) && input->WeightedPrediction) || 
  //   ((img->type == B_SLICE) && input->WeightedBiprediction == 1))
  if (((img->type == P_SLICE || img->type == SP_SLICE) && active_pps->weighted_pred_flag) || 
     ((img->type == B_SLICE) && active_pps->weighted_bipred_idc == 1))  
  {
    len += pred_weight_table();
  }

  if (img->nal_reference_idc)
    len += dec_ref_pic_marking();

  if(input->symbol_mode==CABAC && img->type!=I_SLICE /*&& img->type!=SI_IMG*/)
  {
    len += ue_v("SH: cabac_init_idc", img->model_number, partition);
  }

  // we transmit zero in the pps, so here the real QP
  //len += se_v("SH: slice_qp_delta", (img->qp - 26), partition);
  len += se_v("SH: slice_qp_delta", (currSlice->qp - 26 - active_pps->pic_init_qp_minus26), partition);  

  if (img->type==SP_SLICE /*|| img->type==SI_SLICE*/)
  {
    if (img->type==SP_SLICE) // Switch Flag only for SP pictures
    {
      len += u_1 ("SH: sp_for_switch_flag", 0, partition);   // 1 for switching SP, 0 for normal SP
    }
    len += se_v ("SH: slice_qs_delta", (img->qpsp - 26), partition );
  }
/*
  if (input->LFSendParameters)
  {
    len += ue_v("SH: disable_deblocking_filter_idc",input->LFDisableIdc, partition);  // Turn loop filter on/off on slice basis 

    if (input->LFDisableIdc!=1)
    {
      len += se_v ("SH: slice_alpha_c0_offset_div2", input->LFAlphaC0Offset / 2, partition);

      len += se_v ("SH: slice_beta_offset_div2", input->LFBetaOffset / 2, partition);
    }
  }
*/
  if (active_pps->deblocking_filter_control_present_flag)
  {
    len += ue_v("SH: disable_deblocking_filter_idc",img->LFDisableIdc, partition);  // Turn loop filter on/off on slice basis 

    if (img->LFDisableIdc!=1)
    {
      len += se_v ("SH: slice_alpha_c0_offset_div2", img->LFAlphaC0Offset / 2, partition);

      len += se_v ("SH: slice_beta_offset_div2", img->LFBetaOffset / 2, partition);
    }
  }

	
  if ( active_pps->num_slice_groups_minus1>0 &&
    active_pps->slice_group_map_type>=3 && active_pps->slice_group_map_type<=5)
  {
    numtmp=img->PicHeightInMapUnits*img->PicWidthInMbs/(float)(active_pps->slice_group_change_rate_minus1+1)+1;
    num_bits_slice_group_change_cycle = (int)ceil(log(numtmp)/log(2));
    
    //! img->slice_group_change_cycle can be changed before calling FmoInit()
    len += u_v (num_bits_slice_group_change_cycle, "SH: slice_group_change_cycle", img->slice_group_change_cycle, partition);
  }
  
  // NOTE: The following syntax element is actually part 
  //        Slice data partition A RBSP syntax

  if(input->partition_mode&&!img->currentPicture->idr_flag)
  {
    len += ue_v("DPA: slice_id", img->current_slice_nr, partition);
  }

  return len;
}

/*!
 ********************************************************************************************
 * \brief 
 *    writes the ref_pic_list_reordering syntax
 *    based on content of according fields in img structure
 *
 * \return
 *    number of bits used 
 ********************************************************************************************
*/
static int ref_pic_list_reordering()
{
  int dP_nr = assignSE2partition[input->partition_mode][SE_HEADER];
  DataPartition *partition = &((img->currentSlice)->partArr[dP_nr]);
  Slice *currSlice = img->currentSlice;

  int i, len=0;

  if ((img->type!=I_SLICE) /*&&(img->type!=SI_IMG)*/ )
  {
    len += u_1 ("SH: ref_pic_list_reordering_flag_l0", currSlice->ref_pic_list_reordering_flag_l0, partition);
    if (currSlice->ref_pic_list_reordering_flag_l0)
    {
      i=-1;
      do
      {
        i++;
        len += ue_v ("SH: remapping_of_pic_num_idc", currSlice->remapping_of_pic_nums_idc_l0[i], partition);
        if (currSlice->remapping_of_pic_nums_idc_l0[i]==0 ||
            currSlice->remapping_of_pic_nums_idc_l0[i]==1)
        {
          len += ue_v ("SH: abs_diff_pic_num_minus1_l0", currSlice->abs_diff_pic_num_minus1_l0[i], partition);
        }
        else
        {
          if (currSlice->remapping_of_pic_nums_idc_l0[i]==2)
          {
            len += ue_v ("SH: long_term_pic_idx_l0", currSlice->long_term_pic_idx_l0[i], partition);
          }
        }

      } while (currSlice->remapping_of_pic_nums_idc_l0[i] != 3);
    }
  }

  if (img->type==B_SLICE)
  {
    len += u_1 ("SH: ref_pic_list_reordering_flag_l1", currSlice->ref_pic_list_reordering_flag_l1, partition);
    if (currSlice->ref_pic_list_reordering_flag_l1)
    {
      i=-1;
      do
      {
        i++;
        len += ue_v ("SH: remapping_of_pic_num_idc", currSlice->remapping_of_pic_nums_idc_l1[i], partition);
        if (currSlice->remapping_of_pic_nums_idc_l1[i]==0 ||
            currSlice->remapping_of_pic_nums_idc_l1[i]==1)
        {
          len += ue_v ("SH: abs_diff_pic_num_minus1_l1", currSlice->abs_diff_pic_num_minus1_l1[i], partition);
        }
        else
        {
          if (currSlice->remapping_of_pic_nums_idc_l1[i]==2)
          {
            len += ue_v ("SH: long_term_pic_idx_l1", currSlice->long_term_pic_idx_l1[i], partition);
          }
        }
      } while (currSlice->remapping_of_pic_nums_idc_l1[i] != 3);
    }
  }

  return len;
}


/*!
 ************************************************************************
 * \brief
 *    write the memory menagement control operations
 ************************************************************************
 */
static int dec_ref_pic_marking()
{
  int dP_nr = assignSE2partition[input->partition_mode][SE_HEADER];
  DataPartition *partition = &((img->currentSlice)->partArr[dP_nr]);

  DecRefPicMarking_t *tmp_drpm;

  int val, len=0;

  if (img->currentPicture->idr_flag)
  {
    len += u_1("SH: no_output_of_prior_pics_flag", img->no_output_of_prior_pics_flag, partition);
    len += u_1("SH: long_term_reference_flag", img->long_term_reference_flag, partition);
  }
  else
  {
    img->adaptive_ref_pic_buffering_flag = (img->dec_ref_pic_marking_buffer!=NULL);

    len += u_1("SH: adaptive_ref_pic_buffering_flag", img->adaptive_ref_pic_buffering_flag, partition);

    if (img->adaptive_ref_pic_buffering_flag)
    {
      tmp_drpm = img->dec_ref_pic_marking_buffer;
      // write Memory Management Control Operation 
      do
      {
        if (tmp_drpm==NULL) error ("Error encoding MMCO commands", 500);
        
        val = tmp_drpm->memory_management_control_operation;
        len += ue_v("SH: memory_management_control_operation", val, partition);

        if ((val==1)||(val==3)) 
        {
          len += 1 + ue_v("SH: difference_of_pic_nums_minus1", tmp_drpm->difference_of_pic_nums_minus1, partition);
        }
        if (val==2)
        {
          len+= ue_v("SH: long_term_pic_num", tmp_drpm->long_term_pic_num, partition);
        }
        if ((val==3)||(val==6))
        {
          len+= ue_v("SH: long_term_frame_idx", tmp_drpm->long_term_frame_idx, partition);
        }
        if (val==4)
        {
          len += ue_v("SH: max_long_term_pic_idx_plus1", tmp_drpm->max_long_term_frame_idx_plus1, partition);
        }
        
        tmp_drpm=tmp_drpm->Next;
        
      } while (val != 0);
      
    }
  }
  return len;
}

/*!
 ************************************************************************
 * \brief
 *    write the memory menagement control operations
 ************************************************************************
 */
static int pred_weight_table()
{
  int dP_nr = assignSE2partition[input->partition_mode][SE_HEADER];
  DataPartition *partition = &((img->currentSlice)->partArr[dP_nr]);

  int len = 0;
  int i,j;

  len += ue_v("SH: luma_log_weight_denom", luma_log_weight_denom, partition);
  
  len += ue_v("SH: chroma_log_weight_denom", chroma_log_weight_denom, partition);

  for (i=0; i< img->num_ref_idx_l0_active; i++)
  {
    if ( (wp_weight[0][i][0] != 1<<luma_log_weight_denom) || (wp_offset[0][i][0] != 0) )
    {
      len += u_1 ("SH: luma_weight_flag_l0", 1, partition);
      
      len += se_v ("SH: luma_weight_l0", wp_weight[0][i][0], partition);
        
      len += se_v ("SH: luma_offset_l0", wp_offset[0][i][0], partition);
    }
    else
    {
        len += u_1 ("SH: luma_weight_flag_l0", 0, partition);
    }

    if ( (wp_weight[0][i][1] != 1<<chroma_log_weight_denom) || (wp_offset[0][i][1] != 0) || 
     (wp_weight[0][i][2] != 1<<chroma_log_weight_denom) || (wp_offset[0][i][2] != 0)  )
    {
      len += u_1 ("chroma_weight_flag_l0", 1, partition);
      for (j=1; j<3; j++)
      {
        len += se_v ("chroma_weight_l0", wp_weight[0][i][j] ,partition);
      
        len += se_v ("chroma_offset_l0", wp_offset[0][i][j] ,partition);
      }
    }
    else
    {
      len += u_1 ("chroma_weight_flag_l0", 0, partition);
    }
  }

  if (img->type == B_SLICE)
  {
    for (i=0; i< img->num_ref_idx_l1_active; i++)
    {
      if ( (wp_weight[1][i][0] != 1<<luma_log_weight_denom) || (wp_offset[1][i][0] != 0) )
      {
        len += u_1 ("SH: luma_weight_flag_l1", 1, partition);
        
        len += se_v ("SH: luma_weight_l1", wp_weight[1][i][0], partition);
        
        len += se_v ("SH: luma_offset_l1", wp_offset[1][i][0], partition);
      }
      else
      {
        len += u_1 ("SH: luma_weight_flag_l1", 0, partition);
      }
      
      if ( (wp_weight[1][i][1] != 1<<chroma_log_weight_denom) || (wp_offset[1][i][1] != 0) || 
      (wp_weight[1][i][2] != 1<<chroma_log_weight_denom) || (wp_offset[1][i][2] != 0) )
      {
        len += u_1 ("chroma_weight_flag_l1", 1, partition);
        for (j=1; j<3; j++)
        {
          len += se_v ("chroma_weight_l1", wp_weight[1][i][j] ,partition);
          
          len += se_v ("chroma_offset_l1", wp_offset[1][i][j] ,partition);
        }
      }
      else
      {
        len += u_1 ("chroma_weight_flag_l1", 0, partition);
      }
    }
  }
  return len;
}
  
/********************************************************************************************
 ********************************************************************************************
 *
 * Local Support Functions
 *
 ********************************************************************************************
 ********************************************************************************************/



// StW Note: This function is a hack.  It would be cleaner if the encoder maintains
// the picture type in the given format.  Note further that I have yet to understand
// why the encoder needs to know whether a picture is predicted from one or more
// reference pictures.

/*!
 ************************************************************************
 * \brief
 *    Selects picture type and codes it to symbol
 ************************************************************************
 */
int get_picture_type()
{

  // set this value to zero for transmission without signaling 
  // that the whole picture has the same slice type
  int same_slicetype_for_whole_frame = 5;

  switch (img->type)
  {
  case I_SLICE:
    return 2 + same_slicetype_for_whole_frame;
    break;
  case P_SLICE:
    return 0 + same_slicetype_for_whole_frame;
    break;
  case B_SLICE:
    return 1 + same_slicetype_for_whole_frame;
    break;
  case SP_SLICE:
    return 3 + same_slicetype_for_whole_frame;
    break;
  default:
    error("Picture Type not supported!",1);
    break;
  }
   
  return 0;
}



/*!
 *****************************************************************************
 *
 * \brief 
 *    int Partition_BC_Header () write the Partition type B, C header
 *
 * \return
 *    Number of bits used by the partition header
 *
 * \par Parameters
 *    PartNo: Partition Number to which the header should be written
 *
 * \par Side effects
 *    Partition header as per VCEG-N72r2 is written into the appropriate 
 *    partition bit buffer
 *
 * \par Limitations/Shortcomings/Tweaks
 *    The current code does not support the change of picture parameters within
 *    one coded sequence, hence there is only one parameter set necessary.  This
 *    is hard coded to zero.
 *
 * \date
 *    October 24, 2001
 *
 * \author
 *    Stephan Wenger   stewe@cs.tu-berlin.de
 *****************************************************************************/


int Partition_BC_Header(int PartNo)
{
  DataPartition *partition = &((img->currentSlice)->partArr[PartNo]);
  SyntaxElement symbol, *sym = &symbol;

  int len = 0;

  assert (input->of_mode == PAR_OF_RTP);
  assert (PartNo > 0 && PartNo < img->currentSlice->max_part_nr);

  sym->type = SE_HEADER;         // This will be true for all symbols generated here
  sym->mapping = ue_linfo;       // Mapping rule: Simple code number to len/info
  sym->value2  = 0;

	//ZL 
	//changed according to the g050r1
	SYMTRACESTRING("RTP-PH: Slice ID");
  sym->value1 = img->current_slice_nr;
  len += writeSyntaxElement_UVLC (sym, partition);

	if(active_pps->redundant_pic_cnt_present_flag)
  {
  SYMTRACESTRING("RTP-PH: Picture ID");
  sym->value1 = img->currentSlice->picture_id;
  len += writeSyntaxElement_UVLC (sym, partition);
  }


  return len;
}
