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
 * \file macroblock.c
 *
 * \brief
 *    Process one macroblock
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Inge Lille-Langøy               <inge.lille-langoy@telenor.com>
 *    - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *    - Jani Lainema                    <jani.lainema@nokia.com>
 *    - Sebastian Purreiter             <sebastian.purreiter@mch.siemens.de>
 *    - Detlev Marpe                    <marpe@hhi.de>
 *    - Thomas Wedi                     <wedi@tnt.uni-hannover.de>
 *    - Ragip Kurceren                  <ragip.kurceren@nokia.com>
 *************************************************************************************
 */
#include "contributors.h"

#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "elements.h"
#include "macroblock.h"
#include "refbuf.h"
#include "fmo.h"
#include "vlc.h"
#include "image.h"
#include "mb_access.h"
#include "ratectl.h"              // head file for rate control
#include "cabac.h"

//Rate control
int predict_error,dq;
extern int DELTA_QP,DELTA_QP2;
extern int QP,QP2;

 /*!
 ************************************************************************
 * \brief
 *    updates the coordinates for the next macroblock to be processed
 *
 * \param mb_addr
 *    MB address in scan order
 ************************************************************************
 */

void set_MB_parameters (int mb_addr)
{
  img->current_mb_nr = mb_addr;
  
  get_mb_block_pos(mb_addr, &img->mb_x, &img->mb_y);
  
  img->block_x = img->mb_x << 2;
  img->block_y = img->mb_y << 2;

  img->pix_x   = img->block_x << 2;
  img->pix_y   = img->block_y << 2;

  img->opix_x   = img->pix_x;

  if (img->MbaffFrameFlag)
  {
    if (img->mb_data[mb_addr].mb_field)
    {

      imgY_org  = (mb_addr % 2) ? imgY_org_bot  : imgY_org_top;
      imgUV_org = (mb_addr % 2) ? imgUV_org_bot : imgUV_org_top;
      img->opix_y   = (img->mb_y >> 1 ) << 4;
    }
    else
    {
      imgY_org  = imgY_org_frm;
      imgUV_org = imgUV_org_frm;
      img->opix_y   = img->block_y << 2;
    }
  }
  else
  {
    img->opix_y   = img->block_y << 2;
  }

  img->pix_c_x = img->pix_x >> 1;
  img->pix_c_y = img->pix_y >> 1;

  img->opix_c_x = img->opix_x >> 1;
  img->opix_c_y = img->opix_y >> 1;

//  printf ("set_MB_parameters: mb %d,  mb_x %d,  mb_y %d\n", mb_addr, img->mb_x, img->mb_y);
}


int clip1a(int a)
{
  return ((a)>255?255:((a)<0?0:(a)));
}

/*!
 ************************************************************************
 * \brief
 *    updates the coordinates and statistics parameter for the
 *    next macroblock
 ************************************************************************
 */
void proceed2nextMacroblock()
{
#if TRACE
  int i;
  int use_bitstream_backing = (input->slice_mode == FIXED_RATE || input->slice_mode == CALLBACK);
#endif
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];
  int*        bitCount = currMB->bitcounter;

#if TRACE

  if (p_trace)
  {
    fprintf(p_trace, "\n*********** Pic: %i (I/P) MB: %i Slice: %i **********\n\n", frame_no, img->current_mb_nr, img->current_slice_nr);
    if(use_bitstream_backing)
      fprintf(p_trace, "\n*********** Pic: %i (I/P) MB: %i Slice: %i **********\n\n", frame_no, img->current_mb_nr, img->current_slice_nr);
   // Write out the tracestring for each symbol
    for (i=0; i<currMB->currSEnr; i++)
      trace2out(&(img->MB_SyntaxElements[i]));
  }
#endif

  // Update the statistics
  stat->bit_use_mb_type    [img->type]  += bitCount[BITS_MB_MODE];
  stat->bit_use_coeffY     [img->type]  += bitCount[BITS_COEFF_Y_MB] ;
  stat->tmp_bit_use_cbp    [img->type]  += bitCount[BITS_CBP_MB];
  stat->bit_use_coeffC     [img->type]  += bitCount[BITS_COEFF_UV_MB];
  stat->bit_use_delta_quant[img->type]  += bitCount[BITS_DELTA_QUANT_MB];

  ++stat->mode_use[img->type][currMB->mb_type];
  stat->bit_use_mode[img->type][currMB->mb_type]+= bitCount[BITS_INTER_MB];

  // Statistics
  if ((img->type == P_SLICE)||(img->type==SP_SLICE) )
  {
    ++stat->quant0;
    stat->quant1 += currMB->qp;      // to find average quant for inter frames
  }
}

/*!
 ************************************************************************
 * \brief
 *    initializes the current macroblock
 ************************************************************************
 */
void start_macroblock(int mb_addr, int mb_field)
{
  int i,j,k,l;
  int use_bitstream_backing = (input->slice_mode == FIXED_RATE || input->slice_mode == CALLBACK);
  Macroblock *currMB = &img->mb_data[mb_addr];
  Slice *curr_slice = img->currentSlice;
  DataPartition *dataPart;
  Bitstream *currStream;
  EncodingEnvironmentPtr eep;

  currMB->mb_field = mb_field;
  
  enc_picture->mb_field[mb_addr] = mb_field;

  set_MB_parameters (mb_addr);

  if(use_bitstream_backing)
  {
    // Keep the current state of the bitstreams
    if(!img->cod_counter)
      for (i=0; i<curr_slice->max_part_nr; i++)
      {
        dataPart = &(curr_slice->partArr[i]);
        currStream = dataPart->bitstream;
        currStream->stored_bits_to_go   = currStream->bits_to_go;
        currStream->stored_byte_pos   = currStream->byte_pos;
        currStream->stored_byte_buf   = currStream->byte_buf;

        if (input->symbol_mode ==CABAC)
        {
          eep = &(dataPart->ee_cabac);
          eep->ElowS            = eep->Elow;
          eep->ErangeS           = eep->Erange;
          eep->EbufferS         = eep->Ebuffer;
          eep->Ebits_to_goS     = eep->Ebits_to_go;
          eep->Ebits_to_followS = eep->Ebits_to_follow;
          eep->EcodestrmS       = eep->Ecodestrm;
          eep->Ecodestrm_lenS   = eep->Ecodestrm_len;
          eep->CS               = eep->C;
          eep->BS               = eep->B;
          eep->ES               = eep->E;
        }
      }
  }

  // Save the slice number of this macroblock. When the macroblock below
  // is coded it will use this to decide if prediction for above is possible
  currMB->slice_nr = img->current_slice_nr;

  // Initialize delta qp change from last macroblock. Feature may be used for future rate control
  // Rate control
  currMB->qpsp       = img->qpsp;
  if(input->RCEnable)
  {
    if (img->current_mb_nr==0)
    {
      currMB->prev_qp = img->qp;
      currMB->prev_delta_qp = 0;
    }
    else
    {    
      currMB->prev_qp = img->mb_data[img->current_mb_nr-1].qp;
      currMB->prev_delta_qp = img->mb_data[img->current_mb_nr-1].delta_qp;
    }
    /*frame layer rate control*/
    if(input->basicunit==img->Frame_Total_Number_MB)
    {
      currMB->delta_qp = 0;
      currMB->qp       = img->qp;
    }
/*basic unit layer rate control*/
    else
    {
/*each I or B frame has only one QP*/
     if((img->type==I_SLICE)||(img->type==B_SLICE))
     {
       currMB->delta_qp = 0;
       currMB->qp       = img->qp;
     }
     else if(img->type==P_SLICE)
     {

       if (!img->write_macroblock) //write macroblock
       {
         if (!currMB->mb_field)  //frame macroblock
         {
           if (img->current_mb_nr == 0) //first macroblock
           {
// Initialize delta qp change from last macroblock. Feature may be used for future rate control
             currMB->delta_qp = 0;
             currMB->qp       = img->qp;
             DELTA_QP = DELTA_QP2 = currMB->delta_qp;
             QP = QP2 = currMB->qp;
           }
           else
           {
             if (!((input->MbInterlace) && img->bot_MB)) //top macroblock
             {
               if (img->mb_data[img->current_mb_nr-1].prev_cbp == 1)
               {
                 currMB->delta_qp = 0;
                 currMB->qp       = img->qp;
               }
               else
               {
                 currMB->qp = img->mb_data[img->current_mb_nr-1].prev_qp;
                 currMB->delta_qp = currMB->qp - img->mb_data[img->current_mb_nr-1].qp;
                 img->qp = currMB->qp;
               }
               DELTA_QP = DELTA_QP2 = currMB->delta_qp;
               QP = QP2 = currMB->qp;
             }
             else //bottom macroblock
             {
// Initialize delta qp change from last macroblock. Feature may be used for future rate control
               currMB->delta_qp = 0;
               currMB->qp       = img->qp;       // needed in loop filter (even if constant QP is used)
             }
           }
         }
         else  //field macroblock
         {
           if (!img->bot_MB) //top macroblock 
           {
             currMB->delta_qp = DELTA_QP2;
             currMB->qp   = img->qp    = QP2;
           }
           else//bottom macroblock
           {
             currMB->qp = img->qp;
             currMB->delta_qp = 0;
           }
           
         }
         
       }
       else 
       {
         if (!img->bot_MB) //write top macroblock
         {
           if (img->write_macroblock_frame)
           {
             currMB->delta_qp = DELTA_QP;
             img->qp = currMB->qp = QP;
           }
           else
           {
             currMB->delta_qp = DELTA_QP2;
             img->qp = currMB->qp = QP2;
           }
         }
         else //write bottom macroblock
         {
           currMB->delta_qp = 0;
           currMB->qp = img->qp;
         }
       }

       /*compute the quantization parameter for each basic unit of P frame*/

       if(!((input->MbInterlace)&&img->bot_MB))
       {
         if(!currMB->mb_field)
         {
           
           if((img->NumberofCodedMacroBlocks>0)\
             &&(img->NumberofCodedMacroBlocks%img->BasicUnit==0))
           {
             
             /*frame coding*/
             if(active_sps->frame_mbs_only_flag)
             {
               updateRCModel();
               img->BasicUnitQP=updateQuantizationParameter(img->TopFieldFlag);
             }
             /*adaptive field/frame coding*/
             else if((input->PicInterlace==ADAPTIVE_CODING)&&(!input->MbInterlace)&&(img->IFLAG==0))
             {
               updateRCModel();
               img->BasicUnitQP=updateQuantizationParameter(img->TopFieldFlag);
             }
             /*field coding*/
             else if((input->PicInterlace==FIELD_CODING)&&(!input->MbInterlace)&&(img->IFLAG==0))
             {
               updateRCModel();
               img->BasicUnitQP=updateQuantizationParameter(img->TopFieldFlag);
             }
             /*mb adaptive f/f coding, field coding*/
             else if((input->MbInterlace)&&(img->IFLAG==0)&&(img->FieldControl==1))
             {
               updateRCModel();
               img->BasicUnitQP=updateQuantizationParameter(img->TopFieldFlag);
             }
             /*mb adaptive f/f coding, frame coding*/
             else if((input->MbInterlace)&&(img->IFLAG==0)&&(img->FieldControl==0))
             {
               updateRCModel();
               img->BasicUnitQP=updateQuantizationParameter(img->TopFieldFlag);
             } 
             
             
           }
           
           
           if(img->current_mb_nr==0)
             img->BasicUnitQP=img->qp;

           currMB->predict_qp=img->BasicUnitQP;
           
           if(currMB->predict_qp>currMB->qp+25)
             currMB->predict_qp=currMB->qp+25;
           else if(currMB->predict_qp<currMB->qp-26)
             currMB->predict_qp=currMB->qp-26; 
           
           currMB->prev_qp = currMB->predict_qp;
           
           dq = currMB->delta_qp + currMB->predict_qp-currMB->qp;
           if(dq < -26) 
           {
             dq = -26;
             predict_error = dq-currMB->delta_qp;
             img->qp = img->qp+predict_error;
             currMB->delta_qp = -26;
           }
           else if(dq > 25)
           {
             dq = 25;
             predict_error = dq - currMB->delta_qp;
             img->qp = img->qp + predict_error;
             currMB->delta_qp = 25;
           }
           else
           {
             currMB->delta_qp = dq;
             predict_error=currMB->predict_qp-currMB->qp;
             img->qp = currMB->predict_qp;
           }
           currMB->qp =  img->qp;
           if (input->MbInterlace)
           {
             DELTA_QP = DELTA_QP2 = currMB->delta_qp;
             QP = QP2     = currMB->qp;
             DELTA_QP2 = currMB->delta_qp;
           }
           currMB->predict_error=predict_error;
         }
         else
           predict_error=currMB->predict_error;
       }
       else
         currMB->prev_qp=img->qp;
       }
    }   
  }
  else
  {
    Slice* currSlice = img->currentSlice;
  	
    int prev_mb = FmoGetPreviousMBNr(img->current_mb_nr);
    if (prev_mb>-1)
    {
      currMB->prev_qp = img->mb_data[prev_mb].qp;
      currMB->prev_delta_qp = img->mb_data[prev_mb].delta_qp;
    }
    else
    {
      currMB->prev_qp = currSlice->qp;
      currMB->prev_delta_qp = 0;
    }

    currMB->qp       = currSlice->qp ;
  
    currMB->delta_qp = currMB->qp - currMB->prev_qp;
    DELTA_QP = DELTA_QP2 = currMB->delta_qp;
    QP = QP2 = currMB->qp;
    
  }
  // Initialize counter for MB symbols
  currMB->currSEnr=0;

  // loop filter parameter
  if (active_pps->deblocking_filter_control_present_flag)
  {
    currMB->LFDisableIdc    = img->LFDisableIdc;
    currMB->LFAlphaC0Offset = img->LFAlphaC0Offset;
    currMB->LFBetaOffset    = img->LFBetaOffset;
  }
  else
  {
    currMB->LFDisableIdc    = 0;
    currMB->LFAlphaC0Offset = 0;
    currMB->LFBetaOffset    = 0;
  }

  // If MB is next to a slice boundary, mark neighboring blocks unavailable for prediction
  CheckAvailabilityOfNeighbors();

	if (input->symbol_mode == CABAC)
		CheckAvailabilityOfNeighborsCABAC();
	
  // Reset vectors before doing motion search in motion_search().

  for (l=0; l<2; l++)
  {
    for (j=0; j < BLOCK_MULTIPLE; j++)
      for (i=0; i < BLOCK_MULTIPLE; i++)
        for (k=0; k < 2; k++)
          enc_picture->mv[l][img->block_x+i][img->block_y+j][k]=0;
  }

  //initialize reference index 
  for (j=0; j < BLOCK_MULTIPLE; j++)
  {
    for (i=0; i < BLOCK_MULTIPLE; i++)
      for (l=0; l<2; l++)
      {
        enc_picture->ref_idx[l][img->block_x+i][img->block_y + j] =-1;
        enc_picture->ref_pic_id[l][img->block_x+i][img->block_y+j] = -1;
      }
    }
  
  // Reset syntax element entries in MB struct
  currMB->mb_type   = 0;
  currMB->cbp_blk   = 0;
  currMB->cbp       = 0;

  for (l=0; l < 2; l++)
    for (j=0; j < BLOCK_MULTIPLE; j++)
      for (i=0; i < BLOCK_MULTIPLE; i++)
        for (k=0; k < 2; k++)
          currMB->mvd[l][j][i][k] = 0;
 
  currMB->cbp_bits   = 0;
  currMB->c_ipred_mode = DC_PRED_8; //GB

  for (i=0; i < (BLOCK_MULTIPLE*BLOCK_MULTIPLE); i++)
    currMB->intra_pred_modes[i] = DC_PRED;

  //initialize the whole MB as INTRA coded
  //Blocks ar set to notINTRA in write_one_macroblock
  if (input->UseConstrainedIntraPred)
  {
    img->intra_block[img->current_mb_nr] = 1;
  }

  // store filtering parameters for this MB; For now, we are using the
  // same offset throughout the sequence
  currMB->lf_disable = img->LFDisableIdc;
  currMB->lf_alpha_c0_offset = img->LFAlphaC0Offset;
  currMB->lf_beta_offset = img->LFBetaOffset;
  
  
  // Initialize bitcounters for this macroblock
  if(img->current_mb_nr == 0) // No slice header to account for
  {
    currMB->bitcounter[BITS_HEADER] = 0;
  }
  else if (currMB->slice_nr == img->mb_data[img->current_mb_nr-1].slice_nr) // current MB belongs to the
  // same slice as the last MB
  {
    currMB->bitcounter[BITS_HEADER] = 0;
  }

  currMB->bitcounter[BITS_MB_MODE] = 0;
  currMB->bitcounter[BITS_COEFF_Y_MB] = 0;
  currMB->bitcounter[BITS_INTER_MB] = 0;
  currMB->bitcounter[BITS_CBP_MB] = 0;
  currMB->bitcounter[BITS_DELTA_QUANT_MB] = 0;
  currMB->bitcounter[BITS_COEFF_UV_MB] = 0;

#ifdef _FAST_FULL_ME_
  if(!input->FMEnable)
    ResetFastFullIntegerSearch ();
#endif
}

/*!
 ************************************************************************
 * \brief
 *    terminates processing of the current macroblock depending
 *    on the chosen slice mode
 ************************************************************************
 */
void terminate_macroblock(Boolean *end_of_slice, Boolean *recode_macroblock)
{
  int i;
  Slice *currSlice = img->currentSlice;
  Macroblock    *currMB    = &img->mb_data[img->current_mb_nr];
  SyntaxElement *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
  int *partMap = assignSE2partition[input->partition_mode];
  DataPartition *dataPart;
  Bitstream *currStream;
  int rlc_bits=0;
  EncodingEnvironmentPtr eep;
  int use_bitstream_backing = (input->slice_mode == FIXED_RATE || input->slice_mode == CALLBACK);
  int new_slice;
  static int skip = FALSE;

	 
  // if previous mb in the same slice group has different slice number as the current, it's the
  // the start of new slice
  new_slice=0;
  if ( (img->current_mb_nr==0) || (FmoGetPreviousMBNr(img->current_mb_nr)<0) )
    new_slice=1;
  else if( img->mb_data[FmoGetPreviousMBNr(img->current_mb_nr)].slice_nr != img->current_slice_nr )
    new_slice=1;
	
  
  *recode_macroblock=FALSE;

  switch(input->slice_mode)
  {
  case NO_SLICES:
    currSlice->num_mb++;
    *recode_macroblock = FALSE;
    if ((currSlice->num_mb) == img->total_number_mb) // maximum number of MBs reached
      *end_of_slice = TRUE;

    // if it's end of current slice group, slice ends too
    *end_of_slice |= (img->current_mb_nr == FmoGetLastCodedMBOfSliceGroup (FmoMB2SliceGroup (img->current_mb_nr)));
		
    break;
  case FIXED_MB:
    // For slice mode one, check if a new slice boundary follows
    currSlice->num_mb++;
    *recode_macroblock = FALSE;
    //! Check end-of-slice group condition first
    *end_of_slice = (img->current_mb_nr == FmoGetLastCodedMBOfSliceGroup (FmoMB2SliceGroup (img->current_mb_nr)));
    //! Now check maximum # of MBs in slice
    *end_of_slice |= (currSlice->num_mb >= input->slice_argument);
		
    break;

    // For slice modes two and three, check if coding of this macroblock
    // resulted in too many bits for this slice. If so, indicate slice
    // boundary before this macroblock and code the macroblock again
  case FIXED_RATE:
     // in case of skip MBs check if there is a slice boundary
     // only for UVLC (img->cod_counter is always 0 in case of CABAC)
     if(img->cod_counter)
     {
       // write out the skip MBs to know how many bits we need for the RLC
       currSE->value1 = img->cod_counter;
       currSE->value2 = 0;
       currSE->mapping = ue_linfo;
       currSE->type = SE_MBTYPE;
       dataPart = &(currSlice->partArr[partMap[currSE->type]]);

       dataPart->writeSyntaxElement(  currSE, dataPart);
       rlc_bits=currSE->len;

       currStream = dataPart->bitstream;
       // save the bitstream as it would be if we write the skip MBs
       currStream->bits_to_go_skip  = currStream->bits_to_go;
       currStream->byte_pos_skip    = currStream->byte_pos;
       currStream->byte_buf_skip    = currStream->byte_buf;
       // restore the bitstream
       currStream->bits_to_go = currStream->stored_bits_to_go;
       currStream->byte_pos = currStream->stored_byte_pos;
       currStream->byte_buf = currStream->stored_byte_buf;
       skip = TRUE;
     }
     //! Check if the last coded macroblock fits into the size of the slice
     //! But only if this is not the first macroblock of this slice
     if (!new_slice)
     {
       if(slice_too_big(rlc_bits))
       {
         *recode_macroblock = TRUE;
         *end_of_slice = TRUE;
       }
       else if(!img->cod_counter)
         skip = FALSE;
     }
     // maximum number of MBs
		 
     // check if current slice group is finished
     if ((*recode_macroblock == FALSE) && (img->current_mb_nr == FmoGetLastCodedMBOfSliceGroup (FmoMB2SliceGroup (img->current_mb_nr)))) 
     {
       *end_of_slice = TRUE;
       if(!img->cod_counter)
         skip = FALSE;
     }
   
     //! (first MB OR first MB in a slice) AND bigger that maximum size of slice
     if (new_slice && slice_too_big(rlc_bits))
     {
       *end_of_slice = TRUE;
       if(!img->cod_counter)
         skip = FALSE;
     }
     if (!*recode_macroblock)
       currSlice->num_mb++;
     break;

  case  CALLBACK:
    if (img->current_mb_nr > 0 && !new_slice)
    {
      if (currSlice->slice_too_big(rlc_bits))
      {
        *recode_macroblock = TRUE;
        *end_of_slice = TRUE;
      }
    }
		
    if ( (*recode_macroblock == FALSE) && (img->current_mb_nr == FmoGetLastCodedMBOfSliceGroup (FmoMB2SliceGroup (img->current_mb_nr)))) 
      *end_of_slice = TRUE;
    break;

  default:
    snprintf(errortext, ET_SIZE, "Slice Mode %d not supported", input->slice_mode);
    error(errortext, 600);
  }

  if(*recode_macroblock == TRUE)
  {
    // Restore everything
    for (i=0; i<currSlice->max_part_nr; i++)
    {
      dataPart = &(currSlice->partArr[i]);
      currStream = dataPart->bitstream;
      currStream->bits_to_go = currStream->stored_bits_to_go;
      currStream->byte_pos  = currStream->stored_byte_pos;
      currStream->byte_buf  = currStream->stored_byte_buf;
      if (input->symbol_mode == CABAC)
      {
        eep = &(dataPart->ee_cabac);
        eep->Elow            = eep->ElowS;
        eep->Erange           = eep->ErangeS;
        eep->Ebuffer         = eep->EbufferS;
        eep->Ebits_to_go     = eep->Ebits_to_goS;
        eep->Ebits_to_follow = eep->Ebits_to_followS;
        eep->Ecodestrm       = eep->EcodestrmS;
        eep->Ecodestrm_len   = eep->Ecodestrm_lenS;
        eep->C               = eep->CS;
        eep->B               = eep->BS;
        eep->E               = eep->ES;       
      }
    }
  }

  if(*end_of_slice == TRUE  && skip == TRUE) //! TO 4.11.2001 Skip MBs at the end of this slice
  { 
    //! only for Slice Mode 2 or 3
    // If we still have to write the skip, let's do it!
    if(img->cod_counter && *recode_macroblock == TRUE) //! MB that did not fit in this slice
    { 
      // If recoding is true and we have had skip, 
      // we have to reduce the counter in case of recoding
      img->cod_counter--;
      if(img->cod_counter)
      {
        currSE->value1 = img->cod_counter;
        currSE->value2 = 0;
        currSE->mapping = ue_linfo;
        currSE->type = SE_MBTYPE;
        dataPart = &(currSlice->partArr[partMap[currSE->type]]);
        dataPart->writeSyntaxElement(  currSE, dataPart);
        rlc_bits=currSE->len;
        currMB->bitcounter[BITS_MB_MODE]+=rlc_bits;
        img->cod_counter = 0;
      }
    }
    else //! MB that did not fit in this slice anymore is not a Skip MB
    {
      dataPart = &(currSlice->partArr[partMap[SE_MBTYPE]]);       
      currStream = dataPart->bitstream;
        // update the bitstream
      currStream->bits_to_go = currStream->bits_to_go_skip;
      currStream->byte_pos  = currStream->byte_pos_skip;
      currStream->byte_buf  = currStream->byte_buf_skip;

      // update the statistics
      img->cod_counter = 0;
      skip = FALSE;
    }
  }
  
  //! TO 4.11.2001 Skip MBs at the end of this slice for Slice Mode 0 or 1
  if(*end_of_slice == TRUE && img->cod_counter && !use_bitstream_backing)
  {
    currSE->value1 = img->cod_counter;
    currSE->value2 = 0;
    currSE->mapping = ue_linfo;
    currSE->type = SE_MBTYPE;
    dataPart = &(currSlice->partArr[partMap[currSE->type]]);
    dataPart->writeSyntaxElement(  currSE, dataPart);
    rlc_bits=currSE->len;
    currMB->bitcounter[BITS_MB_MODE]+=rlc_bits;
    img->cod_counter = 0;
  }
}

/*!
 *****************************************************************************
 *
 * \brief 
 *    For Slice Mode 2: Checks if one partition of one slice exceeds the 
 *    allowed size
 * 
 * \return
 *    FALSE if all Partitions of this slice are smaller than the allowed size
 *    TRUE is at least one Partition exceeds the limit
 *
 * \par Side effects
 *    none
 *
 * \date
 *    4 November 2001
 *
 * \author
 *    Tobias Oelbaum      drehvial@gmx.net
 *****************************************************************************/
 
 int slice_too_big(int rlc_bits)
 {
   Slice *currSlice = img->currentSlice;
   DataPartition *dataPart;
   Bitstream *currStream;
   EncodingEnvironmentPtr eep;
   int i;
   int size_in_bytes;
  
   //! UVLC
   if (input->symbol_mode == UVLC)
   {
     for (i=0; i<currSlice->max_part_nr; i++)
     {
       dataPart = &(currSlice->partArr[i]);
       currStream = dataPart->bitstream;
       size_in_bytes = currStream->byte_pos /*- currStream->tmp_byte_pos*/;

       if (currStream->bits_to_go < 8)
         size_in_bytes++;
       if (currStream->bits_to_go < rlc_bits)
         size_in_bytes++;
       if(size_in_bytes > input->slice_argument)
         return TRUE;
     }
   }
    
   //! CABAC
   if (input->symbol_mode ==CABAC)
   {
     for (i=0; i<currSlice->max_part_nr; i++)
     {
        dataPart= &(currSlice->partArr[i]);
        eep = &(dataPart->ee_cabac);
      
       if( arienco_bits_written(eep) > (input->slice_argument*8))
          return TRUE;
     }
   }
   return FALSE;
 }

/*!
 ************************************************************************
 * \brief
 *    Predict one component of a 4x4 Luma block
 ************************************************************************
 */
void
OneComponentLumaPrediction4x4 (int*   mpred,      //  --> array of prediction values (row by row)
                               int    pic_pix_x,  // <--  absolute horizontal coordinate of 4x4 block
                               int    pic_pix_y,  // <--  absolute vertical   coordinate of 4x4 block
                               int*   mv,         // <--  motion vector
                               int    ref,        // <--  reference frame 
                               StorablePicture **list)
{
  pel_t** ref_pic;
  int     pix_add = 4;
  int     j0      = (pic_pix_y << 2) + mv[1], j1=j0+pix_add, j2=j1+pix_add, j3=j2+pix_add;
  int     i0      = (pic_pix_x << 2) + mv[0], i1=i0+pix_add, i2=i1+pix_add, i3=i2+pix_add;
  
  pel_t (*get_pel) (pel_t**, int, int, int, int) = UMVPelY_14;

  int img_width =list[ref]->size_x;
  int img_height=list[ref]->size_y;

  ref_pic   = list[ref]->imgY_ups;
  
  *mpred++ = get_pel (ref_pic, j0, i0, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j0, i1, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j0, i2, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j0, i3, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j1, i0, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j1, i1, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j1, i2, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j1, i3, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j2, i0, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j2, i1, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j2, i2, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j2, i3, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j3, i0, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j3, i1, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j3, i2, img_height, img_width);
  *mpred++ = get_pel (ref_pic, j3, i3, img_height, img_width);

}

/*!
 ************************************************************************
 * \brief
 *    copy foward/backward prediction values of one component of a 4x4 Luma block
 ************************************************************************
 */

void
copyblock4x4 (int*   mpred,      //  --> array of prediction values (row by row)
              int block[BLOCK_SIZE][BLOCK_SIZE])        
{
  *mpred++ = block[0][0];
  *mpred++ = block[1][0];
  *mpred++ = block[2][0];
  *mpred++ = block[3][0];
  *mpred++ = block[0][1];
  *mpred++ = block[1][1];
  *mpred++ = block[2][1];
  *mpred++ = block[3][1];
  *mpred++ = block[0][2];
  *mpred++ = block[1][2];
  *mpred++ = block[2][2];
  *mpred++ = block[3][2];
  *mpred++ = block[0][3];
  *mpred++ = block[1][3];
  *mpred++ = block[2][3];
  *mpred++ = block[3][3];
}

/*!
 ************************************************************************
 * \brief
 *    Predict one 4x4 Luma block
 ************************************************************************
 */
void
LumaPrediction4x4 (int  block_x,    // <--  relative horizontal block coordinate of 4x4 block
                   int  block_y,    // <--  relative vertical   block coordinate of 4x4 block
                   int  p_dir,      // <--  prediction direction (0=forward, 1=backward, 2=bidir)
                   int  fw_mode,    // <--  forward  prediction mode (1-7, 0=DIRECT if bw_mode=0)
                   int  bw_mode,    // <--  backward prediction mode (1-7, 0=DIRECT if fw_mode=0)
                   int  fw_ref_idx, // <--  reference frame for forward prediction (-1: Intra4x4 pred. with fw_mode)
                   int  bw_ref_idx  )    
{
  static int fw_pred[16];
  static int bw_pred[16];

  int  i, j;
  int  block_x4  = block_x+4;
  int  block_y4  = block_y+4;
  int  pic_opix_x = img->opix_x + block_x;
  int  pic_opix_y = img->opix_y + block_y;
  int  bx        = block_x >> 2;
  int  by        = block_y >> 2;
  int* fpred     = fw_pred;
  int* bpred     = bw_pred;
//  int  direct    = (fw_mode == 0 && bw_mode == 0 && (img->type == B_SLICE));
//  int  skipped   = (fw_mode == 0 && bw_mode == 0 && (img->type != B_SLICE));

//  int  apply_weights = ( (input->WeightedPrediction && (img->type == P_SLICE || img->type == SP_SLICE)) ||
//                         (input->WeightedBiprediction && (img->type ==B_SLICE)));  
  int  apply_weights = ( (active_pps->weighted_pred_flag && (img->type== P_SLICE || img->type == SP_SLICE)) ||
                         (active_pps->weighted_bipred_idc && (img->type== B_SLICE)));  

  
  int  list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  if ((p_dir==0)||(p_dir==2))
  {
    OneComponentLumaPrediction4x4 (fw_pred, pic_opix_x, pic_opix_y, img->all_mv[bx][by][LIST_0][fw_ref_idx][fw_mode], fw_ref_idx, listX[0+list_offset]);   
  }

  if ((p_dir==1)||(p_dir==2))
  { 
    OneComponentLumaPrediction4x4 (bw_pred, pic_opix_x, pic_opix_y, img->all_mv[bx][by][LIST_1][bw_ref_idx][bw_mode], bw_ref_idx, listX[1+list_offset]);   
  }

  if (apply_weights)
  {

    if (p_dir==2)
    {
      for   (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  
          img->mpr[i][j] = clip1a(((wbp_weight[0][fw_ref_idx][bw_ref_idx][0] * *fpred++ + 
                                    wbp_weight[1][fw_ref_idx][bw_ref_idx][0] * *bpred++ + 
                                    2*wp_luma_round) >> (luma_log_weight_denom + 1)) + 
                                    ((wp_offset[0][fw_ref_idx][0] + wp_offset[1][bw_ref_idx][0] + 1)>>1)); 
    }
    else if (p_dir==0)
    {
      for   (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  
          img->mpr[i][j] = clip1a(((wp_weight[0][fw_ref_idx][0] * *fpred++  + wp_luma_round) >> luma_log_weight_denom) +
          + wp_offset[0][fw_ref_idx][0] );
    }
    else // p_dir==1
    {
      for   (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  
          img->mpr[i][j] = clip1a(((wp_weight[1][bw_ref_idx][0] * *bpred++  + wp_luma_round) >> luma_log_weight_denom) +
          wp_offset[1][bw_ref_idx][0] );
    }


  }
  else
  {
    if (p_dir==2)
    {
      for   (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  
          img->mpr[i][j] = (*fpred++ + *bpred++ + 1) / 2; 
    }
    else if (p_dir==0)
    {
      for   (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  img->mpr[i][j] = *fpred++;
    }
    else // p_dir==1
    {
      for   (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  img->mpr[i][j] = *bpred++;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Residual Coding of an 8x8 Luma block (not for intra)
 ************************************************************************
 */
int                                       //  ==> coefficient cost
LumaResidualCoding8x8 (int  *cbp,         //  --> cbp (updated according to processed 8x8 luminance block)
                       int  *cbp_blk,     //  --> block cbp (updated according to processed 8x8 luminance block)
                       int  block8x8,     // <--  block number of 8x8 block
                       int  p_dir,        // <--  prediction direction
                       int  fw_mode,      // <--  forward  prediction mode (1-7, 0=DIRECT)
                       int  bw_mode,      // <--  backward prediction mode (1-7, 0=DIRECT)
                       int  fw_refframe,  // <--  reference frame for forward prediction
                       int  bw_refframe   // <--  reference frame for backward prediction
                       )
{
  int    block_y, block_x, pic_pix_y, pic_pix_x, i, j, nonzero, cbp_blk_mask;
  int    coeff_cost = 0;
  int    mb_y       = (block8x8 / 2) << 3;
  int    mb_x       = (block8x8 % 2) << 3;
  int    cbp_mask   = 1 << block8x8;
  int    bxx, byy;                   // indexing curr_blk
  int    scrFlag = 0;                // 0=noSCR, 1=strongSCR, 2=jmSCR
  int    skipped    = (fw_mode == 0 && bw_mode == 0 && (img->type != B_SLICE));

  if (img->type==B_SLICE)
    scrFlag = 1;

  //===== loop over 4x4 blocks =====
  for (byy=0, block_y=mb_y; block_y<mb_y+8; byy+=4, block_y+=4)
  {
    pic_pix_y = img->opix_y + block_y;

    for (bxx=0, block_x=mb_x; block_x<mb_x+8; bxx+=4, block_x+=4)
    {
      pic_pix_x = img->opix_x + block_x;

      cbp_blk_mask = (block_x>>2) + block_y;

      //===== prediction of 4x4 block =====
      LumaPrediction4x4 (block_x, block_y, p_dir, fw_mode, bw_mode, fw_refframe, bw_refframe);

      //===== get displaced frame difference ======                
      for (j=0; j<4; j++)
      for (i=0; i<4; i++)
      {
        img->m7[i][j] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];
      }

      //===== DCT, Quantization, inverse Quantization, IDCT, Reconstruction =====      
      if (img->NoResidueDirect != 1 && !skipped  )
      {
        //===== DCT, Quantization, inverse Quantization, IDCT, Reconstruction =====
        if (img->type!=SP_SLICE)  nonzero = dct_luma   (block_x, block_y, &coeff_cost, 0);
        else                      nonzero = dct_luma_sp(block_x, block_y, &coeff_cost);
        if (nonzero)
        {
          (*cbp_blk) |= 1 << cbp_blk_mask;  // one bit for every 4x4 block
          (*cbp)     |= cbp_mask;           // one bit for the 4x4 blocks of an 8x8 block
        }
      }
    }
  }

  /*
  The purpose of the action below is to prevent that single or 'expensive' coefficients are coded.
  With 4x4 transform there is larger chance that a single coefficient in a 8x8 or 16x16 block may be nonzero.
  A single small (level=1) coefficient in a 8x8 block will cost: 3 or more bits for the coefficient,
  4 bits for EOBs for the 4x4 blocks,possibly also more bits for CBP.  Hence the total 'cost' of that single
  coefficient will typically be 10-12 bits which in a RD consideration is too much to justify the distortion improvement.
  The action below is to watch such 'single' coefficients and set the reconstructed block equal to the prediction according
  to a given criterium.  The action is taken only for inter luma blocks.

  Notice that this is a pure encoder issue and hence does not have any implication on the standard.
  coeff_cost is a parameter set in dct_luma() and accumulated for each 8x8 block.  If level=1 for a coefficient,
  coeff_cost is increased by a number depending on RUN for that coefficient.The numbers are (see also dct_luma()): 3,2,2,1,1,1,0,0,...
  when RUN equals 0,1,2,3,4,5,6, etc.
  If level >1 coeff_cost is increased by 9 (or any number above 3). The threshold is set to 3. This means for example:
  1: If there is one coefficient with (RUN,level)=(0,1) in a 8x8 block this coefficient is discarded.
  2: If there are two coefficients with (RUN,level)=(1,1) and (4,1) the coefficients are also discarded
  sum_cnt_nonz is the accumulation of coeff_cost over a whole macro block.  If sum_cnt_nonz is 5 or less for the whole MB,
  all nonzero coefficients are discarded for the MB and the reconstructed block is set equal to the prediction.
  */

  if (img->NoResidueDirect != 1 && !skipped && coeff_cost <= _LUMA_COEFF_COST_)
  {
    coeff_cost  = 0;
    (*cbp)     &=  (63 - cbp_mask);
    (*cbp_blk) &= ~(51 << (4*block8x8-2*(block8x8%2)));

    for (i=mb_x; i<mb_x+8; i++)
    for (j=mb_y; j<mb_y+8; j++)
    {
      enc_picture->imgY[img->pix_y+j][img->pix_x+i] = img->mpr[i][j];
    }
    if (img->type==SP_SLICE)
    {
      for (i=mb_x; i < mb_x+BLOCK_SIZE*2; i+=BLOCK_SIZE)
        for (j=mb_y; j < mb_y+BLOCK_SIZE*2; j+=BLOCK_SIZE)
          copyblock_sp(i,j);
    }
  }

  return coeff_cost;
}


/*!
 ************************************************************************
 * \brief
 *    Set mode parameters and reference frames for an 8x8 block
 ************************************************************************
 */
void
SetModesAndRefframe (int b8, int* p_dir, int* fw_mode, int* bw_mode, int* fw_ref, int* bw_ref)
{
  Macroblock* currMB = &img->mb_data[img->current_mb_nr];
  int         j      = 2*(b8/2);
  int         i      = 2*(b8%2);

  *fw_mode = *bw_mode = *fw_ref = *bw_ref = -1;

  *p_dir  = currMB->b8pdir[b8];

  if (img->type!=B_SLICE)
  {
    *fw_ref = enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j];
    *bw_ref = 0;
    *fw_mode  = currMB->b8mode[b8];
    *bw_mode  = 0;
  }
  else
  {
    if (currMB->b8pdir[b8]==-1)
    {
      *fw_ref   = -1;
      *bw_ref   = -1;
      *fw_mode  =  0;
      *bw_mode  =  0;
    }
    else if (currMB->b8pdir[b8]==0)
    {
      *fw_ref   = enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j];
      *bw_ref   = 0;
      *fw_mode  = currMB->b8mode[b8];
      *bw_mode  = 0;
    }
    else if (currMB->b8pdir[b8]==1)
    {
      *fw_ref   = 0;
      *bw_ref   = enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j];
      *fw_mode  = 0;
      *bw_mode  = currMB->b8mode[b8];
    }
    else
    {
      *fw_ref   = enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j];
      *bw_ref   = enc_picture->ref_idx[LIST_1][img->block_x+i][img->block_y+j];
      *fw_mode  = currMB->b8mode[b8];
      *bw_mode  = currMB->b8mode[b8];
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    Residual Coding of a Luma macroblock (not for intra)
 ************************************************************************
 */
void
LumaResidualCoding ()
{
  int i,j,block8x8,b8_x,b8_y;
  int p_dir, fw_mode, bw_mode, refframe;
  int sum_cnt_nonz;
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  currMB->cbp     = 0 ;
  currMB->cbp_blk = 0 ;
  sum_cnt_nonz    = 0 ;

  for (block8x8=0; block8x8<4; block8x8++)
  {
    int bw_ref;
    SetModesAndRefframe (block8x8, &p_dir, &fw_mode, &bw_mode, &refframe, &bw_ref);

    sum_cnt_nonz += LumaResidualCoding8x8 (&(currMB->cbp), &(currMB->cbp_blk), block8x8,
                                           p_dir, fw_mode, bw_mode, refframe, bw_ref);
  }

  if (sum_cnt_nonz <= 5 )
  {
     currMB->cbp     &= 0xfffff0 ;
     currMB->cbp_blk &= 0xff0000 ;
     for (i=0; i < MB_BLOCK_SIZE; i++)
     {
       for (j=0; j < MB_BLOCK_SIZE; j++)
       {
         enc_picture->imgY[img->pix_y+j][img->pix_x+i]=img->mpr[i][j];
       }
     }
     if (img->type==SP_SLICE)
     {
       for(block8x8=0;block8x8<4;block8x8++)
       {
         b8_x=(block8x8&1)<<3;
         b8_y=(block8x8&2)<<2;
         for (i=0;i<8;i+=4)
           for (j=0;j<8;j+=4)
             copyblock_sp(b8_x+i,b8_y+j);
       }
     }
   }
}



/*!
 ************************************************************************
 * \brief
 *    Predict one component of a chroma 4x4 block
 ************************************************************************
 */
void
OneComponentChromaPrediction4x4 (int*      mpred,      //!< array to store prediction values
                                 int       block_c_x,  //!< horizontal pixel coordinate of 4x4 block
                                 int       block_c_y,  //!< vertical   pixel coordinate of 4x4 block
                                 int****** mv,         //!< motion vector array
                                 int       list_idx,   //!< reference picture list
                                 int       ref,        //!< reference index
                                 int       blocktype,  //!< block type
                                 int       uv)         //!< chroma component
{
  int     i, j, ii, jj, ii0, jj0, ii1, jj1, if0, if1, jf0, jf1;
  int*    mvb;
  pel_t** refimage;
  int     f1        = 8 , f2=f1-1, f3=f1*f1, f4=f3>>1;
  int     s1        = 3;
  int     list_offset;
  int     max_y_cr;

  StorablePicture **list;

  int curr_mb_field = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field));
  // find out the correct list offsets
  if (curr_mb_field)
  {
    if(img->current_mb_nr%2)
      list_offset = 4; // top field mb
    else
      list_offset = 2; // bottom field mb
    max_y_cr = img->height_cr/2-1;
  }
  else
  {
    list_offset = 0;  // no mb aff or frame mb
    max_y_cr = img->height_cr-1;
  }


  list      = listX[list_idx + list_offset];

  refimage  = list[ref]->imgUV[uv];
  
  for (j=0; j<4; j++)
  for (i=0; i<4; i++)
  {
    //ref_idx   = enc_picture->ref_idx[list_idx][(img->pix_c_x + block_c_x)>>1][(img->pix_c_y + block_c_y)>>1];  

    mvb  = mv [(i+block_c_x)>>1][(j+block_c_y)>>1][list_idx][ref][blocktype];

    ii   = ((i+block_c_x+img->opix_c_x)<<s1) + mvb[0];
    jj   = ((j+block_c_y+img->opix_c_y)<<s1) + mvb[1];
    
    jj  += list[ref]->chroma_vector_adjustment;

    ii0  = max (0, min (img->width_cr -1, ii>>s1     ));
    jj0  = max (0, min (max_y_cr, jj>>s1     ));
    ii1  = max (0, min (img->width_cr -1, (ii>>s1)+1));
    jj1  = max (0, min (max_y_cr, (jj>>s1)+1));

    if1  = (ii&f2);  if0 = f1-if1;
    jf1  = (jj&f2);  jf0 = f1-jf1;

    *mpred++ = (if0 * jf0 * refimage[jj0][ii0] +
                if1 * jf0 * refimage[jj0][ii1] +
                if0 * jf1 * refimage[jj1][ii0] +
                if1 * jf1 * refimage[jj1][ii1] + f4) >> 6;
  }
}


/*!
 ************************************************************************
 * \brief
 *    Predict an intra chroma 4x4 block
 ************************************************************************
 */
void IntraChromaPrediction4x4 (int  uv,       // <-- colour component
                               int  block_x,  // <-- relative horizontal block coordinate of 4x4 block
                               int  block_y)  // <-- relative vertical   block coordinate of 4x4 block
{
  int mode = img->mb_data[img->current_mb_nr].c_ipred_mode;
  int i, j;

  //===== prediction =====
  for (j=block_y; j<block_y+4; j++)
  for (i=block_x; i<block_x+4; i++)
  {
    img->mpr[i][j] = img->mprr_c[uv][mode][i][j];
  }
}



/*!
 ************************************************************************
 * \brief
 *    Predict one chroma 4x4 block
 ************************************************************************
 */
void
ChromaPrediction4x4 (int  uv,           // <-- colour component
                     int  block_x,      // <-- relative horizontal block coordinate of 4x4 block
                     int  block_y,      // <-- relative vertical   block coordinate of 4x4 block
                     int  p_dir,        // <-- prediction direction
                     int  fw_mode,      // <-- forward  prediction mode (1-7, 0=DIRECT if bw_mode=0)
                     int  bw_mode,      // <-- backward prediction mode (1-7, 0=DIRECT if fw_mode=0)
                     int  fw_ref_idx,   // <-- reference frame for forward prediction (if (<0) -> intra prediction)
                     int  bw_ref_idx)   // <-- reference frame for backward prediction 
{
  static int fw_pred[16];
  static int bw_pred[16];

  int  i, j;
  int  block_x4   = block_x+4;
  int  block_y4   = block_y+4;
  int* fpred      = fw_pred;
  int* bpred      = bw_pred;
  int****** mv_array = img->all_mv;

  //int apply_weights = ( (input->WeightedPrediction && (img->type == P_SLICE||img->type == SP_SLICE)) ||
//                     (input->WeightedBiprediction && (img->type == B_SLICE)));
  int  apply_weights = ( (active_pps->weighted_pred_flag && (img->type == P_SLICE || img->type == SP_SLICE)) ||
                         (active_pps->weighted_bipred_idc && (img->type == B_SLICE)));  


  //===== INTRA PREDICTION =====
  if (p_dir==-1)
  {
    IntraChromaPrediction4x4 (uv, block_x, block_y);
    return;
  }
  
  //===== INTER PREDICTION =====
  if ((p_dir==0) || (p_dir==2))
  {
    OneComponentChromaPrediction4x4 (fw_pred, block_x, block_y, mv_array, LIST_0, fw_ref_idx, fw_mode, uv);
  }
  if ((p_dir==1) || (p_dir==2))
  {
    OneComponentChromaPrediction4x4 (bw_pred, block_x, block_y, mv_array, LIST_1, bw_ref_idx, bw_mode, uv);
  }

  if (apply_weights)
  {
    if (p_dir==2)
    {
      for (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  
            img->mpr[i][j] =  clip1a(((wbp_weight[0][fw_ref_idx][bw_ref_idx][uv+1] * *fpred++ + wbp_weight[1][fw_ref_idx][bw_ref_idx][uv+1] * *bpred++ 
                  + 2*wp_chroma_round) >> (chroma_log_weight_denom + 1)) + ((wp_offset[0][fw_ref_idx][uv+1] + wp_offset[1][bw_ref_idx][uv+1] + 1)>>1) );
    }
    else if (p_dir==0)
    {
      for (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  
           img->mpr[i][j] = clip1a(((wp_weight[0][fw_ref_idx][uv+1] * *fpred++ + wp_chroma_round) >> chroma_log_weight_denom) +  wp_offset[0][fw_ref_idx][uv+1]);
    }
    else // (p_dir==1)
    {
      for (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  
          img->mpr[i][j] = clip1a(((wp_weight[1][bw_ref_idx][uv+1] * *bpred++ + wp_chroma_round) >> chroma_log_weight_denom) + wp_offset[1][bw_ref_idx][uv+1]);
    }
  }
  else
  {
    if (p_dir==2)
    {
      for (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  
          img->mpr[i][j] = (*fpred++ + *bpred++ + 1) / 2; 
    }
    else if (p_dir==0)
    {
      for (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  img->mpr[i][j] = *fpred++;
    }
    else // (p_dir==1)
    {
      for (j=block_y; j<block_y4; j++)
        for (i=block_x; i<block_x4; i++)  img->mpr[i][j] = *bpred++;
    }
  }
}




/*!
 ************************************************************************
 * \brief
 *    Chroma residual coding for an macroblock
 ************************************************************************
 */
void ChromaResidualCoding (int* cr_cbp)
{
  int   uv, block8, block_y, block_x, j, i;
  int   p_dir, fw_mode, bw_mode, refframe;
  int   skipped = (img->mb_data[img->current_mb_nr].mb_type == 0 && (img->type == P_SLICE || img->type == SP_SLICE));
  int   bw_ref;

  for (*cr_cbp=0, uv=0; uv<2; uv++)
  {
    //===== prediction of chrominance blocks ===d==
    block8 = 0;
    for (block_y=0; block_y<8; block_y+=4)
    for (block_x=0; block_x<8; block_x+=4, block8++)
    {
      SetModesAndRefframe (block8, &p_dir, &fw_mode, &bw_mode, &refframe, &bw_ref);

      ChromaPrediction4x4 (uv, block_x, block_y, p_dir, fw_mode, bw_mode, refframe, bw_ref);
    }

        // ==== set chroma residue to zero for skip Mode in SP frames 
    if (img->NoResidueDirect)
    {
      for (j=0; j<8; j++)
        for (i=0; i<8; i++)
        {
          enc_picture->imgUV[uv][img->pix_c_y+j][img->pix_c_x+i] = img->mpr[i][j];
        }
    }
    else
      if (skipped && img->type==SP_SLICE)
      {
        for (j=0; j<8; j++)
          for (i=0; i<8; i++)
          {
            img->m7[i][j] = 0;
          }
      }
      else
        if (skipped)
        {
          for (j=0; j<8; j++)
            for (i=0; i<8; i++)
            {
              enc_picture->imgUV[uv][img->pix_c_y+j][img->pix_c_x+i] = img->mpr[i][j];
            }
        }
        else
        {
          for (j=0; j<8; j++)
            for (i=0; i<8; i++)
            {
              img->m7[i][j] = imgUV_org[uv][img->opix_c_y+j][img->opix_c_x+i] - img->mpr[i][j];
            }
        }

    //===== DCT, Quantization, inverse Quantization, IDCT, and Reconstruction =====
    //===== Call function for skip mode in SP frames to properly process frame ====
    
    if (skipped && img->type==SP_SLICE)
    {
      *cr_cbp=dct_chroma_sp(uv,*cr_cbp);
    }
    else
    {
      if (!img->NoResidueDirect && !skipped)
      {
        if (img->type!=SP_SLICE || IS_INTRA (&img->mb_data[img->current_mb_nr]))
          *cr_cbp=dct_chroma   (uv,*cr_cbp);
        else
          *cr_cbp=dct_chroma_sp(uv,*cr_cbp);
      }
    }
  }
  
  //===== update currMB->cbp =====
  img->mb_data[img->current_mb_nr].cbp += ((*cr_cbp)<<4);  
}


/*!
 ************************************************************************
 * \brief
 *    Predict an intra chroma 8x8 block
 ************************************************************************
 */
void IntraChromaPrediction8x8 (int *mb_up, int *mb_left, int*mb_up_left)
{

  Macroblock *currMB = &img->mb_data[img->current_mb_nr];
  int     s, s0, s1, s2, s3, i, j, k;
  pel_t** image;
  int     block_x, block_y;
  int     mb_nr             = img->current_mb_nr;
  int     mb_available_up;
  int     mb_available_left[2];
  int     mb_available_up_left;
  int     ih,iv;
  int     ib,ic,iaa;
  int     uv;
  int     hline[8], vline[9];
  int     mode;
  int     best_mode = DC_PRED_8;         //just an initilaization here, should always be overwritten
  int     cost;
  int     min_cost;
  int     diff[16];
  PixelPos up;       //!< pixel position p(0,-1)
  PixelPos left[9];  //!< pixel positions p(-1, -1..8)


  for (i=0;i<9;i++)
  {
    getNeighbour(mb_nr, -1 ,  i-1 , 0, &left[i]);
  }
  
  getNeighbour(mb_nr, 0     ,  -1 , 0, &up);


  mb_available_up                             = up.available;
  mb_available_up_left                        = left[0].available;
  mb_available_left[0] = mb_available_left[1] = left[1].available;

  if(input->UseConstrainedIntraPred)
  {
    mb_available_up      = up.available ? img->intra_block[up.mb_addr] : 0;
    for (i=1, mb_available_left[0]=1; i<5;i++)
      mb_available_left[0]  &= left[i].available ? img->intra_block[left[i].mb_addr]: 0;
    for (i=5, mb_available_left[1]=1; i<9;i++)
      mb_available_left[1]  &= left[i].available ? img->intra_block[left[i].mb_addr]: 0;
    mb_available_up_left = left[0].available ? img->intra_block[left[0].mb_addr]: 0;
  }

  if (mb_up)
    *mb_up = mb_available_up;
  if (mb_left)
    *mb_left = mb_available_left[0] && mb_available_left[1];
  if( mb_up_left )
    *mb_up_left = mb_available_up_left;

  // compute all chroma intra prediction modes for both U and V
  for (uv=0; uv<2; uv++)
  {
    image = enc_picture->imgUV[uv];

    // DC prediction
    for (block_y=0; block_y<8; block_y+=4)
    for (block_x=0; block_x<8; block_x+=4)
    {
      s=128;
      s0=s1=s2=s3=0;
      //===== get prediction value =====
      switch ((block_y>>1) + (block_x>>2))
      {
      case 0:  //===== TOP LEFT =====
        if      (mb_available_up)       for (i=0;i<4;i++)  s0 += image[up.pos_y][up.pos_x + i];
        if      (mb_available_left[0])  for (i=1;i<5;i++)  s2 += image[left[i].pos_y][left[i].pos_x];
        if      (mb_available_up && mb_available_left[0])  s  = (s0+s2+4) >> 3;
        else if (mb_available_up)                          s  = (s0   +2) >> 2;
        else if (mb_available_left[0])                     s  = (s2   +2) >> 2;
        break;
      case 1: //===== TOP RIGHT =====
        if      (mb_available_up)       for (i=4;i<8;i++)  s1 += image[up.pos_y][up.pos_x + i];
        else if (mb_available_left[0])  for (i=1;i<5;i++)  s2 += image[left[i].pos_y][left[i].pos_x];
        if      (mb_available_up)                          s  = (s1   +2) >> 2;
        else if (mb_available_left[0])                     s  = (s2   +2) >> 2;
        break;
      case 2: //===== BOTTOM LEFT =====
        if      (mb_available_left[1])  for (i=5;i<9;i++)  s3 += image[left[i].pos_y][left[i].pos_x];
        else if (mb_available_up)       for (i=0;i<4;i++)  s0 += image[up.pos_y][up.pos_x + i];
        if      (mb_available_left[1])                     s  = (s3   +2) >> 2;
        else if (mb_available_up)                          s  = (s0   +2) >> 2;
        break;
      case 3: //===== BOTTOM RIGHT =====
        if      (mb_available_up)       for (i=4;i<8;i++)  s1 += image[up.pos_y][up.pos_x + i];
        if      (mb_available_left[1])  for (i=5;i<9;i++)  s3 += image[left[i].pos_y][left[i].pos_x];
        if      (mb_available_up && mb_available_left[1])  s  = (s1+s3+4) >> 3;
        else if (mb_available_up)                          s  = (s1   +2) >> 2;
        else if (mb_available_left[1])                     s  = (s3   +2) >> 2;
        break;
      }


      //===== prediction =====
      for (j=block_y; j<block_y+4; j++)
      for (i=block_x; i<block_x+4; i++)
      {
        img->mprr_c[uv][DC_PRED_8][i][j] = s;
      }
    }

    // vertical prediction
    if (mb_available_up)
    {
      for (i=0; i<8; i++)
        hline[i] = image[up.pos_y][up.pos_x + i];
      for (i=0; i<8; i++)
      for (j=0; j<8; j++)
        img->mprr_c[uv][VERT_PRED_8][i][j] = hline[i];
    }

    // horizontal prediction 
    if (mb_available_left[0] && mb_available_left[1])
    {
      for (i=1; i<9; i++)
        vline[i] = image[left[i].pos_y][left[i].pos_x];
      for (i=0; i<8; i++)
      for (j=0; j<8; j++)
        img->mprr_c[uv][HOR_PRED_8][i][j] = vline[j+1]; 
    }

    // plane prediction 
    if (mb_available_left[0] && mb_available_left[1] && mb_available_up && mb_available_up_left)
    {
      ih = 4*(hline[7] - image[left[0].pos_y][left[0].pos_x]);
      iv = 4*(vline[7+1] - image[left[0].pos_y][left[0].pos_x]);
      for (i=1;i<4;i++)
      {
        ih += i*(hline[3+i] - hline[3-i]);
        iv += i*(vline[3+i+1] - vline[3-i+1]);
      }
      ib=(17*ih+16)>>5;
      ic=(17*iv+16)>>5;

      iaa=16*(hline[7]+vline[7+1]);
      for (j=0; j<8; j++)
      for (i=0; i<8; i++)
        img->mprr_c[uv][PLANE_8][i][j]=max(0,min(255,(iaa+(i-3)*ib +(j-3)*ic + 16)/32));// store plane prediction
    }
  }

  if (!input->rdopt) // the rd-opt part does not work correctly (see encode_one_macroblock)
  {                       // since ipredmodes could be overwritten => encoder-decoder-mismatches
    // pick lowest cost prediction mode
    min_cost = 1<<20;
    for (i=0;i<8;i++)
    {
      getNeighbour(mb_nr, 0 ,  i , 0, &left[i]);
    }
    for (mode=DC_PRED_8; mode<=PLANE_8; mode++)
    {
      if ((mode==VERT_PRED_8 && !mb_available_up) ||
          (mode==HOR_PRED_8 && (!mb_available_left[0] || !mb_available_left[1])) ||
          (mode==PLANE_8 && (!mb_available_left[0] || !mb_available_left[1] || !mb_available_up || !mb_available_up_left)))
        continue;

      cost = 0;
      for (uv=0; uv<2; uv++)
      {
        image = imgUV_org[uv];
        for (block_y=0; block_y<8; block_y+=4)
        for (block_x=0; block_x<8; block_x+=4)
        {
          for (k=0,j=block_y; j<block_y+4; j++)
          for (i=block_x; i<block_x+4; i++,k++)
          {
            diff[k] = image[left[j].pos_y][left[j].pos_x+i] - img->mprr_c[uv][mode][i][j];
          }
          cost += SATD(diff, input->hadamard);
        }
      }
      if (cost < min_cost)
      {
        best_mode = mode;
        min_cost = cost;
      }
    }

    currMB->c_ipred_mode = best_mode;
  }
 
}


/*!
 ************************************************************************
 * \brief
 *    Check if all reference frames for a macroblock are zero
 ************************************************************************
 */
int
ZeroRef (Macroblock* currMB)
{
  int i,j;

  for (j=0; j<4; j++)
  for (i=0; i<4; i++)
  {
    if (enc_picture->ref_idx[LIST_0][img->block_x+i][img->block_y+j]!=0)
    {
        return 0;
    }
  }
  return 1;
}


/*!
 ************************************************************************
 * \brief
 *    Converts macroblock type to coding value
 ************************************************************************
 */
int
MBType2Value (Macroblock* currMB)
{
  static const int dir1offset[3]    =  { 1,  2, 3};
  static const int dir2offset[3][3] = {{ 0,  4,  8},   // 1. block forward
                                       { 6,  2, 10},   // 1. block backward
                                       {12, 14, 16}};  // 1. block bi-directional

  int mbtype, pdir0, pdir1;

  if (img->type!=B_SLICE)
  {
    if      (currMB->mb_type==I4MB)     return (img->type==I_SLICE ? 0 : 6);
    else if (currMB->mb_type==I16MB)    return (img->type==I_SLICE ? 0 : 6) + img->i16offset;
    else if (currMB->mb_type==P8x8)
    {
      if (input->symbol_mode==UVLC && ZeroRef (currMB))  return 5;
      else                                               return 4;
    }
    else                                return currMB->mb_type;
  }
  else
  {
    mbtype = currMB->mb_type;
    pdir0  = currMB->b8pdir[0];
    pdir1  = currMB->b8pdir[3];

    if      (mbtype==0)       return 0;
    else if (mbtype==I4MB)    return 23;
    else if (mbtype==I16MB)   return 23 + img->i16offset;
    else if (mbtype==P8x8)    return 22;
    else if (mbtype==1)       return dir1offset[pdir0];
    else if (mbtype==2)       return 4 + dir2offset[pdir0][pdir1];
    else                      return 5 + dir2offset[pdir0][pdir1];
  }
}



/*!
 ************************************************************************
 * \brief
 *    Writes intra prediction modes for an 8x8 block
 ************************************************************************
 */
int writeIntra4x4Modes(int only_this_block)
{
  int i,j,bs_x,bs_y,ii,jj;
  int block8x8;
  int rate;
  int ipred_array[16],cont_array[16],ipred_number;
  Macroblock    *currMB     = &img->mb_data[img->current_mb_nr];
  SyntaxElement *currSE     = &img->MB_SyntaxElements[currMB->currSEnr];
  int           *bitCount   = currMB->bitcounter;
  Slice         *currSlice  = img->currentSlice;
  DataPartition *dataPart;
  const int     *partMap    = assignSE2partition[input->partition_mode];

  ipred_number=0;
  for(block8x8=0;block8x8<4;block8x8++)
  {
    if( currMB->b8mode[block8x8]==IBLOCK && (only_this_block<0||only_this_block==block8x8) )
    {
      bs_x=bs_y=4;
      ii=(bs_x>>2); // bug fix for solaris. mwi 
      jj=(bs_y>>2); // bug fix for solaris. mwi
      
      for(j=0;j<2;j+=jj)
      {
        for(i=0;i<2;i+=ii)
        {
          ipred_array[ipred_number]=currMB->intra_pred_modes[(block8x8<<2)|(j<<1)|i];
          cont_array[ipred_number]=(block8x8<<2)+(j<<1)+i;
          ipred_number++;
        }
      }
    }
  }
  rate=0;

  for(i=0;i<ipred_number;i++)
  {
    currMB->IntraChromaPredModeFlag = 1;
    currSE->context = cont_array[i];
    currSE->value1  = ipred_array[i];
    currSE->value2  = 0;

#if TRACE
    snprintf(currSE->tracestring, TRACESTRING_SIZE, "Intra mode     = %3d %d",currSE->value1,currSE->context);
#endif

    /*--- set symbol type and function pointers ---*/
    if (input->symbol_mode != UVLC)    currSE->writing = writeIntraPredMode_CABAC;
    currSE->type = SE_INTRAPREDMODE;

    /*--- choose data partition ---*/
    dataPart = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);   
    /*--- encode and update rate ---*/
    if (input->symbol_mode == UVLC)    writeSyntaxElement_Intra4x4PredictionMode(currSE, dataPart);
    else                               dataPart->writeSyntaxElement (currSE, dataPart);
    bitCount[BITS_COEFF_Y_MB]+=currSE->len;
    rate += currSE->len;
    currSE++;
    currMB->currSEnr++;
  }

  return rate;
}



/*!
 ************************************************************************
 * \brief
 *    Converts 8x8 block tyoe to coding value
 ************************************************************************
 */
int
B8Mode2Value (int b8mode, int b8pdir)
{
  static const int b8start[8] = {0,0,0,0, 1, 4, 5, 10};
  static const int b8inc  [8] = {0,0,0,0, 1, 2, 2, 1};

  if (img->type!=B_SLICE)
  {
    return (b8mode-4);
  }
  else
  {
    return b8start[b8mode] + b8inc[b8mode] * b8pdir;
  }
}



/*!
 ************************************************************************
 * \brief
 *    Codes macroblock header
 ************************************************************************
 */
int writeMBHeader (int rdopt)  // GB CHROMA !!!!!!!!
{
  int             i,j;
  int             mb_nr     = img->current_mb_nr;
  Macroblock*     currMB    = &img->mb_data[mb_nr];
  Macroblock*     prevMB    = mb_nr ? (&img->mb_data[mb_nr-1]) : NULL;
  SyntaxElement  *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
  int*            bitCount  = currMB->bitcounter;
  Slice*          currSlice = img->currentSlice;
  DataPartition*  dataPart;
  const int*      partMap   = assignSE2partition[input->partition_mode];
  int             no_bits   = 0;
  int             skip      = currMB->mb_type ? 0:((img->type == B_SLICE) ? !currMB->cbp:1);
  int             mb_type;
  int							prevMbSkipped = 0;
	int							mb_field_tmp;
  Macroblock			*topMB = NULL;
  
  int             WriteFrameFieldMBInHeader = 0;

  if (img->MbaffFrameFlag)
  {
    if (0==(mb_nr%2))
    {
      WriteFrameFieldMBInHeader = 1; // top field

      prevMbSkipped = 0;
    }
    else
    {
      if (prevMB->mb_type ? 0:((img->type == B_SLICE) ? !prevMB->cbp:1))
      {
        WriteFrameFieldMBInHeader = 1; // bottom, if top was skipped
      }

      topMB= &img->mb_data[img->current_mb_nr-1];
      if(!(img->type == B_SLICE))
        prevMbSkipped = (topMB->mb_type == 0);
      else 
        prevMbSkipped = (topMB->mb_type == 0 && topMB->cbp == 0);
    }
  }
  currMB->IntraChromaPredModeFlag = IS_INTRA(currMB);

  // choose the appropriate data partition
  dataPart = &(currSlice->partArr[partMap[SE_MBTYPE]]);
  
  //=====  BITS FOR MACROBLOCK MODE =====
  if(img->type == I_SLICE)//GB
  {
    // write mb_aff 
    if(img->MbaffFrameFlag && !skip) // check for copy mode, Krit
    {
      if(WriteFrameFieldMBInHeader)
      {
        currSE->value1 =  currMB->mb_field;
        currSE->value2 = 0;
        currSE->type   =  SE_MBTYPE;
        
        if (input->symbol_mode==UVLC)   currSE->mapping = ue_linfo;
        else                            currSE->writing = writeFieldModeInfo_CABAC;
          
#if TRACE
        snprintf(currSE->tracestring, TRACESTRING_SIZE, "Field mode = %3d",currMB->mb_field);
#endif
        if( input->symbol_mode==UVLC)
        {
          currSE->bitpattern = (currMB->mb_field ? 1 : 0);
          currSE->len = 1;
          writeSyntaxElement2Buf_Fixed(currSE, dataPart->bitstream);
        }
        else
        {
          dataPart->writeSyntaxElement(currSE, dataPart);
        }

        bitCount[BITS_MB_MODE] += currSE->len;
        no_bits                += currSE->len;
        currSE++;
        currMB->currSEnr++;
      }
    }
    
                        // write mb_type
    currSE->value1  = MBType2Value (currMB);
    currSE->value2  = 0;
    currSE->type    = SE_MBTYPE;
    
    if (input->symbol_mode == UVLC)  currSE->mapping = ue_linfo;
    else                             currSE->writing = writeMB_typeInfo_CABAC;
    
    dataPart->writeSyntaxElement( currSE, dataPart);
#if TRACE
    snprintf(currSE->tracestring, TRACESTRING_SIZE,   "MB mode(%2d,%2d) = %3d",img->mb_x, img->mb_y, currMB->mb_type);
#endif
    bitCount[BITS_MB_MODE] += currSE->len;
    no_bits                += currSE->len;
    currSE++;
    currMB->currSEnr++;
  }
  else if (input->symbol_mode == CABAC)//GB
  {
    
    if (img->MbaffFrameFlag && (img->current_mb_nr%2 == 0||prevMbSkipped))
    {
      mb_field_tmp = currMB->mb_field;
      currMB->mb_field = field_flag_inference();
      CheckAvailabilityOfNeighborsCABAC();
      currMB->mb_field = mb_field_tmp;
    }
    
    // write mb_skip_flag
    mb_type         = MBType2Value (currMB);
    currSE->value1  = mb_type;
    currSE->value2  = currMB->cbp;
    currSE->type    = SE_MBTYPE;
    currSE->writing = writeMB_skip_flagInfo_CABAC;
    dataPart->writeSyntaxElement( currSE, dataPart);
#if TRACE
    if (img->type == B_SLICE)  snprintf(currSE->tracestring, TRACESTRING_SIZE, "B_MB skipflag(%2d,%2d) = %3d",img->mb_x, img->mb_y, (mb_type!=0 ||currMB->cbp!=0));
    else                       snprintf(currSE->tracestring, TRACESTRING_SIZE, "MB skipflag(%2d,%2d,%d) = %3d",img->mb_x, img->mb_y, currSE->context,(mb_type!=0));
#endif
    bitCount[BITS_MB_MODE] += currSE->len;
    no_bits                += currSE->len;
    currSE++;
    currMB->currSEnr++;
    
    CheckAvailabilityOfNeighborsCABAC();
			
    // write mb_aff
    if(img->MbaffFrameFlag && !skip) // check for copy mode, Krit
    {
      if(WriteFrameFieldMBInHeader)
      {
        currSE->value1 = currMB->mb_field;
        currSE->value2 = 0;
        currSE->type   =  SE_MBTYPE;
        
        if (input->symbol_mode==UVLC)   currSE->mapping = ue_linfo;
        else                            currSE->writing = writeFieldModeInfo_CABAC;
        
        if( input->symbol_mode==UVLC)
        {
          currSE->bitpattern = (currMB->mb_field ? 1 : 0);
          currSE->len = 1;
          writeSyntaxElement2Buf_Fixed(currSE, dataPart->bitstream);
        }
        else
        {
          dataPart->writeSyntaxElement(currSE, dataPart);
        }
#if TRACE
        snprintf(currSE->tracestring, TRACESTRING_SIZE, "Field mode = %3d",currMB->mb_field);
#endif
        bitCount[BITS_MB_MODE] += currSE->len;
        no_bits                += currSE->len;
        currSE++;
        currMB->currSEnr++;
      }
    }
      
    // write mb_type
    if (currMB->mb_type != 0 || ((img->type == B_SLICE) && currMB->cbp != 0))
    {
      currSE->value1  = mb_type;
      currSE->value2  = 0;
      currSE->type    = SE_MBTYPE;
      currSE->writing = writeMB_typeInfo_CABAC;
      dataPart->writeSyntaxElement( currSE, dataPart);
#if TRACE
      if (img->type == B_SLICE)  snprintf(currSE->tracestring, TRACESTRING_SIZE, "B_MB mode(%2d,%2d) = %3d",img->mb_x, img->mb_y, currMB->mb_type);
      else                     snprintf(currSE->tracestring, TRACESTRING_SIZE,   "MB mode(%2d,%2d) = %3d",img->mb_x, img->mb_y,currMB->mb_type);
#endif
      bitCount[BITS_MB_MODE] += currSE->len;
      no_bits                += currSE->len;
      currSE++;
      currMB->currSEnr++;
    }
  }

  else if (currMB->mb_type != 0 || ((img->type == B_SLICE) && currMB->cbp != 0))
  {
    //===== Run Length Coding: Non-Skipped macorblock =====
    currSE->value1  = img->cod_counter;
    currSE->value2  = 0;
    currSE->mapping = ue_linfo;
    currSE->type    = SE_MBTYPE;
    dataPart->writeSyntaxElement( currSE, dataPart);
#if TRACE
    snprintf(currSE->tracestring, TRACESTRING_SIZE, "MB runlength = %3d",img->cod_counter);
#endif
    bitCount[BITS_MB_MODE] += currSE->len;
    no_bits                += currSE->len;
    currSE++;
    currMB->currSEnr++;
    
    // Reset cod counter
    img->cod_counter = 0;
    
    // write mb_aff
    if(img->MbaffFrameFlag && !skip) // check for copy mode, Krit
    {
      if(WriteFrameFieldMBInHeader)
      {
        currSE->value1 = currMB->mb_field;
        currSE->type   =  SE_MBTYPE;
        currSE->mapping = ue_linfo;
        
        //dataPart->writeSyntaxElement(currSE, dataPart);
        currSE->bitpattern = (currMB->mb_field ? 1 : 0);
        currSE->len = 1;
        writeSyntaxElement2Buf_Fixed(currSE, dataPart->bitstream);

#if TRACE
        snprintf(currSE->tracestring, TRACESTRING_SIZE, "Field mode = %3d",currMB->mb_field);
#endif
        bitCount[BITS_MB_MODE] += currSE->len;
        no_bits                += currSE->len;
        currSE++;
        currMB->currSEnr++;
      }
    }
    // Put out mb mode
    currSE->value1  = MBType2Value (currMB);
    
    if (img->type != B_SLICE)
    {
      currSE->value1--;
    }
    currSE->mapping = ue_linfo;
    currSE->type    = SE_MBTYPE;
    currSE->value2  = 0;
    
    dataPart->writeSyntaxElement( currSE, dataPart);
#if TRACE
    if (img->type == B_SLICE)   snprintf(currSE->tracestring, TRACESTRING_SIZE, "B_MB mode(%2d,%2d) = %3d",img->mb_x, img->mb_y, currMB->mb_type);
    else                      snprintf(currSE->tracestring, TRACESTRING_SIZE,   "MB mode(%2d,%2d) = %3d",img->mb_x, img->mb_y,currMB->mb_type);
#endif
    bitCount[BITS_MB_MODE] += currSE->len;
    no_bits                += currSE->len;
    currSE++;
    currMB->currSEnr++;
  }
  else
  {
    //Run Length Coding: Skipped macroblock
    img->cod_counter++;

    // CAVLC
    for (j=0; j < 6; j++)
      for (i=0; i < 4; i++)
        img->nz_coeff [img->current_mb_nr][i][j]=0;


    if(img->current_mb_nr == img->total_number_mb)
    {
      // Put out run
      currSE->value1  = img->cod_counter;
      currSE->value2  = 0;
      currSE->mapping = ue_linfo;
      currSE->type    = SE_MBTYPE;
      
      dataPart->writeSyntaxElement( currSE, dataPart);
#if TRACE
      snprintf(currSE->tracestring, TRACESTRING_SIZE, "MB runlength = %3d",img->cod_counter);
#endif
      bitCount[BITS_MB_MODE] += currSE->len;
      no_bits                += currSE->len;
      currSE++;
      currMB->currSEnr++;
      
      // Reset cod counter
      img->cod_counter = 0;
    }
  }
  
  //===== BITS FOR 8x8 SUB-PARTITION MODES =====
  if (IS_P8x8 (currMB))
  {
    dataPart = &(currSlice->partArr[partMap[SE_MBTYPE]]);
    
    for (i=0; i<4; i++)
    {
      if (input->symbol_mode==UVLC)   currSE->mapping = ue_linfo;
      else                            currSE->writing = writeB8_typeInfo_CABAC;

      currSE->value1  = B8Mode2Value (currMB->b8mode[i], currMB->b8pdir[i]);
      currSE->value2  = 0;
      currSE->type    = SE_MBTYPE;
      dataPart->writeSyntaxElement (currSE, dataPart);
#if TRACE
      snprintf(currSE->tracestring, TRACESTRING_SIZE, "8x8 mode/pdir(%2d) = %3d/%d",
        i,currMB->b8mode[i],currMB->b8pdir[i]);
#endif
      bitCount[BITS_MB_MODE]+= currSE->len;
      no_bits               += currSE->len;
      currSE++;
      currMB->currSEnr++;
    }
  }

 //===== BITS FOR INTRA PREDICTION MODES ====
  no_bits += writeIntra4x4Modes(-1);
  //===== BITS FOR CHROMA INTRA PREDICTION MODE ====
  if (currMB->IntraChromaPredModeFlag)
    no_bits += writeChromaIntraPredMode();
  else if(!rdopt) //GB CHROMA !!!!!
    currMB->c_ipred_mode = DC_PRED_8; //setting c_ipred_mode to default is not the right place here
                                      //resetting in rdopt.c (but where ??)
                                      //with cabac and bframes maybe it could crash without this default
                                      //since cabac needs the right neighborhood for the later MBs

  return no_bits;
}

void write_terminating_bit (short bit)
{
  DataPartition*          dataPart;
  const int*              partMap   = assignSE2partition[input->partition_mode];
  EncodingEnvironmentPtr  eep_dp;

  //--- write non-slice termination symbol if the macroblock is not the first one in its slice ---
  dataPart = &(img->currentSlice->partArr[partMap[SE_MBTYPE]]);
  dataPart->bitstream->write_flag = 1;
  eep_dp                          = &(dataPart->ee_cabac);
  
  biari_encode_symbol_final(eep_dp, bit); 
#if TRACE
  fprintf (p_trace, "      CABAC terminating bit = %d\n",bit);
#endif

}


/*!
 ************************************************************************
 * \brief
 *    Write chroma intra prediction mode.
 ************************************************************************
 */
int writeChromaIntraPredMode()
{
  Macroblock*     currMB    = &img->mb_data[img->current_mb_nr];
  SyntaxElement*  currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
  Slice*          currSlice = img->currentSlice;
  int*            bitCount  = currMB->bitcounter;
  const int*      partMap   = assignSE2partition[input->partition_mode];
  int             rate      = 0;
  DataPartition*  dataPart;

  //===== BITS FOR CHROMA INTRA PREDICTION MODES
  if (input->symbol_mode==UVLC)   currSE->mapping = ue_linfo;
  else                            currSE->writing = writeCIPredMode_CABAC;

  currSE->value1 = currMB->c_ipred_mode;
  currSE->value2 = 0;
  currSE->type = SE_INTRAPREDMODE;
  dataPart = &(currSlice->partArr[partMap[SE_INTRAPREDMODE]]);

  dataPart->writeSyntaxElement (currSE, dataPart);
  bitCount[BITS_COEFF_UV_MB] += currSE->len;
  rate                    += currSE->len;
#if TRACE
  snprintf(currSE->tracestring, TRACESTRING_SIZE, "Chroma intra pred mode");
#endif
  currSE++;
  currMB->currSEnr++;

  return rate;
}


/*!
************************************************************************
* \brief
*    Set global last_dquant according to macroblock delta qp
************************************************************************
*/

extern int last_dquant;

void set_last_dquant()
{
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  if ((IS_INTERMV (currMB)  || IS_INTRA (currMB)  ) ||
    ((img->type==B_SLICE)     && currMB->cbp != 0)  )
  {
    // non-skip
    last_dquant = currMB->delta_qp;
  }
  else
  {
    // skip
    last_dquant = 0;
  }
}


/*!
 ************************************************************************
 * \brief
 *    Passes the chosen syntax elements to the NAL
 ************************************************************************
 */
void write_one_macroblock (int eos_bit)
{
  Macroblock* currMB   = &img->mb_data[img->current_mb_nr];
  int*        bitCount = currMB->bitcounter;
  int i,j;

  extern int cabac_encoding;

  //===== init and update number of intra macroblocks =====
  if (img->current_mb_nr==0)
    intras=0;
  //if ((img->type==P_SLICE || img->type==SP_SLICE || (img->type==B_SLICE && img->nal_reference_idc>0)) && IS_INTRA(currMB))
  if (IS_INTRA(currMB))
    intras++;

  //--- write non-slice termination symbol if the macroblock is not the first one in its slice ---
  if (input->symbol_mode==CABAC && img->current_mb_nr!=img->currentSlice->start_mb_nr && eos_bit)
  {
    write_terminating_bit (0);
  }

  cabac_encoding = 1;

  //--- write header ---
  writeMBHeader (0); 

  //  Do nothing more if copy and inter mode
  if ((IS_INTERMV (currMB)  || IS_INTRA (currMB)  ) ||
      ((img->type==B_SLICE)     && currMB->cbp != 0)  )
  {
    writeMotionInfo2NAL  ();
    writeCBPandLumaCoeff ();
    writeChromaCoeff     ();
  }
  else
  { 
    for (j=0; j < 6; j++)
      for (i=0; i < 4; i++)
        img->nz_coeff [img->current_mb_nr][i][j]=0;  // CAVLC
  }

  set_last_dquant();

  //--- constrain intra prediction ---
  if(input->UseConstrainedIntraPred && (img->type==P_SLICE || img->type==B_SLICE))
  {
    if( !IS_NEWINTRA( currMB ) && currMB->mb_type!=I4MB )
    {
      img->intra_block[img->current_mb_nr] = 0;
    }
  }

  //--- set total bit-counter ---
  bitCount[BITS_TOTAL_MB] = bitCount[BITS_MB_MODE] + bitCount[BITS_COEFF_Y_MB]     + bitCount[BITS_INTER_MB]
                          + bitCount[BITS_CBP_MB]  + bitCount[BITS_DELTA_QUANT_MB] + bitCount[BITS_COEFF_UV_MB];

  //Rate control
  img->NumberofMBHeaderBits=bitCount[BITS_MB_MODE]   + bitCount[BITS_INTER_MB]
                           + bitCount[BITS_CBP_MB]  + bitCount[BITS_DELTA_QUANT_MB];
  img->NumberofMBTextureBits= bitCount[BITS_COEFF_Y_MB]+ bitCount[BITS_COEFF_UV_MB];
  img->NumberofTextureBits +=img->NumberofMBTextureBits;
  img->NumberofHeaderBits +=img->NumberofMBHeaderBits;
  /*basic unit layer rate control*/
  if(img->BasicUnit<img->Frame_Total_Number_MB)
  {
    img->NumberofBasicUnitHeaderBits +=img->NumberofMBHeaderBits;
    img->NumberofBasicUnitTextureBits +=img->NumberofMBTextureBits;
  }
  /*record the total number of MBs*/
  img->NumberofCodedMacroBlocks++;
  
  stat->bit_slice += bitCount[BITS_TOTAL_MB];

  cabac_encoding = 0;
}


/*!
 ************************************************************************
 * \brief
 *    Sets context for reference frame parameter
 ************************************************************************
 */
int BType2CtxRef (int btype)
{
  if (btype<4)   return 0;
  else           return 1;
}


/*!
 ************************************************************************
 * \brief
 *    Codes the reference frame
 ************************************************************************
 */
int writeReferenceFrame (int mode, int i, int j, int fwd_flag, int  ref)
{
  Macroblock*     currMB    = &img->mb_data[img->current_mb_nr];
  SyntaxElement*  currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
  Slice*          currSlice = img->currentSlice;
  int*            bitCount  = currMB->bitcounter;
  const int*      partMap   = assignSE2partition[input->partition_mode];
  int             rate      = 0;
  DataPartition*  dataPart;
  int             list_offset = ((img->MbaffFrameFlag)&&(currMB->mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;
  int             num_ref   = ( fwd_flag ? listXsize[LIST_0+list_offset]: listXsize[LIST_1+list_offset]);
  int             flag_mode = 0;

  if( num_ref == 1 )
  {
    return 0;
  }
  if ( num_ref == 2 )
  {
    flag_mode = 1;
  }

  currSE->value1 = ref;
  currSE->value2  = 0;
  currSE->type   = SE_REFFRAME;

  dataPart = &(currSlice->partArr[partMap[currSE->type]]);
  if (input->symbol_mode == UVLC)
  {
    if( flag_mode )
    {
      currSE->bitpattern = 1 - currSE->value1;
      currSE->len = 1;
      writeSyntaxElement2Buf_Fixed(currSE, dataPart->bitstream);
    }
    else
    {
      currSE->mapping = ue_linfo;
      dataPart->writeSyntaxElement (currSE, dataPart);
    }
  }
  else
  {
    currSE->context = BType2CtxRef (mode);
    img->subblock_x = i; // position used for context determination
    img->subblock_y = j; // position used for context determination
    currSE->writing = writeRefFrame_CABAC;
    currSE->value2 = (fwd_flag)? LIST_0:LIST_1;
    dataPart->writeSyntaxElement (currSE, dataPart);
  }

  bitCount[BITS_INTER_MB] += currSE->len;
  rate                    += currSE->len;
#if TRACE
  if (fwd_flag)
  {
    snprintf(currSE->tracestring, TRACESTRING_SIZE, "ref_idx_l0 = %d", currSE->value1);
  }
  else
  {
    snprintf(currSE->tracestring, TRACESTRING_SIZE, "ref_idx_l1 = %d", currSE->value1);
  }
#endif
  currSE++;
  currMB->currSEnr++;

  return rate;
}


/*!
 ************************************************************************
 * \brief
 *    Writes motion vectors of an 8x8 block
 ************************************************************************
 */
int writeMotionVector8x8 (int  i0,
                          int  j0,
                          int  i1,
                          int  j1,
                          int  refframe,
                          int  list_idx,
                          int  mv_mode)
{
  int            i, j, k, l, m;
  int            curr_mvd;
  DataPartition* dataPart;

  int            rate       = 0;
  int            step_h     = input->blc_size[mv_mode][0] >> 2;
  int            step_v     = input->blc_size[mv_mode][1] >> 2;
  Macroblock*    currMB     = &img->mb_data[img->current_mb_nr];
  SyntaxElement* currSE     = &img->MB_SyntaxElements[currMB->currSEnr];
  Slice*         currSlice  = img->currentSlice;
  int*           bitCount   = currMB->bitcounter;
  const int*     partMap    = assignSE2partition[input->partition_mode];
  int            refindex   = refframe;

  int******      all_mv     = img->all_mv;
  int******      pred_mv    = img->pred_mv;


  for (j=j0; j<j1; j+=step_v)
  for (i=i0; i<i1; i+=step_h)
  {
    for (k=0; k<2; k++) 
    {
      curr_mvd = all_mv[i][j][list_idx][refindex][mv_mode][k] - pred_mv[i][j][list_idx][refindex][mv_mode][k];


      //--- store (oversampled) mvd ---
      for (l=0; l < step_v; l++)
        for (m=0; m < step_h; m++)
          currMB->mvd[list_idx][j+l][i+m][k] = curr_mvd;

      currSE->value1 = curr_mvd;
      currSE->value2 = 0;
      currSE->type   = SE_MVD;
      if (input->symbol_mode == UVLC)
      {
        currSE->mapping = se_linfo;
      }
      else
      {
        img->subblock_x = i; // position used for context determination
        img->subblock_y = j; // position used for context determination
        currSE->value2  = 2*k+list_idx; // identifies the component and the direction; only used for context determination
        currSE->writing = writeMVD_CABAC;
      }  
      dataPart = &(currSlice->partArr[partMap[SE_MVD]]);
      dataPart->writeSyntaxElement (currSE, dataPart);
#if TRACE
      if (!list_idx)
      {
        snprintf(currSE->tracestring, TRACESTRING_SIZE, "mvd_l0 (%d) = %3d  (org_mv %3d pred_mv %3d)",k, curr_mvd, all_mv[i][j][list_idx][refindex][mv_mode][k], pred_mv[i][j][list_idx][refindex][mv_mode][k]);
      }
      else
      {
        
        snprintf(currSE->tracestring, TRACESTRING_SIZE, "mvd_l1 (%d) = %3d  (org_mv %3d pred_mv %3d)",k, curr_mvd, all_mv[i][j][list_idx][refindex][mv_mode][k], pred_mv[i][j][list_idx][refindex][mv_mode][k]);
      }

#endif
      bitCount[BITS_INTER_MB] += currSE->len;
      rate                    += currSE->len;
      currSE++;  
      currMB->currSEnr++;
    }
  }

  return rate;
}


/*!
 ************************************************************************
 * \brief
 *    Writes motion info
 ************************************************************************
 */
int writeMotionInfo2NAL ()
{
  int k, j0, i0, refframe;

  Macroblock*     currMB    = &img->mb_data[img->current_mb_nr];
  int             no_bits   = 0;
  int   bframe          = (img->type==B_SLICE);
  int   step_h0         = (input->blc_size[IS_P8x8(currMB) ? 4 : currMB->mb_type][0] >> 2);
  int   step_v0         = (input->blc_size[IS_P8x8(currMB) ? 4 : currMB->mb_type][1] >> 2);

  //=== If multiple ref. frames, write reference frame for the MB ===
  if (IS_INTERMV (currMB))
  {
    // if UVLC is turned on, a 8x8 macroblock with all ref=0 in a P-frame is signalled in macroblock mode
    if (!IS_P8x8 (currMB) || !ZeroRef (currMB) || input->symbol_mode==CABAC || bframe)
    {
      for (j0=0; j0<4; j0+=step_v0)
      for (i0=0; i0<4; i0+=step_h0)
      {
        k=j0+(i0/2);

        if ((currMB->b8pdir[k]==0 || currMB->b8pdir[k]==2) && currMB->b8mode[k]!=0)//has forward vector
        {
          no_bits += writeReferenceFrame (currMB->b8mode[k], i0, j0, 1, enc_picture->ref_idx[LIST_0][img->block_x+i0][img->block_y+j0]);
        }
      }
      for (j0=0; j0<4; j0+=step_v0)
      for (i0=0; i0<4; i0+=step_h0)
      {
        k=j0+(i0/2);
        if ((currMB->b8pdir[k]==1 || currMB->b8pdir[k]==2) && currMB->b8mode[k]!=0)//has backward vector
        {
          no_bits += writeReferenceFrame (currMB->b8mode[k], i0, j0, 0, enc_picture->ref_idx[LIST_1][img->block_x+i0][img->block_y+j0]);
        }
      }
    }
  }

  //===== write forward motion vectors =====
  if (IS_INTERMV (currMB))
  {
    for (j0=0; j0<4; j0+=step_v0)
    for (i0=0; i0<4; i0+=step_h0)
    {
      k=j0+(i0/2);
      if ((currMB->b8pdir[k]==0 || currMB->b8pdir[k]==2) && currMB->b8mode[k]!=0)//has forward vector
      {
        refframe  = enc_picture->ref_idx[LIST_0][img->block_x+i0][img->block_y+j0];
        no_bits  += writeMotionVector8x8 (i0, j0, i0+step_h0, j0+step_v0, refframe, LIST_0, currMB->b8mode[k]);
      }
    }
  }


  //===== write backward motion vectors =====
  if (IS_INTERMV (currMB) && bframe)
  {
    for (j0=0; j0<4; j0+=step_v0)
    for (i0=0; i0<4; i0+=step_h0)
    {
      k=j0+(i0/2);
      if ((currMB->b8pdir[k]==1 || currMB->b8pdir[k]==2) && currMB->b8mode[k]!=0)//has backward vector
      {
        refframe  = enc_picture->ref_idx[LIST_1][img->block_x+i0][img->block_y+j0];
        no_bits  += writeMotionVector8x8 (i0, j0, i0+step_h0, j0+step_v0, refframe, LIST_1, currMB->b8mode[k]);
      }
    }
  }
  return no_bits;
}



/*!
 ************************************************************************
 * \brief
 *    Writes chrominance coefficients
 ************************************************************************
 */
int writeChromaCoeff ()
{
  int             rate      = 0;
  Macroblock*     currMB    = &img->mb_data[img->current_mb_nr];
  SyntaxElement*  currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
  int*            bitCount  = currMB->bitcounter;
  Slice*          currSlice = img->currentSlice;
  const int*      partMap   = assignSE2partition[input->partition_mode];
  int             cbp       = currMB->cbp;
  DataPartition*  dataPart;

  int   level, run;
  int   i, j, k, uv, mb_x, mb_y, i1, ii, j1, jj;
  int   b8, b4, param;
  int*  ACLevel;
  int*  ACRun;
  int*  DCLevel;
  int*  DCRun;


  //=====
  //=====   D C - C O E F F I C I E N T S
  //=====
  if (cbp > 15)  // check if any chroma bits in coded block pattern is set
  {
    for (uv=0; uv < 2; uv++)
    {

      if (input->symbol_mode == UVLC)
      {
        param = uv;
        rate += writeCoeff4x4_CAVLC (CHROMA_DC, 0, 0, param);
          // CAVLC
      }
      else
      {

        DCLevel = img->cofDC[uv+1][0];
        DCRun   = img->cofDC[uv+1][1];

        level=1;
        for (k=0; k < 5 && level != 0; ++k)
        {
          level = currSE->value1 = DCLevel[k]; // level
          run   = currSE->value2 = DCRun  [k]; // run

          if (input->symbol_mode == UVLC)   currSE->mapping = levrun_linfo_c2x2;
          else                              currSE->writing = writeRunLevel_CABAC;

          currSE->context     = CHROMA_DC;
          currSE->type        = (IS_INTRA(currMB) ? SE_CHR_DC_INTRA : SE_CHR_DC_INTER);
          img->is_intra_block =  IS_INTRA(currMB);
          img->is_v_block     = uv;
    
          // choose the appropriate data partition
          dataPart = &(currSlice->partArr[partMap[currSE->type]]);    
          dataPart->writeSyntaxElement (currSE, dataPart);
          bitCount[BITS_COEFF_UV_MB] += currSE->len;
          rate                       += currSE->len;
#if TRACE
          snprintf(currSE->tracestring, TRACESTRING_SIZE, "2x2 DC Chroma %2d: level =%3d run =%2d",k, level, run);
#endif
          // proceed to next SE 
          currSE++;  
          currMB->currSEnr++;
        }
      }
    }
  }


  //=====
  //=====   A C - C O E F F I C I E N T S
  //=====
  uv=-1;   
  if (cbp >> 4 == 2) // check if chroma bits in coded block pattern = 10b
  {  
    for (mb_y=4; mb_y < 6; mb_y += 2)
    for (mb_x=0; mb_x < 4; mb_x += 2)
    {
      for (j=mb_y; j < mb_y+2; j++)
      {
        jj=j/2;
        j1=j-4;
        for (i=mb_x; i < mb_x+2; i++)
        {
          b8      = 4 + i/2;
          b4      = 2*(j/5)+ (i%2);

          if (input->symbol_mode == UVLC)
          {
            param = i << 4 | j;
            rate += writeCoeff4x4_CAVLC (CHROMA_AC, b8, b4, param);
            // CAVLC
          }
          else
          {

            ACLevel = img->cofAC[b8][b4][0];
            ACRun   = img->cofAC[b8][b4][1];

            ii=i/2;
            i1=i%2;
            level=1;
            uv++;

            img->subblock_y = b4/2;
            img->subblock_x = b4%2;

            for (k=0; k < 16 && level != 0; k++)
            {
              level = currSE->value1 = ACLevel[k]; // level
              run   = currSE->value2 = ACRun  [k]; // run

              if (input->symbol_mode == UVLC)   currSE->mapping = levrun_linfo_inter;
              else                              currSE->writing = writeRunLevel_CABAC;
            
              currSE->context     = CHROMA_AC;
              currSE->type        = (IS_INTRA(currMB) ? SE_CHR_AC_INTRA : SE_CHR_AC_INTER);
              img->is_intra_block =  IS_INTRA(currMB);
              img->is_v_block     = (uv>=4);

              // choose the appropriate data partition
              dataPart = &(currSlice->partArr[partMap[currSE->type]]); 
              dataPart->writeSyntaxElement (currSE, dataPart);
              bitCount[BITS_COEFF_UV_MB] += currSE->len;
              rate                       += currSE->len;
#if TRACE
              snprintf(currSE->tracestring, TRACESTRING_SIZE, "AC Chroma %2d: level =%3d run =%2d",k, level, run);
#endif

              // proceed to next SE 
              currSE++;  
              currMB->currSEnr++;
            }
          }
        }
      }
    }
  }

  return rate;
}



/*!
 ************************************************************************
 * \brief
 *    Writes Luma coeff of an 4x4 block
 ************************************************************************
 */
int writeLumaCoeff4x4_CABAC (int b8, int b4, int intra4x4mode)
{
  int             rate      = 0;
  Macroblock*     currMB    = &img->mb_data[img->current_mb_nr];
  SyntaxElement*  currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
  Slice*          currSlice = img->currentSlice;
  const int*      partMap   = assignSE2partition[input->partition_mode];
  int*            bitCount  = currMB->bitcounter;
  DataPartition*  dataPart;

  int   level, run;
  int   k;
  int*  ACLevel = img->cofAC[b8][b4][0];
  int*  ACRun   = img->cofAC[b8][b4][1];

  img->subblock_x = ((b8&0x1)==0)?(((b4&0x1)==0)?0:1):(((b4&0x1)==0)?2:3); // horiz. position for coeff_count context
  img->subblock_y = (b8<2)?((b4<2)?0:1):((b4<2)?2:3); // vert.  position for coeff_count context

  level=1; // get inside loop
  for(k=0; k<=16 && level !=0; k++)
  {
    level = currSE->value1 = ACLevel[k]; // level
    run   = currSE->value2 = ACRun  [k]; // run
      
    currSE->writing = writeRunLevel_CABAC;

    currSE->context     = LUMA_4x4;
    currSE->type        = (k==0 ? (intra4x4mode?SE_LUM_DC_INTRA:SE_LUM_DC_INTER) : (intra4x4mode?SE_LUM_AC_INTRA:SE_LUM_AC_INTER));
    img->is_intra_block = intra4x4mode;

    // choose the appropriate data partition
    dataPart = &(currSlice->partArr[partMap[currSE->type]]);          
    dataPart->writeSyntaxElement (currSE, dataPart);
    bitCount[BITS_COEFF_Y_MB] += currSE->len;
    rate                      += currSE->len;
#if TRACE
    snprintf(currSE->tracestring, TRACESTRING_SIZE, "Luma sng(%2d) level =%3d run =%2d", k, level,run);
#endif
    /* proceed to next SE */
    currSE++;  
    currMB->currSEnr++;
  }

  return rate;
}


/*!
 ************************************************************************
 * \brief
 *    Writes Luma Coeff of an 8x8 block
 ************************************************************************
 */
int writeLumaCoeff8x8 (int block8x8, int intra4x4mode)
{
  int  block4x4, rate = 0;

  for (block4x4=0; block4x4<4; block4x4++)
  {
    if (input->symbol_mode == UVLC )
      rate += writeCoeff4x4_CAVLC (LUMA, block8x8, block4x4, 0);// CAVLC
    else
      rate += writeLumaCoeff4x4_CABAC (block8x8, block4x4, intra4x4mode);
  }

  return rate;
}



/*!
 ************************************************************************
 * \brief
 *    Writes CBP, DQUANT, and Luma Coefficients of an macroblock
 ************************************************************************
 */
int writeCBPandLumaCoeff ()
{
  int             mb_x, mb_y, i, j, k;
  int             level, run;
  int             rate      = 0;
  Macroblock*     currMB    = &img->mb_data[img->current_mb_nr];
  int*            bitCount  = currMB->bitcounter;
  SyntaxElement*  currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
  Slice*          currSlice = img->currentSlice;
  const int*      partMap   = assignSE2partition[input->partition_mode];
  int             cbp       = currMB->cbp;
  DataPartition*  dataPart;

  int   b8, b4;
  int*  DCLevel = img->cofDC[0][0];
  int*  DCRun   = img->cofDC[0][1];
  int*  ACLevel;
  int*  ACRun;

  if (!IS_NEWINTRA (currMB))
  {
    //=====   C B P   =====
    //---------------------
    currSE->value1 = cbp;
    
    if (IS_OLDINTRA (currMB) || currMB->mb_type == SI4MB)
    {
      if (input->symbol_mode == UVLC)  currSE->mapping = cbp_linfo_intra;
      currSE->type = SE_CBP_INTRA;
    }
    else
    {
      if (input->symbol_mode == UVLC)  currSE->mapping = cbp_linfo_inter;
      currSE->type = SE_CBP_INTER;
    }
    if (input->symbol_mode == CABAC)   currSE->writing = writeCBP_CABAC;
                      
    // choose the appropriate data partition
    dataPart = &(currSlice->partArr[partMap[currSE->type]]);
    
    dataPart->writeSyntaxElement(currSE, dataPart);
    bitCount[BITS_CBP_MB] += currSE->len;
    rate                  += currSE->len;
#if TRACE
    snprintf(currSE->tracestring, TRACESTRING_SIZE, "CBP (%2d,%2d) = %3d",img->mb_x, img->mb_y, cbp);
#endif
    // proceed to next SE
    currSE++;  
    currMB->currSEnr++;
  }

  //=====   DQUANT   =====
  //----------------------
  if (cbp!=0 || IS_NEWINTRA (currMB))
  {
    currSE->value1 = currMB->delta_qp;

    if (input->symbol_mode==UVLC)   currSE->mapping = se_linfo;
    else                            currSE->writing = writeDquant_CABAC;

    if (IS_INTER (currMB))  currSE->type = SE_DELTA_QUANT_INTER;
    else                    currSE->type = SE_DELTA_QUANT_INTRA;


    // choose the appropriate data partition
    dataPart = &(img->currentSlice->partArr[partMap[currSE->type]]);
    dataPart->writeSyntaxElement(  currSE, dataPart);
    bitCount[BITS_DELTA_QUANT_MB] += currSE->len;
    rate                          += currSE->len;
#if TRACE
    snprintf(currSE->tracestring, TRACESTRING_SIZE, "Delta QP (%2d,%2d) = %3d",img->mb_x, img->mb_y, currMB->delta_qp);
#endif
    // proceed to next SE
    currSE++;
    currMB->currSEnr++;
  }

  for (j=0; j < 6; j++)
    for (i=0; i < 4; i++)
      img->nz_coeff [img->current_mb_nr][i][j]=0;  // CAVLC

  if (!IS_NEWINTRA (currMB))
  {
    //=====  L U M I N A N C E   =====
    //--------------------------------
    for (i=0; i<4; i++)  if (cbp & (1<<i))
    {
        rate += writeLumaCoeff8x8 (i, (currMB->b8mode[i]==IBLOCK));
    }
  }
  else
  {
    //=====  L U M I N A N C E   f o r   1 6 x 1 6   =====
    //----------------------------------------------------
    // DC coeffs
    if (input->symbol_mode == UVLC)
    {
      rate += writeCoeff4x4_CAVLC (LUMA_INTRA16x16DC, 0, 0, 0);  // CAVLC
    }
    else
    {
      level=1; // get inside loop
      for (k=0; k<=16 && level!=0; k++)
      {
        level = currSE->value1 = DCLevel[k]; // level
        run   = currSE->value2 = DCRun  [k]; // run

        if (input->symbol_mode == UVLC)
        {
          currSE->mapping = levrun_linfo_inter;
        }
        else
        {
          currSE->writing = writeRunLevel_CABAC;
        }

        currSE->context     = LUMA_16DC;
        currSE->type        = SE_LUM_DC_INTRA;   // element is of type DC
        img->is_intra_block = 1;

        // choose the appropriate data partition
        dataPart = &(currSlice->partArr[partMap[currSE->type]]);
    
        dataPart->writeSyntaxElement (currSE, dataPart);
        bitCount[BITS_COEFF_Y_MB] += currSE->len;
        rate                      += currSE->len;
#if TRACE
        snprintf(currSE->tracestring, TRACESTRING_SIZE, "DC luma 16x16 sng(%2d) level =%3d run =%2d", k, level, run);
#endif
        // proceed to next SE
        currSE++;
        currMB->currSEnr++;
      }
    }

    // AC coeffs
    if (cbp & 15)
    {
      for (mb_y=0; mb_y < 4; mb_y += 2)
      for (mb_x=0; mb_x < 4; mb_x += 2)
      for (j=mb_y; j < mb_y+2; j++)
      for (i=mb_x; i < mb_x+2; i++)
      {
        b8      = 2*(j/2) + (i/2);
        b4      = 2*(j%2) + (i%2);
        if (input->symbol_mode == UVLC)
        {
          rate += writeCoeff4x4_CAVLC (LUMA_INTRA16x16AC, b8, b4, 0);  // CAVLC
        }
        else
        {
          ACLevel = img->cofAC[b8][b4][0];
          ACRun   = img->cofAC[b8][b4][1];

          img->subblock_y = j;
          img->subblock_x = i;

          level=1; // get inside loop
          for (k=0;k<16 && level !=0;k++)
          {
            level = currSE->value1 = ACLevel[k]; // level
            run   = currSE->value2 = ACRun  [k]; // run

            if (input->symbol_mode == UVLC)
            {
              currSE->mapping = levrun_linfo_inter;
            }
            else
            {
              currSE->writing = writeRunLevel_CABAC;
            }
            currSE->context     = LUMA_16AC;
            currSE->type        = SE_LUM_AC_INTRA;   // element is of type AC
            img->is_intra_block = 1;

            // choose the appropriate data partition
           dataPart = &(currSlice->partArr[partMap[currSE->type]]);

            dataPart->writeSyntaxElement (currSE, dataPart);
            bitCount[BITS_COEFF_Y_MB] += currSE->len;
            rate                      += currSE->len;
#if TRACE
            snprintf(currSE->tracestring, TRACESTRING_SIZE, "AC luma 16x16 sng(%2d) level =%3d run =%2d", k, level, run);
#endif
            // proceed to next SE
            currSE++;
            currMB->currSEnr++;
          }
        }
      }
    }
  }

  return rate;
}


/*!
 ************************************************************************
 * \brief
 *    Get the Prediction from the Neighboring Blocks for Number of Nonzero Coefficients 
 *    
 *    Luma Blocks
 ************************************************************************
 */
int predict_nnz(int i,int j)
{
  PixelPos pix;

  int pred_nnz = 0;
  int cnt      = 0;
  int mb_nr    = img->current_mb_nr;

  // left block
  getLuma4x4Neighbour(mb_nr, i, j, -1, 0, &pix);
/* to be inserted only for dp
  if (pix.available && img->constrained_intra_pred_flag)
  {
    pix.available &= img->intra_block[pix.mb_addr];
  }
*/  
  if (pix.available)
  {
    pred_nnz = img->nz_coeff [pix.mb_addr ][pix.x][pix.y];
    cnt++;
  }

  // top block
  getLuma4x4Neighbour(mb_nr, i, j, 0, -1, &pix);
/* to be inserted only for dp
  if (pix.available && img->constrained_intra_pred_flag)
  {
    pix.available &= img->intra_block[pix.mb_addr];
  }
*/  
  if (pix.available)
  {
    pred_nnz += img->nz_coeff [pix.mb_addr ][pix.x][pix.y];
    cnt++;
  }

  if (cnt==2)
  {
    pred_nnz++;
    pred_nnz/=cnt; 
  }

  return pred_nnz;
}


/*!
 ************************************************************************
 * \brief
 *    Get the Prediction from the Neighboring Blocks for Number of Nonzero Coefficients 
 *    
 *    Chroma Blocks   
 ************************************************************************
 */
int predict_nnz_chroma(int i,int j)
{
  PixelPos pix;

  int pred_nnz = 0;
  int cnt      =0;
  int mb_nr    = img->current_mb_nr;

  // left block
  getChroma4x4Neighbour(mb_nr, i%2, j-4, -1, 0, &pix);
/*  to be inserted only for dp
  if (pix.available && img->constrained_intra_pred_flag)
  {
    pix.available &= img->intra_block[pix.mb_addr];
  }
*/  
  if (pix.available)
  {
    pred_nnz = img->nz_coeff [pix.mb_addr ][2 * (i/2) + pix.x][4 + pix.y];
    cnt++;
  }
  
  // top block
  getChroma4x4Neighbour(mb_nr, i%2, j-4, 0, -1, &pix);
/*  to be inserted only for dp
  if (pix.available && img->constrained_intra_pred_flag)
  {
    pix.available &= img->intra_block[pix.mb_addr];
  }
*/  
  if (pix.available)
  {
    pred_nnz += img->nz_coeff [pix.mb_addr ][2 * (i/2) + pix.x][4 + pix.y];
    cnt++;
  }

  if (cnt==2)
  {
    pred_nnz++;
    pred_nnz/=cnt; 
  }

  return pred_nnz;
}



/*!
 ************************************************************************
 * \brief
 *    Writes coeff of an 4x4 block (CAVLC)
 *
 * \author
 *    Karl Lillevold <karll@real.com>
 *    contributions by James Au <james@ubvideo.com>
 ************************************************************************
 */

int writeCoeff4x4_CAVLC (int block_type, int b8, int b4, int param)
{
  int           no_bits    = 0;
  Macroblock    *currMB    = &img->mb_data[img->current_mb_nr];
  SyntaxElement *currSE    = &img->MB_SyntaxElements[currMB->currSEnr];
  int           *bitCount  = currMB->bitcounter;
  Slice         *currSlice = img->currentSlice;
  DataPartition *dataPart;
  int           *partMap   = assignSE2partition[input->partition_mode];

  int k,level,run,vlcnum;
  int numcoeff, lastcoeff, numtrailingones; 
  int numones, totzeros, zerosleft, numcoef;
  int numcoeff_vlc;
  int code, level_two_or_higher;
  int dptype = 0, bitcounttype = 0;
  int nnz, max_coeff_num = 0, cdc=0, cac=0;
  int subblock_x, subblock_y;
  char type[15];

  int incVlc[] = {0,3,6,12,24,48,32768};  // maximum vlc = 6


  int*  pLevel = NULL;
  int*  pRun = NULL;

  switch (block_type)
  {
  case LUMA:
    max_coeff_num = 16;
    bitcounttype = BITS_COEFF_Y_MB;

    pLevel = img->cofAC[b8][b4][0];
    pRun   = img->cofAC[b8][b4][1];

    sprintf(type, "%s", "Luma");
    if (IS_INTRA (currMB))
    {
      dptype = SE_LUM_AC_INTRA;
    }
    else
    {
      dptype = SE_LUM_AC_INTER;
    }
    break;
  case LUMA_INTRA16x16DC:
    max_coeff_num = 16;
    bitcounttype = BITS_COEFF_Y_MB;

    pLevel = img->cofDC[0][0];
    pRun   = img->cofDC[0][1];

    sprintf(type, "%s", "Lum16DC");
    dptype = SE_LUM_DC_INTRA;
    break;
  case LUMA_INTRA16x16AC:
    max_coeff_num = 15;
    bitcounttype = BITS_COEFF_Y_MB;

    pLevel = img->cofAC[b8][b4][0];
    pRun   = img->cofAC[b8][b4][1];

    sprintf(type, "%s", "Lum16AC");
    dptype = SE_LUM_AC_INTRA;
    break;

  case CHROMA_DC:
    max_coeff_num = 4;
    bitcounttype = BITS_COEFF_UV_MB;
    cdc = 1;

    pLevel = img->cofDC[param+1][0];
    pRun   = img->cofDC[param+1][1];

    sprintf(type, "%s", "ChrDC");
    if (IS_INTRA (currMB))
    {
      dptype = SE_CHR_DC_INTRA;
    }
    else
    {
      dptype = SE_CHR_DC_INTER;
    }
    break;
  case CHROMA_AC:
    max_coeff_num = 15;
    bitcounttype = BITS_COEFF_UV_MB;
    cac = 1;

    pLevel = img->cofAC[b8][b4][0];
    pRun   = img->cofAC[b8][b4][1];

    sprintf(type, "%s", "ChrAC");
    if (IS_INTRA (currMB))
    {
      dptype = SE_CHR_AC_INTRA;
    }
    else
    {
      dptype = SE_CHR_AC_INTER;
    }
    break;
  default:
    error("writeCoeff4x4_CAVLC: Invalid block type", 600);
    break;
  }

  dataPart = &(currSlice->partArr[partMap[dptype]]);

  numcoeff = 0;
  numtrailingones = 0;
  numones = 0;
  lastcoeff = 0;
  totzeros = 0;
  level = 1;

  for(k = 0; (k <= cdc?4:16)&& level !=0; k++)
  {
    level = pLevel[k]; // level
    run   = pRun[k];   // run

    if (level)
    {
      if (run)
        totzeros += run;
      if (abs(level) == 1)
      {
        numtrailingones ++;
        numones ++;
        if (numtrailingones > 3)
        {
          numtrailingones = 3; /* clip to 3 */
        }
      }
      else
      {
        numtrailingones = 0;
      }
      numcoeff ++;
      lastcoeff = k;
    }
  }

  if (!cdc)
  {
    if (!cac)
    {
      // luma
      subblock_x = ((b8&0x1)==0)?(((b4&0x1)==0)?0:1):(((b4&0x1)==0)?2:3); 
        // horiz. position for coeff_count context
      subblock_y = (b8<2)?((b4<2)?0:1):((b4<2)?2:3); 
        // vert.  position for coeff_count context
      nnz = predict_nnz(subblock_x,subblock_y);
    }
    else
    {
      // chroma AC
      subblock_x = param >> 4;
      subblock_y = param & 15;
      nnz = predict_nnz_chroma(subblock_x,subblock_y);
    }

    img->nz_coeff [img->current_mb_nr ][subblock_x][subblock_y] = numcoeff;


    if (nnz < 2)
    {
      numcoeff_vlc = 0;
    }
    else if (nnz < 4)
    {
      numcoeff_vlc = 1;
    }
    else if (nnz < 8)
    {
      numcoeff_vlc = 2;
    }
    else 
    {
      numcoeff_vlc = 3;
    }


  }
  else
  {
    // chroma DC (has its own VLC)
    // numcoeff_vlc not relevant
    numcoeff_vlc = 0;

    subblock_x = param;
    subblock_y = param;
  }

  currSE->type  = dptype;   

  currSE->value1 = numcoeff;
  currSE->value2 = numtrailingones;
  currSE->len = numcoeff_vlc; /* use len to pass vlcnum */

#if TRACE
  snprintf(currSE->tracestring, 
    TRACESTRING_SIZE, "%s # c & tr.1s(%d,%d) vlc=%d #c=%d #t1=%d",
    type, subblock_x, subblock_y, numcoeff_vlc, numcoeff, numtrailingones);
#endif

  if (!cdc)
    writeSyntaxElement_NumCoeffTrailingOnes(currSE, dataPart);
  else
    writeSyntaxElement_NumCoeffTrailingOnesChromaDC(currSE, dataPart);

  bitCount[bitcounttype]+=currSE->len;
  no_bits               +=currSE->len;

  // proceed to next SE
  currSE++;
  currMB->currSEnr++;


  if (!numcoeff)
    return no_bits;

  if (numcoeff)
  {
    code = 0;
    for (k = lastcoeff; k > lastcoeff-numtrailingones; k--)
    {
      level = pLevel[k]; // level
      if (abs(level) > 1)
      {
        printf("ERROR: level > 1\n");
        exit(-1);
      }
      code <<= 1;
      if (level < 0)
      {
        code |= 0x1;
      }
    }

    if (numtrailingones)
    {
      currSE->type  = dptype;   

      currSE->value2 = numtrailingones;
      currSE->value1 = code;

#if TRACE
      snprintf(currSE->tracestring, 
        TRACESTRING_SIZE, "%s trailing ones sign (%d,%d)", 
        type, subblock_x, subblock_y);
#endif

      writeSyntaxElement_VLC (currSE, dataPart);
      bitCount[bitcounttype]+=currSE->len;
      no_bits               +=currSE->len;

      // proceed to next SE
      currSE++;
      currMB->currSEnr++;
    }

    // encode levels
    level_two_or_higher = 1;
    if (numcoeff > 3 && numtrailingones == 3)
      level_two_or_higher = 0;

    if (numcoeff > 10 && numtrailingones < 3)
      vlcnum = 1;
    else
      vlcnum = 0;

    for (k = lastcoeff - numtrailingones; k >= 0; k--)
    {
      level = pLevel[k]; // level

      currSE->value1 = level;
      currSE->type  = dptype;   
      
#if TRACE
      snprintf(currSE->tracestring, 
        TRACESTRING_SIZE, "%s lev (%d,%d) k=%d vlc=%d lev=%3d",
        type, subblock_x, subblock_y, k, vlcnum, level);
#endif
      
      if (level_two_or_higher)
      {
        if (currSE->value1 > 0)
          currSE->value1 --;
        else
          currSE->value1 ++;
        level_two_or_higher = 0;
      }

      //    encode level
      if (vlcnum == 0)
        writeSyntaxElement_Level_VLC1(currSE, dataPart);
      else
        writeSyntaxElement_Level_VLCN(currSE, vlcnum, dataPart);

      // update VLC table
      if (abs(level)>incVlc[vlcnum])
        vlcnum++;

      if (k == lastcoeff - numtrailingones && abs(level)>3)
        vlcnum = 2;

      bitCount[bitcounttype]+=currSE->len;
      no_bits               +=currSE->len;

      // proceed to next SE
      currSE++;
      currMB->currSEnr++;
    }


    // encode total zeroes
    if (numcoeff < max_coeff_num)
    {

      currSE->type  = dptype;   
      currSE->value1 = totzeros;

      vlcnum = numcoeff-1;

      currSE->len = vlcnum;

#if TRACE
      snprintf(currSE->tracestring, 
        TRACESTRING_SIZE, "%s totalrun (%d,%d) vlc=%d totzeros=%3d",
          type, subblock_x, subblock_y, vlcnum, totzeros);
#endif
      if (!cdc)
        writeSyntaxElement_TotalZeros(currSE, dataPart);
      else
        writeSyntaxElement_TotalZerosChromaDC(currSE, dataPart);

      bitCount[bitcounttype]+=currSE->len;
      no_bits               +=currSE->len;

      // proceed to next SE
      currSE++;
      currMB->currSEnr++;
    }

    // encode run before each coefficient
    zerosleft = totzeros;
    numcoef = numcoeff;
    for (k = lastcoeff; k >= 0; k--)
    {
      run = pRun[k]; // run

      currSE->value1 = run;
      currSE->type  = dptype;   

      // for last coeff, run is remaining totzeros
      // when zerosleft is zero, remaining coeffs have 0 run
      if (numcoeff <= 1 || !zerosleft)
        break;

      if (numcoef > 1 && zerosleft) 
      {

        vlcnum = zerosleft - 1;
        if (vlcnum > RUNBEFORE_NUM-1)
          vlcnum = RUNBEFORE_NUM-1;

        currSE->len = vlcnum;

#if TRACE
        snprintf(currSE->tracestring, 
          TRACESTRING_SIZE, "%s run (%d,%d) k=%d vlc=%d run=%2d",
            type, subblock_x, subblock_y, k, vlcnum, run);
#endif

        writeSyntaxElement_Run(currSE, dataPart);

        bitCount[bitcounttype]+=currSE->len;
        no_bits               +=currSE->len;

        zerosleft -= run;
        numcoef --;

        // proceed to next SE
        currSE++;
        currMB->currSEnr++;
      }
    }
  }

  return no_bits;
}




/*!
 ************************************************************************
 * \brief
 *    Find best 16x16 based intra mode
 *
 * \par Input:
 *    Image parameters, pointer to best 16x16 intra mode
 *
 * \par Output:
 *    best 16x16 based SAD
 ************************************************************************/
int find_sad_16x16(int *intra_mode)
{
  int current_intra_sad_2,best_intra_sad2;
  int M1[16][16],M0[4][4][4][4],M3[4],M4[4][4];

  int i,j,k;
  int ii,jj;
  int mb_nr = img->current_mb_nr;
  
  PixelPos up;          //!< pixel position p(0,-1)
  PixelPos left[17];    //!< pixel positions p(-1, -1..15)

  int up_avail, left_avail, left_up_avail;

  for (i=0;i<17;i++)
  {
    getNeighbour(mb_nr, -1 ,  i-1 , 1, &left[i]);
  }
  
  getNeighbour(mb_nr, 0     ,  -1 , 1, &up);

  if (!(input->UseConstrainedIntraPred))
  {
    up_avail   = up.available;
    left_avail = left[1].available;
    left_up_avail = left[0].available;
  }
  else
  {
    up_avail      = up.available ? img->intra_block[up.mb_addr] : 0;
    for (i=1, left_avail=1; i<17;i++)
      left_avail  &= left[i].available ? img->intra_block[left[i].mb_addr]: 0;
    left_up_avail = left[0].available ? img->intra_block[left[0].mb_addr]: 0;
  }

  best_intra_sad2=MAX_VALUE;

  for (k=0;k<4;k++)
  {
    //check if there are neighbours to predict from
    if ((k==0 && !up_avail) || (k==1 && !left_avail) || (k==3 && (!left_avail || !up_avail || !left_up_avail)))
    {
      ; // edge, do nothing
    }
    else
    {
      for (j=0;j<16;j++)
      {
        for (i=0;i<16;i++)
        {
          M1[i][j]=imgY_org[img->opix_y+j][img->opix_x+i]-img->mprr_2[k][j][i];
          M0[i%4][i/4][j%4][j/4]=M1[i][j];
        }
      }
      current_intra_sad_2=0;              // no SAD start handicap here
      for (jj=0;jj<4;jj++)
      {
        for (ii=0;ii<4;ii++)
        {
          for (j=0;j<4;j++)
          {
            M3[0]=M0[0][ii][j][jj]+M0[3][ii][j][jj];
            M3[1]=M0[1][ii][j][jj]+M0[2][ii][j][jj];
            M3[2]=M0[1][ii][j][jj]-M0[2][ii][j][jj];
            M3[3]=M0[0][ii][j][jj]-M0[3][ii][j][jj];

            M0[0][ii][j][jj]=M3[0]+M3[1];
            M0[2][ii][j][jj]=M3[0]-M3[1];
            M0[1][ii][j][jj]=M3[2]+M3[3];
            M0[3][ii][j][jj]=M3[3]-M3[2];
          }

          for (i=0;i<4;i++)
          {
            M3[0]=M0[i][ii][0][jj]+M0[i][ii][3][jj];
            M3[1]=M0[i][ii][1][jj]+M0[i][ii][2][jj];
            M3[2]=M0[i][ii][1][jj]-M0[i][ii][2][jj];
            M3[3]=M0[i][ii][0][jj]-M0[i][ii][3][jj];

            M0[i][ii][0][jj]=M3[0]+M3[1];
            M0[i][ii][2][jj]=M3[0]-M3[1];
            M0[i][ii][1][jj]=M3[2]+M3[3];
            M0[i][ii][3][jj]=M3[3]-M3[2];
            for (j=0;j<4;j++)
              if ((i+j)!=0)
                current_intra_sad_2 += abs(M0[i][ii][j][jj]);
          }
        }
      }

      for (j=0;j<4;j++)
        for (i=0;i<4;i++)
          M4[i][j]=M0[0][i][0][j]/4;

        // Hadamard of DC koeff
        for (j=0;j<4;j++)
        {
          M3[0]=M4[0][j]+M4[3][j];
          M3[1]=M4[1][j]+M4[2][j];
          M3[2]=M4[1][j]-M4[2][j];
          M3[3]=M4[0][j]-M4[3][j];

          M4[0][j]=M3[0]+M3[1];
          M4[2][j]=M3[0]-M3[1];
          M4[1][j]=M3[2]+M3[3];
          M4[3][j]=M3[3]-M3[2];
        }

        for (i=0;i<4;i++)
        {
          M3[0]=M4[i][0]+M4[i][3];
          M3[1]=M4[i][1]+M4[i][2];
          M3[2]=M4[i][1]-M4[i][2];
          M3[3]=M4[i][0]-M4[i][3];

          M4[i][0]=M3[0]+M3[1];
          M4[i][2]=M3[0]-M3[1];
          M4[i][1]=M3[2]+M3[3];
          M4[i][3]=M3[3]-M3[2];

          for (j=0;j<4;j++)
            current_intra_sad_2 += abs(M4[i][j]);
        }
        if(current_intra_sad_2 < best_intra_sad2)
        {
          best_intra_sad2=current_intra_sad_2;
          *intra_mode = k; // update best intra mode

        }
    }
  }
  best_intra_sad2 = best_intra_sad2/2;

  return best_intra_sad2;

}
