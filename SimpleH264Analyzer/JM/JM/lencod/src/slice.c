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
 **************************************************************************************
 * \file
 *    slice.c
 * \brief
 *    generate the slice header, setup the bit buffer for slices,
 *    and generates the slice NALU(s)

 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Thomas Stockhammer            <stockhammer@ei.tum.de>
 *      - Detlev Marpe                  <marpe@hhi.de>
 *      - Stephan Wenger                <stewe@cs.tu-berlin.de>
 ***************************************************************************************
 */

#include "contributors.h"

#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "global.h"
#include "header.h"
#include "rtp.h"
#include "nalu.h"
#include "annexb.h"
#include "parset.h"
#include "fmo.h"
#include "vlc.h"
#include "image.h"
#include "cabac.h"
#include "elements.h"
#include "mbuffer.h"

// Local declarations

static Slice *malloc_slice();
static void  free_slice(Slice *slice);
static void  init_slice(int start_mb_addr);
static void set_ref_pic_num();
extern ColocatedParams *Co_located;
extern StorablePicture **listX[6];

/*!
 ************************************************************************
 * \brief
 *    init_ref_pic_list_reordering initializations should go here
 ************************************************************************
 */
void init_ref_pic_list_reordering()
{
  Slice* currSlice = img->currentSlice;

  currSlice->ref_pic_list_reordering_flag_l0 = 0;
  currSlice->ref_pic_list_reordering_flag_l1 = 0;
}


/*!
 ************************************************************************
 *  \brief
 *     This function generates the slice (and partition) header(s) 
 *
 *  \return number of bits used for the slice (and partition) header(s)
 *
 *  \par Side effects:
 *      Adds slice/partition header symbols to the symbol buffer
 *      increments Picture->no_slices, allocates memory for the
 *      slice, sets img->currSlice
 ************************************************************************
*/
int start_slice()
{
  EncodingEnvironmentPtr eep;
  Slice *currSlice = img->currentSlice;
  Bitstream *currStream;
  int header_len = 0;
  int i;
  int NumberOfPartitions = (input->partition_mode == PAR_DP_1?1:3);

	//one  partition for IDR img
	if(img->currentPicture->idr_flag)
	{
     NumberOfPartitions = 1;
	}

  init_ref_pic_list_reordering();

  RTPUpdateTimestamp (img->tr);   // this has no side effects, just leave it for all NALs

  for (i=0; i<NumberOfPartitions; i++)
  {
    currStream = (currSlice->partArr[i]).bitstream;

    currStream->write_flag = 0;
    if (i==0)     // First partition
      header_len += SliceHeader (0);
    else          // Second/Third partition
      header_len += Partition_BC_Header(i);
     
    //! Initialize CABAC
    if (input->symbol_mode == CABAC)
    {
      eep = &((currSlice->partArr[i]).ee_cabac);
      if (currStream->bits_to_go != 8)
        header_len+=currStream->bits_to_go;
      writeVlcByteAlign(currStream);
      arienco_start_encoding(eep, currStream->streamBuffer, &(currStream->byte_pos)/*, &(currStream->last_startcode)*/,img->type);
      cabac_new_slice();
    } else 
    {
      // Initialize CA-VLC
      CAVLC_init();
    }
  }
  if(input->symbol_mode == CABAC)
  {
    init_contexts();
  }
  return header_len;
}



/*!
 ************************************************************************
 * \brief
 *    This function terminates a slice (but doesn't write it out), 
 *    the old terminate_slice (0)
 * \return
 *    0 if OK,                                                         \n
 *    1 in case of error
 *
 ************************************************************************
 */
int terminate_slice()
{
  int bytes_written;
  Bitstream *currStream;
  Slice *currSlice = img->currentSlice;
  EncodingEnvironmentPtr eep;
  int i;
  int byte_pos_before_startcode_emu_prevention;

  if (input->symbol_mode == CABAC)
    write_terminating_bit (1);      // only once, not for all partitions
  
  for (i=0; i<currSlice->max_part_nr; i++)
  {
    currStream = (currSlice->partArr[i]).bitstream;
    if (input->symbol_mode == UVLC)
    {
      SODBtoRBSP(currStream);
      byte_pos_before_startcode_emu_prevention = currStream->byte_pos;
      currStream->byte_pos = RBSPtoEBSP(currStream->streamBuffer, 0 , currStream->byte_pos, 0);
      *(stat->em_prev_bits) += (currStream->byte_pos - byte_pos_before_startcode_emu_prevention) * 8;
    }
    else     // CABAC
    {
      eep = &((currSlice->partArr[i]).ee_cabac);
      // terminate the arithmetic code
      arienco_done_encoding(eep);
      currStream->bits_to_go = eep->Ebits_to_go;
      currStream->byte_buf = 0;
      bytes_written = currStream->byte_pos;
      byte_pos_before_startcode_emu_prevention= currStream->byte_pos;
      currStream->byte_pos = RBSPtoEBSP(currStream->streamBuffer, 0, currStream->byte_pos, eep->E);
      *(stat->em_prev_bits) += (currStream->byte_pos - byte_pos_before_startcode_emu_prevention) * 8;
    }           // CABAC
  }           // partition loop
  if( input->symbol_mode == CABAC )
  {
    store_contexts();
  }

  return 0;   
}

/*!
 ************************************************************************
 * \brief
 *    Encodes one slice
 * \par
 *   returns the number of coded MBs in the SLice 
 ************************************************************************
 */
int encode_one_slice (int SliceGroupId, Picture *pic)
{
  Boolean end_of_slice = FALSE;
  Boolean recode_macroblock;
  int len;
  int NumberOfCodedMBs = 0;
  int CurrentMbAddr;
  double FrameRDCost, FieldRDCost;

  img->cod_counter = 0;

  set_ref_pic_num();

  if (img->type == B_SLICE)
      compute_collocated(Co_located, listX);

  CurrentMbAddr = FmoGetFirstMacroblockInSlice (SliceGroupId);
// printf ("\n\nEncode_one_slice: PictureID %d SliceGroupId %d  SliceID %d  FirstMB %d \n", img->tr, SliceGroupId, img->current_slice_nr, CurrentMbInScanOrder);

  init_slice (CurrentMbAddr);
  Bytes_After_Header = img->currentSlice->partArr[0].bitstream->byte_pos;

  if (input->symbol_mode==CABAC)
  {
    SetCtxModelNumber ();
  }

/*
  // Tian Dong: June 7, 2002 JVT-B042
  // When the pictures are put into different layers and subseq, not all the reference frames
  // in multi-frame buffer are valid for prediction. The acutual number of the valid reference
  // frames, fb->num_short_used, will be given by start_slice(sym).
  // Save the fb->short_used.
  if (input->NumFramesInELSubSeq)
    {
      short_used = fb->short_used;
      img_ref = img->nb_references;
    }
*/

  len = start_slice ();

  // Rate control
  img->NumberofHeaderBits +=len;

  // basic unit layer rate control
  if(img->BasicUnit<img->Frame_Total_Number_MB)
    img->NumberofBasicUnitHeaderBits +=len;

//  printf("short size, used, num-used: (%d,%d,%d)\n", fb->short_size, fb->short_used, fb->num_short_used);

/*
  // Tian Dong: June 7, 2002 JVT-B042
  if (input->NumFramesInELSubSeq)
    {
      fb->short_used = fb->num_short_used;
      img->nb_references = fb->short_used + fb->long_used;
    }
*/
  // Update statistics
  stat->bit_slice += len;
  stat->bit_use_header[img->type] += len;
// printf ("\n\n");

  while (end_of_slice == FALSE) // loop over macroblocks
  {
    //sw paff
    if (!img->MbaffFrameFlag)
    {
      recode_macroblock = FALSE;
      rdopt = &rddata_top_frame_mb;   // store data in top frame MB 
      
      start_macroblock (CurrentMbAddr, FALSE);
      encode_one_macroblock ();
      write_one_macroblock (1);
      terminate_macroblock (&end_of_slice, &recode_macroblock);

// printf ("encode_one_slice: mb %d,  slice %d,   bitbuf bytepos %d EOS %d\n", 
//       img->current_mb_nr, img->current_slice_nr, 
//       img->currentSlice->partArr[0].bitstream->byte_pos, end_of_slice);

      if (recode_macroblock == FALSE)       // The final processing of the macroblock has been done
      {
        CurrentMbAddr = FmoGetNextMBNr (CurrentMbAddr);
        if (CurrentMbAddr == -1)   // end of slice
        {
// printf ("FMO End of Slice Group detected, current MBs %d, force end of slice\n", NumberOfCodedMBs+1);
          end_of_slice = TRUE;
        }
        NumberOfCodedMBs++;       // only here we are sure that the coded MB is actually included in the slice
        proceed2nextMacroblock (CurrentMbAddr);
      }
      else
      {
        //!Go back to the previous MB to recode it
        img->current_mb_nr = FmoGetPreviousMBNr(img->current_mb_nr);
        if(img->current_mb_nr == -1 )   // The first MB of the slice group  is too big,
                                        // which means it's impossible to encode picture using current slice bits restriction
        {
          snprintf (errortext, ET_SIZE, "Error encoding first MB with spcified parameter, bits of current MB may be too big");
          error (errortext, 300);
        }
      }
    }
    else                      // TBD -- Addition of FMO
    {

//! This following ugly code breaks slices, at least for a slice mode that accumulates a certain
//! number of bits into one slice.  
//! The suggested algorithm is as follows:
//!
//! SaveState (Bitstream, stats,  etc. etc.);
//! BitsForThisMBPairInFrameMode = CodeMB (Upper, FRAME_MODE) + CodeMB (Lower, FRAME_MODE);
//! DistortionForThisMBPairInFrameMode = CalculateDistortion(Upper) + CalculateDistortion (Lower);
//! RestoreState();
//! BitsForThisMBPairInFieldMode = CodeMB (Upper, FIELD_MODE) + CodeMB (Lower, FIELD_MODE);
//! DistortionForThisMBPairInFrameMode = CalculateDistortion(Upper) + CalculateDistortion (Lower);
//! FrameFieldMode = Decision (...)
//! RestoreState()
//! if (FrameFieldMode == FRAME) {
//!   CodeMB (Upper, FRAME); CodeMB (Lower, FRAME);
//! } else {
//!   CodeMB (Upper FIELD); CodeMB (Lower, FIELD);
//! }
//!
//! Open questions/issues:
//!   1. CABAC/CA-VLC state:  It seems that the CABAC/CA_VLC states are changed during the
//!      dummy encoding processes (for the R-D based selection), but that they are never
//!      reset, once the selection is made.  I believe that this breaks the MB-adaptive
//!      frame/field coding.  The necessary code for the state saves is readily available
//!      in macroblock.c, start_macroblock() and terminate_macroblock() (this code needs
//!      to be double checked that it works with CA-VLC as well
//!   2. would it be an option to allocate Bitstreams with zero data in them (or copy the
//!      already generated bitstream) for the "test coding"?  

      if (input->MbInterlace == ADAPTIVE_CODING)
      {
        // code MB pair as frame MB 
        recode_macroblock = FALSE;
        
        
        img->field_mode = 0;  // MB coded as frame
        img->top_field = 0;   // Set top field to 0
        
        //Rate control
        img->write_macroblock = 0;
        img->bot_MB = 0;   
        
        start_macroblock (CurrentMbAddr, FALSE);
        
        rdopt = &rddata_top_frame_mb; // store data in top frame MB 
        encode_one_macroblock ();     // code the MB as frame
        FrameRDCost = rdopt->min_rdcost;
        //***   Top MB coded as frame MB ***//
        
        //Rate control
        img->bot_MB = 1; //for Rate control
        
        // go to the bottom MB in the MB pair
        img->field_mode = 0;  // MB coded as frame  //GB
        
        start_macroblock (CurrentMbAddr+1, FALSE);
        rdopt = &rddata_bot_frame_mb; // store data in top frame MB
        encode_one_macroblock ();     // code the MB as frame
        FrameRDCost += rdopt->min_rdcost;
        
        //***   Bottom MB coded as frame MB ***//
      }

      if ((input->MbInterlace == ADAPTIVE_CODING) || (input->MbInterlace == FIELD_CODING))
      {
        //Rate control
        img->bot_MB = 0; 
        
        // start coding the MB pair as a field MB pair
        img->field_mode = 1;  // MB coded as frame
        img->top_field = 1;   // Set top field to 1
        img->buf_cycle <<= 1;
        input->num_reference_frames <<= 1;
        img->num_ref_idx_l0_active <<= 1;
        img->num_ref_idx_l0_active += 1;
        start_macroblock (CurrentMbAddr, TRUE);
        

        rdopt = &rddata_top_field_mb; // store data in top frame MB 
//        TopFieldIsSkipped = 0;        // set the top field MB skipped flag to 0
        encode_one_macroblock ();     // code the MB as frame
        FieldRDCost = rdopt->min_rdcost;
        //***   Top MB coded as field MB ***//
        //Rate control
        img->bot_MB = 1;//for Rate control

        img->top_field = 0;   // Set top field to 0
        start_macroblock (CurrentMbAddr+1, TRUE);
        rdopt = &rddata_bot_field_mb; // store data in top frame MB 
        encode_one_macroblock ();     // code the MB as frame
        FieldRDCost += rdopt->min_rdcost;
        //***   Bottom MB coded as field MB ***//
      }

      //Rate control
      img->write_macroblock_frame = 0;  //Rate control

      
      // decide between frame/field MB pair
      if ((input->MbInterlace == ADAPTIVE_CODING) && (FrameRDCost < FieldRDCost))
      {
        img->field_mode = 0;
        img->buf_cycle >>= 1;
        input->num_reference_frames >>= 1;
        MBPairIsField = 0;
        img->num_ref_idx_l0_active -= 1;
        img->num_ref_idx_l0_active >>= 1;
        
        //Rate control
        img->write_macroblock_frame = 1;  //for Rate control
      }
      else
      {
        img->field_mode = 1;
        MBPairIsField = 1;
      }
      
      //Rate control
      img->write_macroblock = 1;//Rate control 
      
      if (MBPairIsField)
        img->top_field = 1;
      else
        img->top_field = 0;
      
      //Rate control
      img->bot_MB = 0;// for Rate control

      // go back to the Top MB in the MB pair
      start_macroblock (CurrentMbAddr, img->field_mode);
      
      rdopt =  img->field_mode ? &rddata_top_field_mb : &rddata_top_frame_mb;
      copy_rdopt_data (0);  // copy the MB data for Top MB from the temp buffers
      write_one_macroblock (1);     // write the Top MB data to the bitstream
      NumberOfCodedMBs++;   // only here we are sure that the coded MB is actually included in the slice
      terminate_macroblock (&end_of_slice, &recode_macroblock);     // done coding the Top MB 
      proceed2nextMacroblock (CurrentMbAddr);        // Go to next macroblock
      
      //Rate control
      img->bot_MB = 1;//for Rate control
      // go to the Bottom MB in the MB pair
      img->top_field = 0;
      start_macroblock (CurrentMbAddr+1, img->field_mode);

      rdopt = img->field_mode ? &rddata_bot_field_mb : &rddata_bot_frame_mb;
      copy_rdopt_data (1);  // copy the MB data for Bottom MB from the temp buffers
      
      write_one_macroblock (0);     // write the Bottom MB data to the bitstream
      NumberOfCodedMBs++;   // only here we are sure that the coded MB is actually included in the slice
      terminate_macroblock (&end_of_slice, &recode_macroblock);     // done coding the Top MB 
      proceed2nextMacroblock (CurrentMbAddr);        // Go to next macroblock
      
      if (MBPairIsField)    // if MB Pair was coded as field the buffer size variables back to frame mode
      {
        img->buf_cycle >>= 1;
        input->num_reference_frames >>= 1;
        img->num_ref_idx_l0_active -= 1;
        img->num_ref_idx_l0_active >>= 1;
      }
      img->field_mode = img->top_field = 0; // reset to frame mode
      
      
      // go to next MB pair, not next MB
      CurrentMbAddr = FmoGetNextMBNr (CurrentMbAddr);
      CurrentMbAddr = FmoGetNextMBNr (CurrentMbAddr);
      
      if (CurrentMbAddr == FmoGetLastCodedMBOfSliceGroup (FmoMB2SliceGroup (CurrentMbAddr)))
        end_of_slice = TRUE;        // just in case it does n't get set in terminate_macroblock  
    }
  }  
/*
  // Tian Dong: June 7, 2002 JVT-B042
  // Restore the short_used
  if (input->NumFramesInELSubSeq)
    {
      fb->short_used = short_used;
      img->nb_references = img_ref;
    }
*/
  terminate_slice ();
  return NumberOfCodedMBs;
}



/*!
 ************************************************************************
 * \brief
 *    Initializes the parameters for a new slice and
 *     allocates the memory for the coded slice in the Picture structure
 *  \par Side effects:
 *      Adds slice/partition header symbols to the symbol buffer
 *      increments Picture->no_slices, allocates memory for the
 *      slice, sets img->currSlice
 ************************************************************************
 */
static void init_slice (int start_mb_addr)
{
  int i;
  Picture *currPic = img->currentPicture;
  Slice *curr_slice;
  DataPartition *dataPart;
  Bitstream *currStream;

  img->current_mb_nr = start_mb_addr;

  // Allocate new Slice in the current Picture, and set img->currentSlice
  assert (currPic != NULL);
  currPic->no_slices++;
  if (currPic->no_slices >= MAXSLICEPERPICTURE)
    error ("Too many slices per picture, increase MAXLSICESPERPICTURE in global.h.", -1);
  currPic->slices[currPic->no_slices-1] = malloc_slice();
  curr_slice = currPic->slices[currPic->no_slices-1];
  img->currentSlice = curr_slice;

  curr_slice->picture_id = img->tr % 256;
  curr_slice->qp = img->qp;
  curr_slice->start_mb_nr = start_mb_addr;
  curr_slice->slice_too_big = dummy_slice_too_big;

  for (i = 0; i < curr_slice->max_part_nr; i++)
  {
    dataPart = &(curr_slice->partArr[i]);
    if (input->symbol_mode == UVLC)
      dataPart->writeSyntaxElement = writeSyntaxElement_UVLC;
    else
      dataPart->writeSyntaxElement = writeSyntaxElement_CABAC;
    
    currStream = dataPart->bitstream;
    currStream->bits_to_go = 8;
    currStream->byte_pos = 0;
    currStream->byte_buf = 0;
  }
}


/*!
 ************************************************************************
 * \brief
 *    Allocates a slice structure along with its dependentdata structures
 * \return
 *    Pointer to a Slice
 ************************************************************************
 */
static Slice *malloc_slice()
{
  int i;
  DataPartition *dataPart;
  Slice *slice;
  const int buffer_size = (img->width * img->height * 4); // AH 190202: There can be data expansion with 
                                                          // low QP values. So, we make sure that buffer 
                                                          // does not everflow. 4 is probably safe multiplier.

  if ((slice = (Slice *) calloc(1, sizeof(Slice))) == NULL) no_mem_exit ("malloc_slice: slice structure");

  if (input->symbol_mode == CABAC)
    {
      // create all context models
      slice->mot_ctx = create_contexts_MotionInfo();
      slice->tex_ctx = create_contexts_TextureInfo();
    }

  slice->max_part_nr = input->partition_mode==0?1:3;

	//for IDR img there should be only one partition
	if(img->currentPicture->idr_flag)
		slice->max_part_nr = 1;
  
  assignSE2partition[0] = assignSE2partition_NoDP;
	//ZL 
	//for IDR img all the syntax element shoulde be mapped to one partition        
  if(!img->currentPicture->idr_flag&&input->partition_mode==1)
    assignSE2partition[1] =  assignSE2partition_DP;
	else
    assignSE2partition[1] =  assignSE2partition_NoDP;    		 



  slice->num_mb = 0;          // no coded MBs so far

  if ((slice->partArr = (DataPartition *) calloc(slice->max_part_nr, sizeof(DataPartition))) == NULL) no_mem_exit ("malloc_slice: partArr");
  for (i=0; i<slice->max_part_nr; i++) // loop over all data partitions
  {
    dataPart = &(slice->partArr[i]);
    if ((dataPart->bitstream = (Bitstream *) calloc(1, sizeof(Bitstream))) == NULL) no_mem_exit ("malloc_slice: Bitstream");
    if ((dataPart->bitstream->streamBuffer = (byte *) calloc(buffer_size, sizeof(byte))) == NULL) no_mem_exit ("malloc_slice: StreamBuffer");
    // Initialize storage of bitstream parameters
  }
  return slice;
}


/*!
 ************************************************************************
 * \brief
 *    Memory frees of all Slice structures and of its dependent
 *    data structures
 * \par Input:
 *    Image Parameters struct struct img_par *img
 ************************************************************************
 */
void free_slice_list(Picture *currPic)
{
  int i;

  for (i=0; i<currPic->no_slices; i++)
  {
    free_slice (currPic->slices[i]);
    currPic->slices[i]=NULL;
  }
}


/*!
 ************************************************************************
 * \brief
 *    Memory frees of the Slice structure and of its dependent
 *    data structures
 * \param slice:
 *    Slice to be freed
 ************************************************************************
 */
static void free_slice(Slice *slice)
{
  int i;
  DataPartition *dataPart;

  if (slice != NULL)
  {
    for (i=0; i<slice->max_part_nr; i++) // loop over all data partitions
    {
      dataPart = &(slice->partArr[i]);
      if (dataPart != NULL)
      {
        if (dataPart->bitstream->streamBuffer != NULL)
          free(dataPart->bitstream->streamBuffer);
        if (dataPart->bitstream != NULL)
          free(dataPart->bitstream);
      }
    }
    if (slice->partArr != NULL)
      free(slice->partArr);
    if (input->symbol_mode == CABAC)
    {
      delete_contexts_MotionInfo(slice->mot_ctx);
      delete_contexts_TextureInfo(slice->tex_ctx);
    }
    //free(img->currentSlice);
    free(slice);
  }
}


/*!
 ************************************************************************
 * \brief
 *    This function set the value of a bit in a bitstream to 1
 ************************************************************************
 */
void modify_redundant_pic_cnt(unsigned char *buffer)
{
  unsigned char tmp = 1 << (rpc_bits_to_go-1);
  buffer[rpc_bytes_to_go] |= tmp;
}

void set_ref_pic_num()
{
  int i,j;
  
  //! need to add field ref_pic_num that handles field pair.

  for (i=0;i<listXsize[LIST_0];i++)
  {
    enc_picture->ref_pic_num        [LIST_0][i]=listX[LIST_0][i]->poc * 2 + ((listX[LIST_0][i]->structure==BOTTOM_FIELD)?1:0) ; 
    enc_picture->frm_ref_pic_num    [LIST_0][i]=listX[LIST_0][i]->frame_poc * 2; 
    enc_picture->top_ref_pic_num    [LIST_0][i]=listX[LIST_0][i]->top_poc * 2; 
    enc_picture->bottom_ref_pic_num [LIST_0][i]=listX[LIST_0][i]->bottom_poc * 2 + 1; 
  }

  for (i=0;i<listXsize[LIST_1];i++)
  {
    enc_picture->ref_pic_num        [LIST_1][i]=listX[LIST_1][i]->poc  *2 + ((listX[LIST_1][i]->structure==BOTTOM_FIELD)?1:0);
    enc_picture->frm_ref_pic_num    [LIST_1][i]=listX[LIST_1][i]->frame_poc * 2; 
    enc_picture->top_ref_pic_num    [LIST_1][i]=listX[LIST_1][i]->top_poc * 2; 
    enc_picture->bottom_ref_pic_num [LIST_1][i]=listX[LIST_1][i]->bottom_poc * 2 + 1; 
  }

  if (img->structure==FRAME)
    for (j=2;j<6;j++)
      for (i=0;i<listXsize[j];i++)
      {    
        enc_picture->ref_pic_num[j][i] = listX[j][i]->poc * 2 + ((listX[j][i]->structure==BOTTOM_FIELD)?1:0);
        enc_picture->frm_ref_pic_num[j][i] = listX[j][i]->frame_poc * 2 ;
        enc_picture->top_ref_pic_num[j][i] = listX[j][i]->top_poc * 2 ;
        enc_picture->bottom_ref_pic_num[j][i] = listX[j][i]->bottom_poc * 2 + 1;
      }

}
