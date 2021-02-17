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
 *  \mainpage
 *     This is the H.264/AVC encoder reference software. For detailed documentation
 *     see the comments in each file.
 *
 *  \author
 *     The main contributors are listed in contributors.h
 *
 *  \version
 *     JM 8.6
 *
 *  \note
 *     tags are used for document system "doxygen"
 *     available at http://www.doxygen.org
 */
/*!
 *  \file
 *     lencod.c
 *  \brief
 *     H.264/AVC reference encoder project main()
 *  \author
 *   Main contributors (see contributors.h for copyright, address and affiliation details)
 *   - Inge Lille-Langøy               <inge.lille-langoy@telenor.com>
 *   - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *   - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *   - Jani Lainema                    <jani.lainema@nokia.com>
 *   - Byeong-Moon Jeon                <jeonbm@lge.com>
 *   - Yoon-Seong Soh                  <yunsung@lge.com>
 *   - Thomas Stockhammer              <stockhammer@ei.tum.de>
 *   - Detlev Marpe                    <marpe@hhi.de>
 *   - Guido Heising                   <heising@hhi.de>
 *   - Karsten Suehring                <suehring@hhi.de>
 ***********************************************************************
 */

#include "contributors.h"

#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <stdlib.h>
#if defined WIN32
  #include <conio.h>
#endif
#include <assert.h>

#include "global.h"
#include "configfile.h"
#include "leaky_bucket.h"
#include "memalloc.h"
#include "mbuffer.h"
#include "intrarefresh.h"
#include "fmo.h"
#include "sei.h"
#include "parset.h"
#include "image.h"
#include "output.h"
#include "fast_me.h"
#include "ratectl.h"

#define JM      "8"
#define VERSION "8.6"

InputParameters inputs, *input = &inputs;
ImageParameters images, *img   = &images;
StatParameters  stats,  *stat  = &stats;
SNRParameters   snrs,   *snr   = &snrs;
Decoders decoders, *decs=&decoders;


#ifdef _ADAPT_LAST_GROUP_
int initial_Bframes = 0;
#endif

Boolean In2ndIGOP = FALSE;
int    start_frame_no_in_this_IGOP = 0;
int    start_tr_in_this_IGOP = 0;
int    FirstFrameIn2ndIGOP=0;
int    cabac_encoding = 0;
extern ColocatedParams *Co_located;

void Init_Motion_Search_Module ();
void Clear_Motion_Search_Module ();

/*!
 ***********************************************************************
 * \brief
 *    Main function for encoder.
 * \param argc
 *    number of command line arguments
 * \param argv
 *    command line arguments
 * \return
 *    exit code
 ***********************************************************************
 */
int main(int argc,char **argv)
{
  int M,N,n,np,nb;           //Rate control
  
  p_dec = p_stat = p_log = p_trace = NULL;

  Configure (argc, argv);

  AllocNalPayloadBuffer();

  init_poc();
  GenerateParameterSets();

  init_img();

  frame_pic = malloc_picture();

  if (input->PicInterlace != FRAME_CODING)
  {
    top_pic = malloc_picture();
    bottom_pic = malloc_picture();
  }
  init_rdopt ();

  init_dpb(input);
  init_out_buffer();

  enc_picture = enc_frame_picture = enc_top_picture = enc_bottom_picture = NULL;

  init_global_buffers();
  create_context_memory ();

  Init_Motion_Search_Module ();

  information_init();

  //Rate control 
  if(input->RCEnable)
    rc_init_seq();

  if(input->FMEnable)
    DefineThreshold();

  // B pictures
  Bframe_ctr=0;
  tot_time=0;                 // time for total encoding session

#ifdef _ADAPT_LAST_GROUP_
  if (input->last_frame > 0)
    input->no_frames = 1 + (input->last_frame + input->jumpd) / (input->jumpd + 1);
  initial_Bframes = input->successive_Bframe;
#endif

  PatchInputNoFrames();

  // Write sequence header (with parameter sets)
  stat->bit_ctr_parametersets = 0;
  stat->bit_slice = start_sequence();
  stat->bit_ctr_parametersets += stat->bit_ctr_parametersets_n;
  start_frame_no_in_this_IGOP = 0;

  for (img->number=0; img->number < input->no_frames; img->number++)
  {
    img->nal_reference_idc = 1;

    //much of this can go in init_frame() or init_field()?
    //poc for this frame or field
    img->toppoc = (input->intra_period && input->idr_enable ? IMG_NUMBER % input->intra_period : IMG_NUMBER) * (2*(input->successive_Bframe+1)); 

    if ((input->PicInterlace==FRAME_CODING)&&(input->MbInterlace==FRAME_CODING))
      img->bottompoc = img->toppoc;     //progressive
    else 
      img->bottompoc = img->toppoc+1;   //hard coded

    img->framepoc = min (img->toppoc, img->bottompoc);

    //frame_num for this frame
    if (input->StoredBPictures == 0 || input->successive_Bframe == 0 || img->number < 2)
      img->frame_num = (input->intra_period && input->idr_enable ? IMG_NUMBER % input->intra_period : IMG_NUMBER) % (1 << (log2_max_frame_num_minus4 + 4)); 
    else 
    {
      img->frame_num ++;
      if (input->intra_period && input->idr_enable)
      {
        if (0== (img->number % input->intra_period))
        {
          img->frame_num=0;
        }
      }
      img->frame_num %= (1 << (log2_max_frame_num_minus4 + 4)); 
    }
    
    //the following is sent in the slice header
    img->delta_pic_order_cnt[0]=0;
    if (input->StoredBPictures)
    {
      if (img->number)
      {
        img->delta_pic_order_cnt[0]=+2 * input->successive_Bframe;
      }
    }

    SetImgType();

#ifdef _ADAPT_LAST_GROUP_
    if (input->successive_Bframe && input->last_frame && IMG_NUMBER+1 == input->no_frames)
    {                                           
      int bi = (int)((float)(input->jumpd+1)/(input->successive_Bframe+1.0)+0.499999);
      
      input->successive_Bframe = (input->last_frame-(img->number-1)*(input->jumpd+1))/bi-1;

      //about to code the last ref frame, adjust deltapoc         
      img->delta_pic_order_cnt[0]= -2*(initial_Bframes - input->successive_Bframe);
      img->toppoc += img->delta_pic_order_cnt[0];
      img->bottompoc += img->delta_pic_order_cnt[0];
    }
#endif

     //Rate control
    if (img->type == I_SLICE)
    {
      if(input->RCEnable)
      {
        if (input->intra_period == 0)
        {
          n = input->no_frames + (input->no_frames - 1) * input->successive_Bframe;
          
          /* number of P frames */
          np = input->no_frames-1; 
          
          /* number of B frames */
          nb = (input->no_frames - 1) * input->successive_Bframe;
        }else
        {
          N = input->intra_period*(input->successive_Bframe+1);
          M = input->successive_Bframe+1;
          n = (img->number==0) ? N - ( M - 1) : N;
          
          /* last GOP may contain less frames */
          if(img->number/input->intra_period >= input->no_frames / input->intra_period)
          {
            if (img->number != 0)
              n = (input->no_frames - img->number) + (input->no_frames - img->number - 1) * input->successive_Bframe + input->successive_Bframe;
            else
              n = input->no_frames  + (input->no_frames - 1) * input->successive_Bframe;
          }
          
          /* number of P frames */
          if (img->number == 0)
            np = (n + 2 * (M - 1)) / M - 1; /* first GOP */
          else
            np = (n + (M - 1)) / M - 1;
          
          /* number of B frames */
          nb = n - np - 1;
        }
        rc_init_GOP(np,nb);
      }
    }


    // which layer the image belonged to?
    if ( IMG_NUMBER % (input->NumFramesInELSubSeq+1) == 0 )
      img->layer = 0;
    else
      img->layer = 1;

    encode_one_frame(); // encode one I- or P-frame


    img->nb_references += 1;
    img->nb_references = min(img->nb_references, img->buf_cycle); // Tian Dong. PLUS1, +1, June 7, 2002

    if ((input->successive_Bframe != 0) && (IMG_NUMBER > 0)) // B-frame(s) to encode
    {
      img->type = B_SLICE;            // set image type to B-frame

      if (input->NumFramesInELSubSeq == 0) 
        img->layer = 0;
      else 
        img->layer = 1;

      if (input->StoredBPictures == 0 )
      {
        img->frame_num++;                 //increment frame_num once for B-frames
        img->frame_num %= (1 << (log2_max_frame_num_minus4 + 4));
      }
      img->nal_reference_idc = 0;     

      for(img->b_frame_to_code=1; img->b_frame_to_code<=input->successive_Bframe; img->b_frame_to_code++)
      {

        img->nal_reference_idc = 0;     
        if (input->StoredBPictures == 1 )

        {
          img->nal_reference_idc = 1;
          img->frame_num++;                 //increment frame_num once for B-frames
          img->frame_num %= (1 << (log2_max_frame_num_minus4 + 4));
        }

        //! somewhere here the disposable flag was set -- B frames are always disposable in this encoder.
        //! This happens now in slice.c, terminate_slice, where the nal_reference_idc is set up
        //poc for this B frame
        img->toppoc = 2 + (input->intra_period && input->idr_enable ? (IMG_NUMBER% input->intra_period)-1 : IMG_NUMBER-1)*(2*(input->successive_Bframe+1)) + 2* (img->b_frame_to_code-1);
  
        if ((input->PicInterlace==FRAME_CODING)&&(input->MbInterlace==FRAME_CODING))
          img->bottompoc = img->toppoc;     //progressive
        else 
          img->bottompoc = img->toppoc+1;
        
        img->framepoc = min (img->toppoc, img->bottompoc);

        //the following is sent in the slice header
        if (!input->StoredBPictures)
        {
          img->delta_pic_order_cnt[0]= 2*(img->b_frame_to_code-1);
        }
        else
        {
          img->delta_pic_order_cnt[0]= -2;
        }

        img->delta_pic_order_cnt[1]= 0;   // POC200301
        
        encode_one_frame();  // encode one B-frame
      }
    }
    
    process_2nd_IGOP();
  }
  // terminate sequence
  terminate_sequence();

  flush_dpb();

  fclose(p_in);
  if (p_dec)
    fclose(p_dec);
  if (p_trace)
    fclose(p_trace);

  Clear_Motion_Search_Module ();

  RandomIntraUninit();
  FmoUninit();
  
  // free structure for rd-opt. mode decision
  clear_rdopt ();

#ifdef _LEAKYBUCKET_
  calc_buffer();
#endif

  // report everything
  report();

  free_picture (frame_pic);
  if (top_pic)
    free_picture (top_pic);
  if (bottom_pic)
    free_picture (bottom_pic);

  free_dpb();
  free_collocated(Co_located);
  uninit_out_buffer();

  free_global_buffers();

  // free image mem
  free_img ();
  free_context_memory ();
  FreeNalPayloadBuffer();
  FreeParameterSets();
  return 0;                         //encode JM73_FME version
}
/*!
 ***********************************************************************
 * \brief
 *    Terminates and reports statistics on error.
 * 
 ***********************************************************************
 */
void report_stats_on_error()
{
  input->no_frames=img->number-1;
  terminate_sequence();
  
  flush_dpb();
  
  fclose(p_in);
  if (p_dec)
    fclose(p_dec);
  if (p_trace)
    fclose(p_trace);
  
  Clear_Motion_Search_Module ();
  
  RandomIntraUninit();
  FmoUninit();
  
  // free structure for rd-opt. mode decision
  clear_rdopt ();
  
#ifdef _LEAKYBUCKET_
  calc_buffer();
#endif
  
  // report everything
  report();
  
  free_picture (frame_pic);
  if (top_pic)
    free_picture (top_pic);
  if (bottom_pic)
    free_picture (bottom_pic);
  
  free_dpb();
  free_collocated(Co_located);
  uninit_out_buffer();
  
  free_global_buffers();
  
  // free image mem
  free_img ();
  free_context_memory ();
  FreeNalPayloadBuffer();
  FreeParameterSets();
}

/*!
 ***********************************************************************
 * \brief
 *    Initializes the POC structure with appropriate parameters.
 * 
 ***********************************************************************
 */
void init_poc()
{
//  if(input->no_frames > (1<<15))error("too many frames",-998);

  //the following should probably go in sequence parameters
  // frame poc's increase by 2, field poc's by 1

  img->pic_order_cnt_type=input->pic_order_cnt_type;

  img->delta_pic_order_always_zero_flag=0;
  img->num_ref_frames_in_pic_order_cnt_cycle= 1;

  if (input->StoredBPictures)
  {
    img->offset_for_non_ref_pic  =  0;
    img->offset_for_ref_frame[0] =   2;
  }
  else
  {
    img->offset_for_non_ref_pic  =  -2*(input->successive_Bframe);
    img->offset_for_ref_frame[0] =   2*(input->successive_Bframe+1);
  }

  if ((input->PicInterlace==FRAME_CODING)&&(input->MbInterlace==FRAME_CODING))
    img->offset_for_top_to_bottom_field=0;
  else    
    img->offset_for_top_to_bottom_field=1;

                                    //the following should probably go in picture parameters
//  img->pic_order_present_flag=0;    //img->delta_pic_order_cnt[1] not sent
  // POC200301
  if ((input->PicInterlace==FRAME_CODING)&&(input->MbInterlace==FRAME_CODING))
  {
    img->pic_order_present_flag=0;
    img->delta_pic_order_cnt_bottom = 0;
  }
  else    
  {
    img->pic_order_present_flag=1;
    img->delta_pic_order_cnt_bottom = 1;
  }
}


/*!
 ***********************************************************************
 * \brief
 *    Initializes the img->nz_coeff
 * \par Input:
 *    none
 * \par  Output:
 *    none
 * \ side effects
 *    sets omg->nz_coef[][][][] to -1
 ***********************************************************************
 */
void CAVLC_init()
{
  unsigned int i, k, l;

  for (i=0;i < img->PicSizeInMbs; i++)
    for (k=0;k<4;k++)
      for (l=0;l<6;l++)
        img->nz_coeff[i][k][l]=-1;


}


/*!
 ***********************************************************************
 * \brief
 *    Initializes the Image structure with appropriate parameters.
 * \par Input:
 *    Input Parameters struct inp_par *inp
 * \par  Output:
 *    Image Parameters struct img_par *img
 ***********************************************************************
 */
void init_img()
{
  int i,j;

  img->num_reference_frames = active_sps->num_ref_frames;
  img->max_num_references   = active_sps->frame_mbs_only_flag ? active_sps->num_ref_frames : 2 * active_sps->num_ref_frames;
  
  img->buf_cycle = input->num_reference_frames;

  img->DeblockCall = 0;
    
//  img->framerate=INIT_FRAME_RATE;   // The basic frame rate (of the original sequence)
  img->framerate=input->FrameRate;   // The basic frame rate (of the original sequence)


  get_mem_mv (&(img->pred_mv));
  get_mem_mv (&(img->all_mv));

  get_mem_ACcoeff (&(img->cofAC));
  get_mem_DCcoeff (&(img->cofDC));

  if(input->MbInterlace) 
  {
    get_mem_mv (&(rddata_top_frame_mb.pred_mv));
    get_mem_mv (&(rddata_top_frame_mb.all_mv));

    get_mem_mv (&(rddata_bot_frame_mb.pred_mv));
    get_mem_mv (&(rddata_bot_frame_mb.all_mv));

    get_mem_mv (&(rddata_top_field_mb.pred_mv));
    get_mem_mv (&(rddata_top_field_mb.all_mv));

    get_mem_mv (&(rddata_bot_field_mb.pred_mv));
    get_mem_mv (&(rddata_bot_field_mb.all_mv));

    get_mem_ACcoeff (&(rddata_top_frame_mb.cofAC));
    get_mem_DCcoeff (&(rddata_top_frame_mb.cofDC));

    get_mem_ACcoeff (&(rddata_bot_frame_mb.cofAC));
    get_mem_DCcoeff (&(rddata_bot_frame_mb.cofDC));

    get_mem_ACcoeff (&(rddata_top_field_mb.cofAC));
    get_mem_DCcoeff (&(rddata_top_field_mb.cofDC));

    get_mem_ACcoeff (&(rddata_bot_field_mb.cofAC));
    get_mem_DCcoeff (&(rddata_bot_field_mb.cofDC));
  }

  if ((img->quad = (int*)calloc (511, sizeof(int))) == NULL)
    no_mem_exit ("init_img: img->quad");
  img->quad+=255;
  for (i=0; i < 256; ++i)
  {
    img->quad[i]=img->quad[-i]=i*i;
  }

  img->width    = input->img_width;
  img->height   = input->img_height;
  img->width_cr = input->img_width/2;
  img->height_cr= input->img_height/2;
  
  img->PicWidthInMbs    = input->img_width/MB_BLOCK_SIZE;
  img->FrameHeightInMbs = input->img_height/MB_BLOCK_SIZE;
  img->FrameSizeInMbs   = img->PicWidthInMbs * img->FrameHeightInMbs;

  img->PicHeightInMapUnits = ( active_sps->frame_mbs_only_flag ? img->FrameHeightInMbs : img->FrameHeightInMbs/2 );
  

  if(((img->mb_data) = (Macroblock *) calloc(img->FrameSizeInMbs,sizeof(Macroblock))) == NULL)
    no_mem_exit("init_img: img->mb_data");

  if(input->UseConstrainedIntraPred)
  {
    if(((img->intra_block) = (int*)calloc(img->FrameSizeInMbs,sizeof(int))) == NULL)
      no_mem_exit("init_img: img->intra_block");
  }

  get_mem2Dint(&(img->ipredmode), img->width/BLOCK_SIZE, img->height/BLOCK_SIZE);        //need two extra rows at right and bottom

  if(input->MbInterlace) 
  {
    get_mem2Dint(&(rddata_top_frame_mb.ipredmode), img->width/BLOCK_SIZE, img->height/BLOCK_SIZE);
    get_mem2Dint(&(rddata_bot_frame_mb.ipredmode), img->width/BLOCK_SIZE, img->height/BLOCK_SIZE);
    get_mem2Dint(&(rddata_top_field_mb.ipredmode), img->width/BLOCK_SIZE, img->height/BLOCK_SIZE);
    get_mem2Dint(&(rddata_bot_field_mb.ipredmode), img->width/BLOCK_SIZE, img->height/BLOCK_SIZE);
  }
  // CAVLC mem
  get_mem3Dint(&(img->nz_coeff), img->FrameSizeInMbs, 4, 6);

  CAVLC_init();

  for (i=0; i < img->width/BLOCK_SIZE; i++)
    for (j=0; j < img->height/BLOCK_SIZE; j++)
    {
      img->ipredmode[i][j]=-1;
    }

  img->mb_y_upd=0;

  RandomIntraInit (img->width/16, img->height/16, input->RandomIntraMBRefresh);

  InitSEIMessages();  // Tian Dong (Sept 2002)

  // Initialize filtering parameters. If sending parameters, the offsets are 
  // multiplied by 2 since inputs are taken in "div 2" format.
  // If not sending paramters, all fields are cleared 
  if (input->LFSendParameters)
  {
    input->LFAlphaC0Offset <<= 1;
    input->LFBetaOffset <<= 1;
  }
  else
  {
    input->LFDisableIdc = 0;
    input->LFAlphaC0Offset = 0;
    input->LFBetaOffset = 0;
  }
}

/*!
 ***********************************************************************
 * \brief
 *    Free the Image structures
 * \par Input:
 *    Image Parameters struct img_par *img
 ***********************************************************************
 */
void free_img ()
{
  CloseSEIMessages(); // Tian Dong (Sept 2002)
  free_mem_mv (img->pred_mv);
  free_mem_mv (img->all_mv);

  free_mem_ACcoeff (img->cofAC);
  free_mem_DCcoeff (img->cofDC);

  free (img->quad-255);
}



/*!
 ************************************************************************
 * \brief
 *    Allocates the picture structure along with its dependent
 *    data structures
 * \return
 *    Pointer to a Picture
 ************************************************************************
 */

Picture *malloc_picture()
{
  Picture *pic;
  if ((pic = calloc (1, sizeof (Picture))) == NULL) no_mem_exit ("malloc_picture: Picture structure");
  //! Note: slice structures are allocated as needed in code_a_picture
  return pic;
}

/*!
 ************************************************************************
 * \brief
 *    Frees a picture
 * \param
 *    pic: POinter to a Picture to be freed
 ************************************************************************
 */


void free_picture(Picture *pic)
{
  if (pic != NULL)
  {
    free_slice_list(pic);
    free (pic);
  }
}




/*!
 ************************************************************************
 * \brief
 *    Reports the gathered information to appropriate outputs
 * \par Input:
 *    struct inp_par *inp,                                            \n
 *    struct img_par *img,                                            \n
 *    struct stat_par *stat,                                          \n
 *    struct stat_par *stat                                           \n
 *
 * \par Output:
 *    None
 ************************************************************************
 */
void report()
{
  int bit_use[NUM_PIC_TYPE][2] ;
  int i,j;
  char name[20];
  int total_bits;
  float frame_rate;
  float mean_motion_info_bit_use[2];

#ifndef WIN32
  time_t now;
  struct tm *l_time;
  char string[1000];
#else
  char timebuf[128];
#endif
  bit_use[I_SLICE][0] = 1;
  bit_use[P_SLICE][0] = max(1,input->no_frames-1);
  bit_use[B_SLICE][0] = Bframe_ctr;

  //  Accumulate bit usage for inter and intra frames
  for (j=0;j<NUM_PIC_TYPE;j++)
  {
    bit_use[j][1] = 0;
  }

  for (j=0;j<NUM_PIC_TYPE;j++)
  {
    for(i=0; i<MAXMODE; i++)
      bit_use[j][1] += stat->bit_use_mode    [j][i]; 

    bit_use[j][1]+=stat->bit_use_header      [j];
    bit_use[j][1]+=stat->bit_use_mb_type     [j];
    bit_use[j][1]+=stat->tmp_bit_use_cbp     [j];
    bit_use[j][1]+=stat->bit_use_coeffY      [j];
    bit_use[j][1]+=stat->bit_use_coeffC      [j];
    bit_use[j][1]+=stat->bit_use_delta_quant [j];
    bit_use[j][1]+=stat->bit_use_stuffingBits[j];
  }

  // B pictures
  if(Bframe_ctr!=0)
  {
    frame_rate = (float)(img->framerate *(input->successive_Bframe + 1)) / (float) (input->jumpd+1);

//! Currently adding NVB bits on P rate. Maybe additional stat info should be created instead and added in log file
//    stat->bitrate_P=(stat->bit_ctr_0+stat->bit_ctr_P)*(float)(frame_rate)/(float) (input->no_frames + Bframe_ctr);
    stat->bitrate_P=(stat->bit_ctr_0+stat->bit_ctr_P + stat->bit_ctr_parametersets)*(float)(frame_rate)/(float) (input->no_frames + Bframe_ctr);

#ifdef _ADAPT_LAST_GROUP_
    stat->bitrate_B=(stat->bit_ctr_B)*(float)(frame_rate)/(float) (input->no_frames + Bframe_ctr);    
#else
    stat->bitrate_B=(stat->bit_ctr_B)*(float)(frame_rate)/(float) (input->no_frames + Bframe_ctr);    
#endif

  }
  else
  {
    if (input->no_frames > 1)
    {
      stat->bitrate=(bit_use[I_SLICE][1]+bit_use[P_SLICE][1])*(float)img->framerate/(input->no_frames*(input->jumpd+1));
    }
  }

  fprintf(stdout,"-------------------------------------------------------------------------------\n");
  fprintf(stdout,   " Freq. for encoded bitstream       : %1.0f\n",(float)img->framerate/(float)(input->jumpd+1));
  if(input->hadamard)
    fprintf(stdout," Hadamard transform                : Used\n");
  else
    fprintf(stdout," Hadamard transform                : Not used\n");

  fprintf(stdout," Image format                      : %dx%d\n",input->img_width,input->img_height);

  if(input->intra_upd)
    fprintf(stdout," Error robustness                  : On\n");
  else
    fprintf(stdout," Error robustness                  : Off\n");
  fprintf(stdout,    " Search range                      : %d\n",input->search_range);


  fprintf(stdout,   " No of ref. frames used in P pred  : %d\n",input->num_reference_frames);
  if(input->successive_Bframe != 0)
    fprintf(stdout,   " No of ref. frames used in B pred  : %d\n",input->num_reference_frames);
  fprintf(stdout,   " Total encoding time for the seq.  : %.3f sec \n",tot_time*0.001);
  fprintf(stdout,   " Total ME time for sequence        : %.3f sec \n",me_tot_time*0.001);

  // B pictures
  fprintf(stdout, " Sequence type                     :" );
  if(input->successive_Bframe==0 && input->StoredBPictures > 0)
    fprintf(stdout, " I-P-BS-BS (QP: I %d, P BS %d) \n",input->qp0, input->qpN);
  else if(input->successive_Bframe==1 && input->StoredBPictures > 0)
    fprintf(stdout, " I-B-P-BS-B-BS (QP: I %d, P BS %d, B %d) \n",input->qp0, input->qpN, input->qpB);
  else if(input->successive_Bframe==2 && input->StoredBPictures > 0)
    fprintf(stdout, " I-B-B-P-B-B-BS (QP: I %d, P BS %d, B %d) \n",input->qp0, input->qpN, input->qpB);
  else if(input->successive_Bframe==1)   fprintf(stdout, " IBPBP (QP: I %d, P %d, B %d) \n",
    input->qp0, input->qpN, input->qpB);
  else if(input->successive_Bframe==2) fprintf(stdout, " IBBPBBP (QP: I %d, P %d, B %d) \n",
    input->qp0, input->qpN, input->qpB);
  else if(input->successive_Bframe==0 && input->sp_periodicity==0) fprintf(stdout, " IPPP (QP: I %d, P %d) \n",   input->qp0, input->qpN);

  else fprintf(stdout, " I-P-P-SP-P (QP: I %d, P %d, SP (%d, %d)) \n",  input->qp0, input->qpN, input->qpsp, input->qpsp_pred);

  // report on entropy coding  method
  if (input->symbol_mode == UVLC)
    fprintf(stdout," Entropy coding method             : CAVLC\n");
  else
    fprintf(stdout," Entropy coding method             : CABAC\n");

  fprintf(stdout,  " Profile/Level IDC                 : (%d,%d)\n",input->ProfileIDC,input->LevelIDC);
#ifdef _FULL_SEARCH_RANGE_
  if (input->full_search == 2)
    fprintf(stdout," Search range restrictions         : none\n");
  else if (input->full_search == 1)
    fprintf(stdout," Search range restrictions         : older reference frames\n");
  else
    fprintf(stdout," Search range restrictions         : smaller blocks and older reference frames\n");
#endif

  if (input->rdopt)
    fprintf(stdout," RD-optimized mode decision        : used\n");
  else
    fprintf(stdout," RD-optimized mode decision        : not used\n");

  switch(input->partition_mode)
  {
  case PAR_DP_1:
    fprintf(stdout," Data Partitioning Mode            : 1 partition \n");
    break;
  case PAR_DP_3:
    fprintf(stdout," Data Partitioning Mode            : 3 partitions \n");
    break;
  default:
    fprintf(stdout," Data Partitioning Mode            : not supported\n");
    break;
  }

  switch(input->of_mode)
  {
  case PAR_OF_ANNEXB:
    fprintf(stdout," Output File Format                : H.264 Bit Stream File Format \n");
    break;
  case PAR_OF_RTP:
    fprintf(stdout," Output File Format                : RTP Packet File Format \n");
    break;
  default:
    fprintf(stdout," Output File Format                : not supported\n");
    break;
  }



  fprintf(stdout,"------------------ Average data all frames  -----------------------------------\n");
  fprintf(stdout," SNR Y(dB)                         : %5.2f\n",snr->snr_ya);
  fprintf(stdout," SNR U(dB)                         : %5.2f\n",snr->snr_ua);
  fprintf(stdout," SNR V(dB)                         : %5.2f\n",snr->snr_va);

  if(Bframe_ctr!=0)
  {

//      fprintf(stdout, " Total bits                        : %d (I %5d, P %5d, B %d) \n",
//            total_bits=stat->bit_ctr_P + stat->bit_ctr_0 + stat->bit_ctr_B, stat->bit_ctr_0, stat->bit_ctr_P, stat->bit_ctr_B);
      fprintf(stdout, " Total bits                        : %d (I %5d, P %5d, B %d NVB %d) \n",
            total_bits=stat->bit_ctr_P + stat->bit_ctr_0 + stat->bit_ctr_B + stat->bit_ctr_parametersets, stat->bit_ctr_0, stat->bit_ctr_P, stat->bit_ctr_B,stat->bit_ctr_parametersets);

    frame_rate = (float)(img->framerate *(input->successive_Bframe + 1)) / (float) (input->jumpd+1);
    stat->bitrate= ((float) total_bits * frame_rate)/((float) (input->no_frames + Bframe_ctr));

    fprintf(stdout, " Bit rate (kbit/s)  @ %2.2f Hz     : %5.2f\n", frame_rate, stat->bitrate/1000);

  }
  else if (input->sp_periodicity==0)
  {
//    fprintf(stdout, " Total bits                        : %d (I %5d, P %5d) \n",
//    total_bits=stat->bit_ctr_P + stat->bit_ctr_0 , stat->bit_ctr_0, stat->bit_ctr_P);
      fprintf(stdout, " Total bits                        : %d (I %5d, P %5d, NVB %d) \n",
      total_bits=stat->bit_ctr_P + stat->bit_ctr_0 + stat->bit_ctr_parametersets, stat->bit_ctr_0, stat->bit_ctr_P, stat->bit_ctr_parametersets);


    frame_rate = (float)img->framerate / ( (float) (input->jumpd + 1) );
    stat->bitrate= ((float) total_bits * frame_rate)/((float) input->no_frames );

    fprintf(stdout, " Bit rate (kbit/s)  @ %2.2f Hz     : %5.2f\n", frame_rate, stat->bitrate/1000);
  }else
  {
    //fprintf(stdout, " Total bits                        : %d (I %5d, P %5d) \n",
    //total_bits=stat->bit_ctr_P + stat->bit_ctr_0 , stat->bit_ctr_0, stat->bit_ctr_P);
      fprintf(stdout, " Total bits                        : %d (I %5d, P %5d, NVB %d) \n",
      total_bits=stat->bit_ctr_P + stat->bit_ctr_0 + stat->bit_ctr_parametersets, stat->bit_ctr_0, stat->bit_ctr_P, stat->bit_ctr_parametersets);


    frame_rate = (float)img->framerate / ( (float) (input->jumpd + 1) );
    stat->bitrate= ((float) total_bits * frame_rate)/((float) input->no_frames );

    fprintf(stdout, " Bit rate (kbit/s)  @ %2.2f Hz     : %5.2f\n", frame_rate, stat->bitrate/1000);
  }

  fprintf(stdout, " Bits to avoid Startcode Emulation : %d \n", stat->bit_ctr_emulationprevention);
  fprintf(stdout, " Bits for parameter sets           : %d \n", stat->bit_ctr_parametersets);

  fprintf(stdout,"-------------------------------------------------------------------------------\n");
  fprintf(stdout,"Exit JM %s encoder ver %s ", JM, VERSION);
  fprintf(stdout,"\n");

  // status file
  if ((p_stat=fopen("stat.dat","wt"))==0)
  {
    snprintf(errortext, ET_SIZE, "Error open file %s", "stat.dat");
    error(errortext, 500);
  }
  fprintf(p_stat," -------------------------------------------------------------- \n");
  fprintf(p_stat,"  This file contains statistics for the last encoded sequence   \n");
  fprintf(p_stat," -------------------------------------------------------------- \n");
  fprintf(p_stat,   " Sequence                     : %s\n",input->infile);
  fprintf(p_stat,   " No.of coded pictures         : %4d\n",input->no_frames+Bframe_ctr);
  fprintf(p_stat,   " Freq. for encoded bitstream  : %4.0f\n",frame_rate);

  // B pictures
  if(input->successive_Bframe != 0)
  {
    fprintf(p_stat,   " BaseLayer Bitrate(kb/s)      : %6.2f\n", stat->bitrate_P/1000);
    fprintf(p_stat,   " EnhancedLayer Bitrate(kb/s)  : %6.2f\n", stat->bitrate_B/1000);
  }
  else
    fprintf(p_stat,   " Bitrate(kb/s)                : %6.2f\n", stat->bitrate/1000);

  if(input->hadamard)
    fprintf(p_stat," Hadamard transform           : Used\n");
  else
    fprintf(p_stat," Hadamard transform           : Not used\n");

  fprintf(p_stat,  " Image format                 : %dx%d\n",input->img_width,input->img_height);

  if(input->intra_upd)
    fprintf(p_stat," Error robustness             : On\n");
  else
    fprintf(p_stat," Error robustness             : Off\n");

  fprintf(p_stat,  " Search range                 : %d\n",input->search_range);

  fprintf(p_stat,   " No of frame used in P pred   : %d\n",input->num_reference_frames);
  if(input->successive_Bframe != 0)
    fprintf(p_stat, " No of frame used in B pred   : %d\n",input->num_reference_frames);

  if (input->symbol_mode == UVLC)
    fprintf(p_stat,   " Entropy coding method        : CAVLC\n");
  else
    fprintf(p_stat,   " Entropy coding method        : CABAC\n");

    fprintf(p_stat,   " Profile/Level IDC            : (%d,%d)\n",input->ProfileIDC,input->LevelIDC);
  if(input->MbInterlace)
    fprintf(p_stat, " MB Field Coding : On \n");

#ifdef _FULL_SEARCH_RANGE_
  if (input->full_search == 2)
    fprintf(p_stat," Search range restrictions    : none\n");
  else if (input->full_search == 1)
    fprintf(p_stat," Search range restrictions    : older reference frames\n");
  else
    fprintf(p_stat," Search range restrictions    : smaller blocks and older reference frames\n");
#endif
  if (input->rdopt)
    fprintf(p_stat," RD-optimized mode decision   : used\n");
  else
    fprintf(p_stat," RD-optimized mode decision   : not used\n");

  fprintf(p_stat," ---------------------|----------------|---------------|\n");
  fprintf(p_stat,"     Item             |      Intra     |   All frames  |\n");
  fprintf(p_stat," ---------------------|----------------|---------------|\n");
  fprintf(p_stat," SNR Y(dB)            |");
  fprintf(p_stat,"  %5.2f         |",snr->snr_y1);
  fprintf(p_stat," %5.2f         |\n",snr->snr_ya);
  fprintf(p_stat," SNR U/V (dB)         |");
  fprintf(p_stat,"  %5.2f/%5.2f   |",snr->snr_u1,snr->snr_v1);
  fprintf(p_stat," %5.2f/%5.2f   |\n",snr->snr_ua,snr->snr_va);

  // QUANT.
  fprintf(p_stat," Average quant        |");
  fprintf(p_stat,"  %5d         |",absm(input->qp0));
  fprintf(p_stat," %5.2f         |\n",(float)stat->quant1/max(1.0,(float)stat->quant0));

  // MODE
  fprintf(p_stat,"\n ---------------------|----------------|\n");
  fprintf(p_stat,"   Intra              |    Mode used   |\n");
  fprintf(p_stat," ---------------------|----------------|\n");

  fprintf(p_stat," Mode 0  intra 4x4    |  %5d         |\n",stat->mode_use[I_SLICE][I4MB]);
  fprintf(p_stat," Mode 1+ intra 16x16  |  %5d         |\n",stat->mode_use[I_SLICE][I16MB]);

  fprintf(p_stat,"\n ---------------------|----------------|-----------------|\n");
  fprintf(p_stat,"   Inter              |    Mode used   | MotionInfo bits |\n");
  fprintf(p_stat," ---------------------|----------------|-----------------|");
  fprintf(p_stat,"\n Mode  0  (copy)      |  %5d         |    %8.2f     |",stat->mode_use[P_SLICE][0   ],(float)stat->bit_use_mode[P_SLICE][0   ]/(float)bit_use[P_SLICE][0]);
  fprintf(p_stat,"\n Mode  1  (16x16)     |  %5d         |    %8.2f     |",stat->mode_use[P_SLICE][1   ],(float)stat->bit_use_mode[P_SLICE][1   ]/(float)bit_use[P_SLICE][0]);
  fprintf(p_stat,"\n Mode  2  (16x8)      |  %5d         |    %8.2f     |",stat->mode_use[P_SLICE][2   ],(float)stat->bit_use_mode[P_SLICE][2   ]/(float)bit_use[P_SLICE][0]);
  fprintf(p_stat,"\n Mode  3  (8x16)      |  %5d         |    %8.2f     |",stat->mode_use[P_SLICE][3   ],(float)stat->bit_use_mode[P_SLICE][3   ]/(float)bit_use[P_SLICE][0]);
  fprintf(p_stat,"\n Mode  4  (8x8)       |  %5d         |    %8.2f     |",stat->mode_use[P_SLICE][P8x8],(float)stat->bit_use_mode[P_SLICE][P8x8]/(float)bit_use[P_SLICE][0]);
  fprintf(p_stat,"\n Mode  5  intra 4x4   |  %5d         |-----------------|",stat->mode_use[P_SLICE][I4MB]);
  fprintf(p_stat,"\n Mode  6+ intra 16x16 |  %5d         |",stat->mode_use[P_SLICE][I16MB]);
  mean_motion_info_bit_use[0] = (float)(stat->bit_use_mode[P_SLICE][0] + stat->bit_use_mode[P_SLICE][1] + stat->bit_use_mode[P_SLICE][2] 
                                      + stat->bit_use_mode[P_SLICE][3] + stat->bit_use_mode[P_SLICE][P8x8])/(float) bit_use[P_SLICE][0]; 

  // B pictures
  if(input->successive_Bframe!=0 && Bframe_ctr!=0)
  {
 
    fprintf(p_stat,"\n\n ---------------------|----------------|-----------------|\n");
    fprintf(p_stat,"   B frame            |    Mode used   | MotionInfo bits |\n");
    fprintf(p_stat," ---------------------|----------------|-----------------|");
    fprintf(p_stat,"\n Mode  0  (copy)      |  %5d         |    %8.2f     |",stat->mode_use[B_SLICE][0   ],(float)stat->bit_use_mode[B_SLICE][0   ]/(float)Bframe_ctr);
    fprintf(p_stat,"\n Mode  1  (16x16)     |  %5d         |    %8.2f     |",stat->mode_use[B_SLICE][1   ],(float)stat->bit_use_mode[B_SLICE][1   ]/(float)Bframe_ctr);
    fprintf(p_stat,"\n Mode  2  (16x8)      |  %5d         |    %8.2f     |",stat->mode_use[B_SLICE][2   ],(float)stat->bit_use_mode[B_SLICE][2   ]/(float)Bframe_ctr);
    fprintf(p_stat,"\n Mode  3  (8x16)      |  %5d         |    %8.2f     |",stat->mode_use[B_SLICE][3   ],(float)stat->bit_use_mode[B_SLICE][3   ]/(float)Bframe_ctr);
    fprintf(p_stat,"\n Mode  4  (8x8)       |  %5d         |    %8.2f     |",stat->mode_use[B_SLICE][P8x8],(float)stat->bit_use_mode[B_SLICE][P8x8]/(float)Bframe_ctr);
    fprintf(p_stat,"\n Mode  5  intra 4x4   |  %5d         |-----------------|",stat->mode_use[B_SLICE][I4MB]);
    fprintf(p_stat,"\n Mode  6+ intra 16x16 |  %5d         |",stat->mode_use[B_SLICE][I16MB]);
    mean_motion_info_bit_use[1] = (float)(stat->bit_use_mode[B_SLICE][0] + stat->bit_use_mode[B_SLICE][1] + stat->bit_use_mode[B_SLICE][2] 
                                      + stat->bit_use_mode[B_SLICE][3] + stat->bit_use_mode[B_SLICE][P8x8])/(float) Bframe_ctr; 

  }

  fprintf(p_stat,"\n\n ---------------------|----------------|----------------|----------------|\n");
  fprintf(p_stat,"  Bit usage:          |      Intra     |      Inter     |    B frame     |\n");
  fprintf(p_stat," ---------------------|----------------|----------------|----------------|\n");

  fprintf(p_stat," Header               |");
  fprintf(p_stat," %10.2f     |",(float) stat->bit_use_header[I_SLICE]/bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |",(float) stat->bit_use_header[P_SLICE]/bit_use[P_SLICE][0]);
  if(input->successive_Bframe!=0 && Bframe_ctr!=0)
    fprintf(p_stat," %10.2f      |",(float) stat->bit_use_header[B_SLICE]/Bframe_ctr);
  else fprintf(p_stat," %10.2f      |", 0.);
  fprintf(p_stat,"\n");

  fprintf(p_stat," Mode                 |");
  fprintf(p_stat," %10.2f     |",(float)stat->bit_use_mb_type[I_SLICE]/bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |",(float)stat->bit_use_mb_type[P_SLICE]/bit_use[P_SLICE][0]);
  if(input->successive_Bframe!=0 && Bframe_ctr!=0)
    fprintf(p_stat," %10.2f     |",(float)stat->bit_use_mb_type[B_SLICE]/Bframe_ctr);
  else fprintf(p_stat," %10.2f     |", 0.);
  fprintf(p_stat,"\n");

  fprintf(p_stat," Motion Info          |");
  fprintf(p_stat,"        ./.     |");
  fprintf(p_stat," %10.2f     |",mean_motion_info_bit_use[0]);
  if(input->successive_Bframe!=0 && Bframe_ctr!=0)
    fprintf(p_stat," %10.2f     |",mean_motion_info_bit_use[1]);
  else fprintf(p_stat," %10.2f     |", 0.);
  fprintf(p_stat,"\n");

  fprintf(p_stat," CBP Y/C              |");
  fprintf(p_stat," %10.2f     |", (float)stat->tmp_bit_use_cbp[I_SLICE]/bit_use[I_SLICE][0]);
  fprintf(p_stat," %10.2f     |", (float)stat->tmp_bit_use_cbp[P_SLICE]/bit_use[P_SLICE][0]);
  if(input->successive_Bframe!=0 && Bframe_ctr!=0)
    fprintf(p_stat," %10.2f     |", (float)stat->tmp_bit_use_cbp[B_SLICE]/Bframe_ctr);
  else fprintf(p_stat," %10.2f     |", 0.);
  fprintf(p_stat,"\n");

  if(input->successive_Bframe!=0 && Bframe_ctr!=0)
    fprintf(p_stat," Coeffs. Y            | %10.2f     | %10.2f     | %10.2f     |\n",
    (float)stat->bit_use_coeffY[I_SLICE]/bit_use[I_SLICE][0], (float)stat->bit_use_coeffY[P_SLICE]/bit_use[P_SLICE][0], (float)stat->bit_use_coeffY[B_SLICE]/Bframe_ctr);
  else
    fprintf(p_stat," Coeffs. Y            | %10.2f     | %10.2f     | %10.2f     |\n",
      (float)stat->bit_use_coeffY[I_SLICE]/bit_use[I_SLICE][0], (float)stat->bit_use_coeffY[P_SLICE]/(float)bit_use[P_SLICE][0], 0.);

  if(input->successive_Bframe!=0 && Bframe_ctr!=0)
    fprintf(p_stat," Coeffs. C            | %10.2f     | %10.2f     | %10.2f     |\n",
      (float)stat->bit_use_coeffC[I_SLICE]/bit_use[I_SLICE][0], (float)stat->bit_use_coeffC[P_SLICE]/bit_use[P_SLICE][0], (float)stat->bit_use_coeffC[B_SLICE]/Bframe_ctr);
  else
    fprintf(p_stat," Coeffs. C            | %10.2f     | %10.2f     | %10.2f     |\n",
      (float)stat->bit_use_coeffC[I_SLICE]/bit_use[I_SLICE][0], (float)stat->bit_use_coeffC[P_SLICE]/bit_use[P_SLICE][0], 0.);

  if(input->successive_Bframe!=0 && Bframe_ctr!=0)
    fprintf(p_stat," Delta quant          | %10.2f     | %10.2f     | %10.2f     |\n",
      (float)stat->bit_use_delta_quant[I_SLICE]/bit_use[I_SLICE][0], (float)stat->bit_use_delta_quant[P_SLICE]/bit_use[P_SLICE][0], (float)stat->bit_use_delta_quant[B_SLICE]/Bframe_ctr);
  else
    fprintf(p_stat," Delta quant          | %10.2f     | %10.2f     | %10.2f     |\n",
      (float)stat->bit_use_delta_quant[I_SLICE]/bit_use[I_SLICE][0], (float)stat->bit_use_delta_quant[P_SLICE]/bit_use[P_SLICE][0], 0.);

  if(input->successive_Bframe!=0 && Bframe_ctr!=0)
    fprintf(p_stat," Stuffing Bits        | %10.2f     | %10.2f     | %10.2f     |\n",
      (float)stat->bit_use_stuffingBits[I_SLICE]/bit_use[I_SLICE][0], (float)stat->bit_use_stuffingBits[P_SLICE]/bit_use[P_SLICE][0], (float)stat->bit_use_stuffingBits[B_SLICE]/Bframe_ctr);
  else
    fprintf(p_stat," Stuffing Bits        | %10.2f     | %10.2f     | %10.2f     |\n",
      (float)stat->bit_use_stuffingBits[I_SLICE]/bit_use[I_SLICE][0], (float)stat->bit_use_stuffingBits[P_SLICE]/bit_use[P_SLICE][0], 0.);



  fprintf(p_stat," ---------------------|----------------|----------------|----------------|\n");

  fprintf(p_stat," average bits/frame   |");

  fprintf(p_stat," %10.2f     |", (float) bit_use[I_SLICE][1]/(float) bit_use[I_SLICE][0] );
  fprintf(p_stat," %10.2f     |", (float) bit_use[P_SLICE][1]/(float) bit_use[P_SLICE][0] );
  
  if(input->successive_Bframe!=0 && Bframe_ctr!=0)
    fprintf(p_stat," %10.2f     |", (float) bit_use[B_SLICE][1]/ (float) Bframe_ctr );
  else fprintf(p_stat," %10.2f     |", 0.);

  fprintf(p_stat,"\n");
  fprintf(p_stat," ---------------------|----------------|----------------|----------------|\n");

  fclose(p_stat);

  // write to log file
  if ((p_log=fopen("log.dat","r"))==0)                      // check if file exist
  {
    if ((p_log=fopen("log.dat","a"))==NULL)            // append new statistic at the end
    {
      snprintf(errortext, ET_SIZE, "Error open file %s  \n","log.dat");
      error(errortext, 500);
    }
    else                                            // Create header for new log file
    {
      fprintf(p_log," ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ \n");
      fprintf(p_log,"|            Encoder statistics. This file is generated during first encoding session, new sessions will be appended                                                                                          |\n");
      fprintf(p_log," ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ \n");
      fprintf(p_log,"| Date  | Time  |    Sequence        | #Img | QPI| QPP| QPB| Format  | #B | Hdmd | S.R |#Ref | Freq |Intra upd|SNRY 1|SNRU 1|SNRV 1|SNRY N|SNRU N|SNRV N|#Bitr P|#Bitr B|     Total Time   |      Me Time     |\n");
      fprintf(p_log," ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ \n");
/*
      fprintf(p_log," ----------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
      fprintf(p_log,"|            Encoder statistics. This file is generated during first encoding session, new sessions will be appended                                              |\n");
      fprintf(p_log," ----------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
      fprintf(p_log,"| Date  | Time  |    Sequence        |#Img|Quant1|QuantN|Format|Hadamard|Search r|#Ref | Freq |Intra upd|SNRY 1|SNRU 1|SNRV 1|SNRY N|SNRU N|SNRV N|#Bitr P|#Bitr B|\n");
      fprintf(p_log," ----------------------------------------------------------------------------------------------------------------------------------------------------------------- \n");
*/
    }
  }
  else
  {
    fclose (p_log);
    if ((p_log=fopen("log.dat","a"))==NULL)            // File exist,just open for appending
    {
      snprintf(errortext, ET_SIZE, "Error open file %s  \n","log.dat");
      error(errortext, 500);
    }
  }

#ifdef WIN32
  _strdate( timebuf );
  fprintf(p_log,"| %1.5s |",timebuf );

  _strtime( timebuf);
  fprintf(p_log," % 1.5s |",timebuf);
#else
  now = time ((time_t *) NULL); // Get the system time and put it into 'now' as 'calender time'
  time (&now);
  l_time = localtime (&now);
  strftime (string, sizeof string, "%d-%b-%Y", l_time);
  fprintf(p_log,"| %1.5s |",string );

  strftime (string, sizeof string, "%H:%M:%S", l_time);
  fprintf(p_log," %1.5s |",string );
#endif

  for (i=0;i<20;i++)
    name[i]=input->infile[i+max(0,((int)strlen(input->infile))-20)]; // write last part of path, max 20 chars
  fprintf(p_log,"%20.20s|",name);

  fprintf(p_log,"%5d |",input->no_frames);
  fprintf(p_log,"  %2d  |",input->qp0);
  fprintf(p_log,"  %2d  |",input->qpN);
  fprintf(p_log," %2d |",input->qpB);

  fprintf(p_log,"%4dx%-4d|",input->img_width,input->img_height);

  fprintf(p_log,"%3d |",input->successive_Bframe); 


  if (input->hadamard==1)
    fprintf(p_log,"   ON   |");
  else
    fprintf(p_log,"   OFF  |");

  fprintf(p_log,"   %3d   |",input->search_range );

  fprintf(p_log," %2d  |",input->num_reference_frames);

//  fprintf(p_log," %3d  |",img->framerate/(input->jumpd+1));
    fprintf(p_log," %3d  |",(img->framerate *(input->successive_Bframe + 1)) / (input->jumpd+1));


  if (input->intra_upd==1)
    fprintf(p_log,"   ON    |");
  else
    fprintf(p_log,"   OFF   |");

  fprintf(p_log,"%-5.3f|",snr->snr_y1);
  fprintf(p_log,"%-5.3f|",snr->snr_u1);
  fprintf(p_log,"%-5.3f|",snr->snr_v1);
  fprintf(p_log,"%-5.3f|",snr->snr_ya);
  fprintf(p_log,"%-5.3f|",snr->snr_ua);
  fprintf(p_log,"%-5.3f|",snr->snr_va);
  if(input->successive_Bframe != 0)
  {
    fprintf(p_log,"%7.0f|",stat->bitrate_P);
    fprintf(p_log,"%7.0f|",stat->bitrate_B);
  }
  else
  {
    fprintf(p_log,"%7.0f|",stat->bitrate);
    fprintf(p_log,"%7.0f|",0.0);
  }


  fprintf(p_log,"   %12d   |", tot_time);
  fprintf(p_log,"   %12d   |\n", me_tot_time);

/*
  if(input->successive_Bframe != 0)
  {
    fprintf(p_log,"%7.0f|",stat->bitrate_P);
    fprintf(p_log,"%7.0f|\n",stat->bitrate_B);
  }
  else
  {
    fprintf(p_log,"%7.0f|",stat->bitrate);
    fprintf(p_log,"%7.0f|\n",0.0);
  }
*/

  fclose(p_log);

  p_log=fopen("data.txt","a");

  if(input->successive_Bframe != 0 && Bframe_ctr != 0) // B picture used
  {
    fprintf(p_log, "%3d %2d %2d %2.2f %2.2f %2.2f %5d "
          "%2.2f %2.2f %2.2f %5d "
        "%2.2f %2.2f %2.2f %5d %5d %.3f\n",
        input->no_frames, input->qp0, input->qpN,
        snr->snr_y1,
        snr->snr_u1,
        snr->snr_v1,
        stat->bit_ctr_0,
        0.0,
        0.0,
        0.0,
        0,
        snr->snr_ya,
        snr->snr_ua,
        snr->snr_va,
        (stat->bit_ctr_0+stat->bit_ctr)/(input->no_frames+Bframe_ctr),
        stat->bit_ctr_B/Bframe_ctr,
        (double)0.001*tot_time/(input->no_frames+Bframe_ctr));
  }
  else
  {
    fprintf(p_log, "%3d %2d %2d %2.2f %2.2f %2.2f %5d "
          "%2.2f %2.2f %2.2f %5d "
        "%2.2f %2.2f %2.2f %5d %5d %.3f\n",
        input->no_frames, input->qp0, input->qpN,
        snr->snr_y1,
        snr->snr_u1,
        snr->snr_v1,
        stat->bit_ctr_0,
        0.0,
        0.0,
        0.0,
        0,
        snr->snr_ya,
        snr->snr_ua,
        snr->snr_va,
        (stat->bit_ctr_0+stat->bit_ctr)/input->no_frames,
        0,
        (double)0.001*tot_time/input->no_frames);
  }

  fclose(p_log);

 
}


/*!
 ************************************************************************
 * \brief
 *    Prints the header of the protocol.
 * \par Input:
 *    struct inp_par *inp
 * \par Output:
 *    none
 ************************************************************************
 */
void information_init()
{

  printf("-------------------------------------------------------------------------------\n");
  printf(" Input YUV file                    : %s \n",input->infile);
  printf(" Output H.264 bitstream            : %s \n",input->outfile);
  if (p_dec != NULL)
    printf(" Output YUV file                   : %s \n",input->ReconFile);
  printf(" Output log file                   : log.dat \n");
  printf(" Output statistics file            : stat.dat \n");
  printf("-------------------------------------------------------------------------------\n");
  printf("  Frame  Bit/pic WP QP   SnrY    SnrU    SnrV    Time(ms) MET(ms) Frm/Fld   I D\n");
//  printf(" Frame   Bit/pic   QP   SnrY    SnrU    SnrV    Time(ms) Frm/Fld IntraMBs\n");
  printf("-------------------------------------------------------------------------------\n");
}


/*!
 ************************************************************************
 * \brief
 *    Dynamic memory allocation of frame size related global buffers
 *    buffers are defined in global.h, allocated memory must be freed in
 *    void free_global_buffers()
 * \par Input:
 *    Input Parameters struct inp_par *inp,                            \n
 *    Image Parameters struct img_par *img
 * \return Number of allocated bytes
 ************************************************************************
 */
int init_global_buffers()
{
  int j,memory_size=0;
  int height_field = img->height/2;
#ifdef _ADAPT_LAST_GROUP_
  extern int *last_P_no_frm;
  extern int *last_P_no_fld;

  if ((last_P_no_frm = (int*)malloc(2*img->max_num_references*sizeof(int))) == NULL)
    no_mem_exit("init_global_buffers: last_P_no");
  if(!active_sps->frame_mbs_only_flag)
    if ((last_P_no_fld = (int*)malloc(4*img->max_num_references*sizeof(int))) == NULL)
      no_mem_exit("init_global_buffers: last_P_no");
#endif

  // allocate memory for encoding frame buffers: imgY, imgUV

  // allocate memory for reference frame buffers: imgY_org, imgUV_org
  // byte imgY_org[288][352];
  // byte imgUV_org[2][144][176];
  memory_size += get_mem2D(&imgY_org_frm, img->height, img->width);
  memory_size += get_mem3D(&imgUV_org_frm, 2, img->height_cr, img->width_cr);


    if (input->WeightedPrediction || input->WeightedBiprediction)
    {
      // Currently only use up to 20 references. Need to use different indicator such as maximum num of references in list
      memory_size += get_mem3Dint(&wp_weight,6,MAX_REFERENCE_PICTURES,3);
      memory_size += get_mem3Dint(&wp_offset,6,MAX_REFERENCE_PICTURES,3);

      memory_size += get_mem4Dint(&wbp_weight, 6, MAX_REFERENCE_PICTURES, MAX_REFERENCE_PICTURES, 3);
    }

  // allocate memory for reference frames of each block: refFrArr
  // int  refFrArr[72][88];

  if(input->successive_Bframe!=0 || input->StoredBPictures > 0)
  {    
    memory_size += get_mem3Dint(&direct_ref_idx, 2, img->width/BLOCK_SIZE, img->height/BLOCK_SIZE);
    memory_size += get_mem2Dint(&direct_pdir, img->width/BLOCK_SIZE, img->height/BLOCK_SIZE);
  }

  // allocate memory for temp quarter pel luma frame buffer: img4Y_tmp
  // int img4Y_tmp[576][704];  (previously int imgY_tmp in global.h)
  memory_size += get_mem2Dint(&img4Y_tmp, img->height+2*IMG_PAD_SIZE, (img->width+2*IMG_PAD_SIZE)*4);

  if (input->rdopt==2)
  {
    memory_size += get_mem2Dint(&decs->resY, MB_BLOCK_SIZE, MB_BLOCK_SIZE);
    if ((decs->decref = (byte****) calloc(input->NoOfDecoders,sizeof(byte***))) == NULL) 
      no_mem_exit("init_global_buffers: decref");
    for (j=0 ; j<input->NoOfDecoders; j++)
    {
      memory_size += get_mem3D(&decs->decref[j], img->max_num_references+1, img->height, img->width);
    }
    memory_size += get_mem2D(&decs->RefBlock, BLOCK_SIZE,BLOCK_SIZE);
    memory_size += get_mem3D(&decs->decY, input->NoOfDecoders, img->height, img->width);
    memory_size += get_mem3D(&decs->decY_best, input->NoOfDecoders, img->height, img->width);
    memory_size += get_mem2D(&decs->status_map, img->height/MB_BLOCK_SIZE,img->width/MB_BLOCK_SIZE);
    memory_size += get_mem2D(&decs->dec_mb_mode, img->width/MB_BLOCK_SIZE,img->height/MB_BLOCK_SIZE);
  }
  if (input->RestrictRef)
  {
    memory_size += get_mem2D(&pixel_map, img->height,img->width);
    memory_size += get_mem2D(&refresh_map, img->height/8,img->width/8);
  }

  if(!active_sps->frame_mbs_only_flag)
  {
    // allocate memory for encoding frame buffers: imgY, imgUV
    memory_size += get_mem2D(&imgY_com, img->height, img->width);
    memory_size += get_mem3D(&imgUV_com, 2, img->height/2, img->width_cr);

    // allocate memory for reference frame buffers: imgY_org, imgUV_org
    memory_size += get_mem2D(&imgY_org_top, height_field, img->width);
    memory_size += get_mem3D(&imgUV_org_top, 2, height_field/2, img->width_cr);
    memory_size += get_mem2D(&imgY_org_bot, height_field, img->width);
    memory_size += get_mem3D(&imgUV_org_bot, 2, height_field/2, img->width_cr);

  }

  if(input->FMEnable)
    memory_size += get_mem_FME();

  return (memory_size);
}

/*!
 ************************************************************************
 * \brief
 *    Free allocated memory of frame size related global buffers
 *    buffers are defined in global.h, allocated memory is allocated in
 *    int get_mem4global_buffers()
 * \par Input:
 *    Input Parameters struct inp_par *inp,                             \n
 *    Image Parameters struct img_par *img
 * \par Output:
 *    none
 ************************************************************************
 */
void free_global_buffers()
{
  int  i,j;

#ifdef _ADAPT_LAST_GROUP_
  extern int *last_P_no_frm;
  extern int *last_P_no_fld;
  free (last_P_no_frm);
  free (last_P_no_fld);
#endif

  free_mem2D(imgY_org_frm);      // free ref frame buffers
  free_mem3D(imgUV_org_frm,2);

  // free multiple ref frame buffers
  // number of reference frames increased by one for next P-frame

  if (input->WeightedPrediction || input->WeightedBiprediction)
  {
    free_mem3Dint(wp_weight,6);
    free_mem3Dint(wp_offset,6);
    free_mem4Dint(wbp_weight,6,MAX_REFERENCE_PICTURES);
  }

  if(input->successive_Bframe!=0 || input->StoredBPictures > 0)
  {
    free_mem3Dint(direct_ref_idx,2);
    free_mem2Dint(direct_pdir);
  } // end if B frame


  free_mem2Dint(img4Y_tmp);    // free temp quarter pel frame buffer

  // free mem, allocated in init_img()
  // free intra pred mode buffer for blocks
  free_mem2Dint(img->ipredmode);
  free(img->mb_data);

  if(input->UseConstrainedIntraPred)
  {
    free (img->intra_block);
  }

  if (input->rdopt==2)
  {
    free(decs->resY[0]);
    free(decs->resY);
    free(decs->RefBlock[0]);
    free(decs->RefBlock);
    for (j=0; j<input->NoOfDecoders; j++)
    {
      free(decs->decY[j][0]);
      free(decs->decY[j]);
      free(decs->decY_best[j][0]);
      free(decs->decY_best[j]);
      for (i=0; i<img->max_num_references+1; i++)
      {
        free(decs->decref[j][i][0]);
        free(decs->decref[j][i]);
      }
      free(decs->decref[j]);
    }
    free(decs->decY);
    free(decs->decY_best);
    free(decs->decref);
    free(decs->status_map[0]);
    free(decs->status_map);
    free(decs->dec_mb_mode[0]);
    free(decs->dec_mb_mode);
  }
  if (input->RestrictRef)
  {
    free(pixel_map[0]);
    free(pixel_map);
    free(refresh_map[0]);
    free(refresh_map);
  }

  if(!active_sps->frame_mbs_only_flag)
  {
    free_mem2D(imgY_com);
    free_mem3D(imgUV_com,2);
    free_mem2D(imgY_org_top);      // free ref frame buffers
    free_mem3D(imgUV_org_top,2);
    free_mem2D(imgY_org_bot);      // free ref frame buffers
    free_mem3D(imgUV_org_bot,2);

  }

  free_mem3Dint(img->nz_coeff, img->FrameSizeInMbs);

  if(input->FMEnable)
    free_mem_FME();
}

/*!
 ************************************************************************
 * \brief
 *    Allocate memory for mv
 * \par Input:
 *    Image Parameters struct img_par *img                             \n
 *    int****** mv
 * \return memory size in bytes
 ************************************************************************
 */
int get_mem_mv (int******* mv)
{
  int i, j, k, l, m;

  if ((*mv = (int******)calloc(4,sizeof(int*****))) == NULL)
    no_mem_exit ("get_mem_mv: mv");
  for (i=0; i<4; i++)
  {
    if (((*mv)[i] = (int*****)calloc(4,sizeof(int****))) == NULL)
      no_mem_exit ("get_mem_mv: mv");
    for (j=0; j<4; j++)
    {
      if (((*mv)[i][j] = (int****)calloc(2,sizeof(int***))) == NULL)
        no_mem_exit ("get_mem_mv: mv");
      for (k=0; k<2; k++)
      {
        if (((*mv)[i][j][k] = (int***)calloc(img->max_num_references,sizeof(int**))) == NULL)
          no_mem_exit ("get_mem_mv: mv");
        for (l=0; l<img->max_num_references; l++)
        {
          if (((*mv)[i][j][k][l] = (int**)calloc(9,sizeof(int*))) == NULL)
            no_mem_exit ("get_mem_mv: mv");
          for (m=0; m<9; m++)
            if (((*mv)[i][j][k][l][m] = (int*)calloc(2,sizeof(int))) == NULL)
              no_mem_exit ("get_mem_mv: mv");
        }
      }
    }
  }
  return 4*4*img->max_num_references*9*2*sizeof(int);
}


/*!
 ************************************************************************
 * \brief
 *    Free memory from mv
 * \par Input:
 *    int****** mv
 ************************************************************************
 */
void free_mem_mv (int****** mv)
{
  int i, j, k, l, m;

  for (i=0; i<4; i++)
  {
    for (j=0; j<4; j++)
    {
      for (k=0; k<2; k++)
      {
        for (l=0; l<img->max_num_references; l++)
        {
          for (m=0; m<9; m++)
          {
            free (mv[i][j][k][l][m]);
          }
          free (mv[i][j][k][l]);
        }
        free (mv[i][j][k]);
      }
      free (mv[i][j]);
    }
    free (mv[i]);
  }
  free (mv);
}





/*!
 ************************************************************************
 * \brief
 *    Allocate memory for AC coefficients
 ************************************************************************
 */
int get_mem_ACcoeff (int***** cofAC)
{
  int i, j, k;

  if ((*cofAC = (int****)calloc (6, sizeof(int***))) == NULL)              no_mem_exit ("get_mem_ACcoeff: cofAC");
  for (k=0; k<6; k++)
  {
    if (((*cofAC)[k] = (int***)calloc (4, sizeof(int**))) == NULL)         no_mem_exit ("get_mem_ACcoeff: cofAC");
    for (j=0; j<4; j++)
    {
      if (((*cofAC)[k][j] = (int**)calloc (2, sizeof(int*))) == NULL)      no_mem_exit ("get_mem_ACcoeff: cofAC");
      for (i=0; i<2; i++)
      {
        if (((*cofAC)[k][j][i] = (int*)calloc (65, sizeof(int))) == NULL)  no_mem_exit ("get_mem_ACcoeff: cofAC"); // 18->65 for ABT
      }
    }
  }
  return 6*4*2*65*sizeof(int);// 18->65 for ABT
}

/*!
 ************************************************************************
 * \brief
 *    Allocate memory for DC coefficients
 ************************************************************************
 */
int get_mem_DCcoeff (int**** cofDC)
{
  int j, k;

  if ((*cofDC = (int***)calloc (3, sizeof(int**))) == NULL)           no_mem_exit ("get_mem_DCcoeff: cofDC");
  for (k=0; k<3; k++)
  {
    if (((*cofDC)[k] = (int**)calloc (2, sizeof(int*))) == NULL)      no_mem_exit ("get_mem_DCcoeff: cofDC");
    for (j=0; j<2; j++)
    {
      if (((*cofDC)[k][j] = (int*)calloc (65, sizeof(int))) == NULL)  no_mem_exit ("get_mem_DCcoeff: cofDC"); // 18->65 for ABT
    }
  }
  return 3*2*65*sizeof(int); // 18->65 for ABT
}


/*!
 ************************************************************************
 * \brief
 *    Free memory of AC coefficients
 ************************************************************************
 */
void free_mem_ACcoeff (int**** cofAC)
{
  int i, j, k;

  for (k=0; k<6; k++)
  {
    for (i=0; i<4; i++)
    {
      for (j=0; j<2; j++)
      {
        free (cofAC[k][i][j]);
      }
      free (cofAC[k][i]);
    }
    free (cofAC[k]);
  }
  free (cofAC);
}

/*!
 ************************************************************************
 * \brief
 *    Free memory of DC coefficients
 ************************************************************************
 */
void free_mem_DCcoeff (int*** cofDC)
{
  int i, j;

  for (j=0; j<3; j++)
  {
    for (i=0; i<2; i++)
    {
      free (cofDC[j][i]);
    }
    free (cofDC[j]);
  }
  free (cofDC);
}


/*!
 ************************************************************************
 * \brief
 *    form frame picture from two field pictures 
 ************************************************************************
 */
void combine_field()
{
  int i;

  for (i=0; i<img->height / 2; i++)
  {
    memcpy(imgY_com[i*2], enc_top_picture->imgY[i], img->width);     // top field
    memcpy(imgY_com[i*2 + 1], enc_bottom_picture->imgY[i], img->width); // bottom field
  }

  for (i=0; i<img->height_cr / 2; i++)
  {
    memcpy(imgUV_com[0][i*2], enc_top_picture->imgUV[0][i], img->width_cr);
    memcpy(imgUV_com[0][i*2 + 1], enc_bottom_picture->imgUV[0][i], img->width_cr);
    memcpy(imgUV_com[1][i*2], enc_top_picture->imgUV[1][i], img->width_cr);
    memcpy(imgUV_com[1][i*2 + 1], enc_bottom_picture->imgUV[1][i], img->width_cr);
  }
}

/*!
 ************************************************************************
 * \brief
 *    RD decision of frame and field coding 
 ************************************************************************
 */
int decide_fld_frame(float snr_frame_Y, float snr_field_Y, int bit_field, int bit_frame, double lambda_picture)
{
  double cost_frame, cost_field;

  cost_frame = bit_frame * lambda_picture + snr_frame_Y;
  cost_field = bit_field * lambda_picture + snr_field_Y;

  if (cost_field > cost_frame)
    return (0);
  else
    return (1);
}

/*!
 ************************************************************************
 * \brief
 *    Do some initializaiton work for encoding the 2nd IGOP
 ************************************************************************
 */
void process_2nd_IGOP()
{
  Boolean FirstIGOPFinished = FALSE;
  if ( img->number == input->no_frames-1 )
    FirstIGOPFinished = TRUE;
  if (input->NumFrameIn2ndIGOP==0) return;
  if (!FirstIGOPFinished || In2ndIGOP) return;
  In2ndIGOP = TRUE;

//  img->number = -1;
  start_frame_no_in_this_IGOP = input->no_frames;
  start_tr_in_this_IGOP = (input->no_frames-1)*(input->jumpd+1) +1;
  input->no_frames = input->no_frames + input->NumFrameIn2ndIGOP;

/*  reset_buffers();

  frm->picbuf_short[0]->used=0;
  frm->picbuf_short[0]->picID=-1;
  frm->picbuf_short[0]->lt_picID=-1;
  frm->short_used = 0; */
  img->nb_references = 0;
}


/*!
 ************************************************************************
 * \brief
 *    Set the image type for I,P and SP pictures (not B!)
 ************************************************************************
 */
void SetImgType()
{
  if (input->intra_period == 0)
  {
    if (IMG_NUMBER == 0)
    {
      img->type = I_SLICE;        // set image type for first image to I-frame
    }
    else
    {
      img->type = P_SLICE;        // P-frame

      if (input->sp_periodicity)
      {
        if ((IMG_NUMBER % input->sp_periodicity) ==0)
        {
          img->type=SP_SLICE;
        }
      }
    }
  }
  else
  {
    if ((IMG_NUMBER%input->intra_period) == 0)
    {
      img->type = I_SLICE;
    }
    else
    {
      img->type = P_SLICE;        // P-frame
      if (input->sp_periodicity)
      {
        if ((IMG_NUMBER % input->sp_periodicity) ==0)
            img->type=SP_SLICE;
      }
    }
  }
}

 
