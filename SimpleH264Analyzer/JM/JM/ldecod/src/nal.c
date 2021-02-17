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
 * \file  nal.c
 *
 * \brief
 *    Converts Encapsulated Byte Sequence Packets (EBSP) to Raw Byte 
 *    Sequence Packets (RBSP), and then onto String Of Data Bits (SODB)
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Shankar L. Regunathan <shanre@microsoft.com>
 ************************************************************************
 */

#include "contributors.h"

#include <assert.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>
#include <string.h>

#include "defines.h"
#include "global.h"


 /*!
 ************************************************************************
 * \brief
 *    Converts RBSP to string of data bits
 * \param streamBuffer
 *          pointer to buffer containing data
 *  \param last_byte_pos
 *          position of the last byte containing data.
 * \return last_byte_pos
 *          position of the last byte pos. If the last-byte was entirely a stuffing byte,
 *          it is removed, and the last_byte_pos is updated.
 *  
************************************************************************/

int RBSPtoSODB(byte *streamBuffer, int last_byte_pos)
{
  int ctr_bit, bitoffset;
  
  bitoffset = 0; 
  //find trailing 1
  ctr_bit = (streamBuffer[last_byte_pos-1] & (0x01<<bitoffset));   // set up control bit
  
  while (ctr_bit==0)
  {                 // find trailing 1 bit
    bitoffset++;
    if(bitoffset == 8) 
    {
      if(last_byte_pos == 0)
        printf(" Panic: All zero data sequence in RBSP \n");
      assert(last_byte_pos != 0);
      last_byte_pos -= 1;
      bitoffset = 0;
    }
    ctr_bit= streamBuffer[last_byte_pos-1] & (0x01<<(bitoffset));
  }
  
  
  // We keep the stop bit for now
/*  if (remove_stop)
  {
    streamBuffer[last_byte_pos-1] -= (0x01<<(bitoffset));
    if(bitoffset == 7)
      return(last_byte_pos-1);
    else
      return(last_byte_pos);
  }
*/
  return(last_byte_pos);
  
}


/*!
************************************************************************
* \brief
*    Converts Encapsulated Byte Sequence Packets to RBSP
* \param streamBuffer
*    pointer to data stream
* \param end_bytepos
*    size of data stream
* \param begin_bytepos
*    Position after beginning 
************************************************************************/


int EBSPtoRBSP(byte *streamBuffer, int end_bytepos, int begin_bytepos)
{
  int i, j, count;
  count = 0;
  
  if(end_bytepos < begin_bytepos)
    return end_bytepos;
  
  j = begin_bytepos;
  
  for(i = begin_bytepos; i < end_bytepos; i++) 
  { //starting from begin_bytepos to avoid header information
    if(count == ZEROBYTES_SHORTSTARTCODE && streamBuffer[i] == 0x03) 
    {
      i++;
      count = 0;
    }
    streamBuffer[j] = streamBuffer[i];
    if(streamBuffer[i] == 0x00)
      count++;
    else
      count = 0;
    j++;
  }
  
  return j;
}
