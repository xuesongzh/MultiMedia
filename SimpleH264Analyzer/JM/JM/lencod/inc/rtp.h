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
 ***************************************************************************
 *
 * \file rtp.h
 *
 * \brief
 *    Definition of structures and functions to handle RTP headers.  For a
 *    description of RTP see RFC1889 on http://www.ietf.org
 *
 * \date
 *    30 September 2001
 *
 * \author
 *    Stephan Wenger   stewe@cs.tu-berlin.de
 **************************************************************************/

#ifndef _RTP_H_
#define _RTP_H_

#include <stdio.h>
#include "global.h"
#include "nalu.h"

#define MAXRTPPAYLOADLEN  (65536 - 40)    //!< Maximum payload size of an RTP packet
#define MAXRTPPACKETSIZE  (65536 - 28)    //!< Maximum size of an RTP packet incl. header
#define H26LPAYLOADTYPE 105               //!< RTP paylaod type fixed here for simplicity
#define H26LSSRC 0x12345678               //!< SSRC, chosen to simplify debugging
#define RTP_TR_TIMESTAMP_MULT 1000        //!< should be something like 27 Mhz / 29.97 Hz

typedef struct 
{
  unsigned int v;          //!< Version, 2 bits, MUST be 0x2
  unsigned int p;          //!< Padding bit, Padding MUST NOT be used
  unsigned int x;          //!< Extension, MUST be zero */
  unsigned int cc;         /*!< CSRC count, normally 0 in the absence 
                                of RTP mixers */
  unsigned int m;          //!< Marker bit
  unsigned int pt;         //!< 7 bits, Payload Type, dynamically established
  unsigned int seq;        /*!< RTP sequence number, incremented by one for 
                                each sent packet */
  unsigned int timestamp;  //!< timestamp, 27 MHz for H.264
  unsigned int ssrc;       //!< Synchronization Source, chosen randomly
  byte *       payload;    //!< the payload including payload headers
  unsigned int paylen;     //!< length of payload in bytes
  byte *       packet;     //!< complete packet including header and payload
  unsigned int packlen;    //!< length of packet, typically paylen+12
} RTPpacket_t;

#if 0
int  ComposeRTPPacket (RTPpacket_t *p);
int  DecomposeRTPpacket (RTPpacket_t *p);
int  WriteRTPPacket (RTPpacket_t *p, FILE *f);
void DumpRTPHeader (RTPpacket_t *p);
void RTPUpdateTimestamp (int tr);
int  RTPWriteBits (int Marker, int PacketType, void * bitstream, 
                   int BitStreamLenInByte, FILE *out);

Boolean isAggregationPacket();
int aggregationRTPWriteBits (int Marker, int PacketType, int subPacketType, void * bitstream, int BitStreamLenInByte, FILE *out);

void begin_sub_sequence_rtp();
void end_sub_sequence_rtp();
#endif

void RTPUpdateTimestamp (int tr);
void OpenRTPFile (char *Filename);
void CloseRTPFile ();
int WriteRTPNALU (NALU_t *n);



#endif

