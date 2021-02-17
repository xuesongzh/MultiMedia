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
 * \file  nal_part.c
 *
 * \brief
 *    Network Adaptation layer for partition file
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Tobias Oelbaum <oelbaum@hhi.de, oelbaum@drehvial.de>
 ************************************************************************
 */

#include <string.h>

#include "contributors.h"
#include "global.h"
#include "elements.h"

int assignSE2partition[][SE_MAX_ELEMENTS] =
{
  // 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19  // elementnumber (no not uncomment)
  {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   //!< all elements in one partition no data partitioning
  {  0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 2, 2, 2, 2, 0, 0, 0, 0 }    //!< three partitions per slice
};

int PartitionMode;

/*!
 ************************************************************************
 * \brief
 *    Resets the entries in the bitstream struct
 ************************************************************************
 */
void free_Partition(Bitstream *currStream)
{
  byte *buf = currStream->streamBuffer;

  currStream->bitstream_length = 0;
  currStream->frame_bitoffset = 0;
  currStream->ei_flag =0;
  memset (buf, 0x00, MAX_CODED_FRAME_SIZE);
}
