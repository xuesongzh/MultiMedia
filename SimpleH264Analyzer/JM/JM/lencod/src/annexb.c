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
 * \file annexb.c
 *
 * \brief
 *    Annex B Byte Stream format NAL Unit writing routines
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *************************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "global.h"
#include "nalucommon.h"

static FILE *f = NULL;    // the output file


/*!
 ********************************************************************************************
 * \brief 
 *    Writes a NALU to the Annex B Byte Stream
 *
 * \return
 *    number of bits written
 *
 ********************************************************************************************
*/
int WriteAnnexbNALU (NALU_t *n)
{
  int BitsWritten = 0;

  assert (n != NULL);
  assert (n->forbidden_bit == 0);
  assert (f != NULL);
  assert (n->startcodeprefix_len == 3 || n->startcodeprefix_len == 4);

// printf ("WriteAnnexbNALU: writing %d bytes w/ startcode_len %d\n", n->len+1, n->startcodeprefix_len); 
  if (n->startcodeprefix_len > 3)
  {
    putc (0, f);
    BitsWritten =+ 8;
  }
  putc (0, f);
  putc (0, f);
  putc (1, f);
  BitsWritten += 24;

  n->buf[0] =
    n->forbidden_bit << 7      |
    n->nal_reference_idc << 5  |
    n->nal_unit_type;

// printf ("First Byte %x, nal_ref_idc %x, nal_unit_type %d\n", n->buf[0], n->nal_reference_idc, n->nal_unit_type);

  if (n->len != fwrite (n->buf, 1, n->len, f))
  {
    printf ("Fatal: cannot write %d bytes to bitstream file, exit (-1)\n", n->len);
    exit (-1);
  }
  BitsWritten += n->len * 8;

  fflush (f);
#if TRACE
  fprintf (p_trace, "\n\nAnnex B NALU w/ %s startcode, len %d, forbidden_bit %d, nal_reference_idc %d, nal_unit_type %d\n\n",
    n->startcodeprefix_len == 4?"long":"short", n->len, n->forbidden_bit, n->nal_reference_idc, n->nal_unit_type);
  fflush (p_trace);
#endif
  return BitsWritten;
}


/*!
 ********************************************************************************************
 * \brief 
 *    Opens the output file for the bytestream    
 *
 * \param Filename
 *    The filename of the file to be opened
 *
 * \return
 *    none.  Function terminates the program in case of an error
 *
 ********************************************************************************************
*/
void OpenAnnexbFile (char *Filename)
{
  if ((f = fopen (Filename, "wb")) == NULL)
  {
    printf ("Fatal: cannot open Annex B bytestream file '%s', exit (-1)\n", Filename);
    exit (-1);
  }
}


/*!
 ********************************************************************************************
 * \brief 
 *    Closes the output bit stream file
 *
 * \return
 *    none.  Funtion trerminates the program in case of an error
 ********************************************************************************************
*/
void CloseAnnexbFile() {
  if (fclose (f))
  {
    printf ("Fatal: cannot close Annex B bytestream file, exit (-1)\n");
    exit (-1);
  }
}

