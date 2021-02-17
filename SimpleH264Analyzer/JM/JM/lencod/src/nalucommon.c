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
 * \file  nalucommon.c
 *
 * \brief
 *    Common NALU support functions
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *    - Stephan Wenger   <stewe@cs.tu-berlin.de>
 ************************************************************************
 */

#include <stdio.h>
#include <assert.h>
#include <malloc.h>

#include "global.h"
#include "memory.h"
#include "nalu.h"
#include "memalloc.h"


/*! 
 *************************************************************************************
 * \brief
 *    Allocates memory for a NALU
 *
 * \param buffersize
 *     size of NALU buffer 
 *
 * \return
 *    pointer to a NALU
 *************************************************************************************
 */
 

NALU_t *AllocNALU(int buffersize)
{
  NALU_t *n;

  if ((n = (NALU_t*)calloc (1, sizeof (NALU_t))) == NULL) no_mem_exit ("AllocNALU: n");

  n->max_size=buffersize;

  if ((n->buf = (byte*)calloc (buffersize, sizeof (byte))) == NULL) no_mem_exit ("AllocNALU: n->buf");
  
  return n;
}


/*! 
 *************************************************************************************
 * \brief
 *    Frees a NALU
 *
 * \param n 
 *    NALU to be freed
 *
 *************************************************************************************
 */

void FreeNALU(NALU_t *n)
{
  if (n)
  {
    if (n->buf)
    {
      free(n->buf);
      n->buf=NULL;
    }
    free (n);
  }
}

