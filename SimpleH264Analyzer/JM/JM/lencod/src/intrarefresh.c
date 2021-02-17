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
 *****************************************************************************
 *
 * \file intrarefresh.c
 *
 * \brief
 *    Encoder support for pseudo-random intra macroblock refresh
 *
 * \date
 *    16 June 2002
 *
 * \author
 *    Stephan Wenger   stewe@cs.tu-berlin.de
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <memory.h>
#include <malloc.h>

#include "intrarefresh.h"

static int *RefreshPattern;
static int *IntraMBs;
static int WalkAround = 0;
static int NumberOfMBs = 0;
static int NumberIntraPerPicture;
 
/*!
 ************************************************************************
 * \brief
 *    RandomIntraInit: Initializes Random Intra module.  Should be called
 *    only after initialization (or changes) of the picture size or the
 *    random intra refresh value.  In version jm2.1 it is impossible to
 *    change those values on-the-fly, hence RandomIntraInit should be
 *    called immediately after the parsing of the config file
 *
 * \par Input:
 *    xsize, ysize: size of the picture (in MBs)
 *    refresh     : refresh rate in MBs per picture
 ************************************************************************
 */

void RandomIntraInit(int xsize, int ysize, int refresh)
{
  int i, pos;

  srand (1);      // A fixed random initializer to make things reproducable
  NumberOfMBs = xsize * ysize;
  NumberIntraPerPicture = refresh;

  RefreshPattern = malloc (sizeof (int) * NumberOfMBs);
  assert (RefreshPattern != NULL);

  IntraMBs = malloc (sizeof (int) * refresh);
  assert (IntraMBs != NULL);

  for (i= 0; i<NumberOfMBs; i++)
    RefreshPattern[i] = -1;

   for (i=0; i<NumberOfMBs; i++)
   {
     do
     {
       pos = rand() % NumberOfMBs;
     } while (RefreshPattern [pos] != -1);
     RefreshPattern [pos] = i;
   }
/*
for (i=0; i<NumberOfMBs; i++) printf ("%d\t", RefreshPattern[i]);
getchar();
*/
}

/*!
 ************************************************************************
 * \brief
 *    RandomIntra: Code an MB as Intra?
 *
 * \par Input
 *    MacroblockNumberInScanOrder
 * \par Output
 *    1 if an MB should be forced to Intra, according the the 
 *      RefreshPattern
 *    0 otherwise
 *
 ************************************************************************
 */

int RandomIntra (int mb)
{
  int i;

  for (i=0; i<NumberIntraPerPicture; i++)
    if (IntraMBs[i] == mb)
      return 1;
  return 0;
}


/*!
 ************************************************************************
 * \brief
 *    RandomIntraNewPicture: Selects new set of MBs for forced Intra
 *
 * \par
 *    This function should be called exactly once per picture, and 
 *    requires a finished initialization 
 *
 ************************************************************************
 */

void RandomIntraNewPicture ()
{
  int i, j;

  WalkAround += NumberIntraPerPicture;
  for (j=0,i=WalkAround; j<NumberIntraPerPicture; j++, i++)
    IntraMBs[j] = RefreshPattern [i%NumberOfMBs];
}

void RandomIntraUninit()
{
  free(RefreshPattern);
  free(IntraMBs);
}
