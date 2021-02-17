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
 *    parset.h
 * \brief
 *    Picture and Sequence Parameter Sets, decoder operations
 *    This code reflects JVT version xxx
 * \date 25 November 2002
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details) 
 *      - Stephan Wenger        <stewe@cs.tu-berlin.de>
 ***************************************************************************************
 */
#ifndef _PARSET_H_
#define _PARSET_H_


#include "parsetcommon.h"
#include "nalucommon.h"

void PPSConsistencyCheck (pic_parameter_set_rbsp_t *pps);
void SPSConsistencyCheck (seq_parameter_set_rbsp_t *sps);

void MakePPSavailable (int id, pic_parameter_set_rbsp_t *pps);
void MakeSPSavailable (int id, seq_parameter_set_rbsp_t *sps);

void ProcessSPS (NALU_t *nalu);
void ProcessPPS (NALU_t *nalu);

void UseParameterSet (int PicParsetId);

void activate_sps (seq_parameter_set_rbsp_t *sps);
void activate_pps (pic_parameter_set_rbsp_t *pps);

#endif
