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
 * \file vlc.h
 *
 * \brief
 *    header for (CA)VLC coding functions
 *
 * \author
 *    Karsten Suehring
 *
 ************************************************************************
 */

#ifndef _VLC_H_
#define _VLC_H_

int se_v (char *tracestring, Bitstream *bitstream);
int ue_v (char *tracestring, Bitstream *bitstream);
int u_1 (char *tracestring, Bitstream *bitstream);
int u_v (int LenInBits, char *tracestring, Bitstream *bitstream);

// UVLC mapping
void linfo_ue(int len, int info, int *value1, int *dummy);
void linfo_se(int len, int info, int *value1, int *dummy);

void linfo_cbp_intra(int len,int info,int *cbp, int *dummy);
void linfo_cbp_inter(int len,int info,int *cbp, int *dummy);
void linfo_levrun_inter(int len,int info,int *level,int *irun);
void linfo_levrun_c2x2(int len,int info,int *level,int *irun);

int  readSyntaxElement_VLC (SyntaxElement *sym, Bitstream *currStream);
int  readSyntaxElement_UVLC(SyntaxElement *sym, struct img_par *img, struct inp_par *inp, struct datapartition *dp);
int  readSyntaxElement_Intra4x4PredictionMode(SyntaxElement *sym, struct img_par *img, struct inp_par *inp, struct datapartition *dp);

int  GetVLCSymbol (byte buffer[],int totbitoffset,int *info, int bytecount);
int  GetVLCSymbol_IntraMode (byte buffer[],int totbitoffset,int *info, int bytecount);

int readSyntaxElement_FLC(SyntaxElement *sym, Bitstream *currStream);
int readSyntaxElement_NumCoeffTrailingOnes(SyntaxElement *sym,  DataPartition *dP,
                                           char *type);
int readSyntaxElement_NumCoeffTrailingOnesChromaDC(SyntaxElement *sym,  DataPartition *dP);
int readSyntaxElement_Level_VLC0(SyntaxElement *sym, struct datapartition *dP);
int readSyntaxElement_Level_VLCN(SyntaxElement *sym, int vlc, struct datapartition *dP);
int readSyntaxElement_TotalZeros(SyntaxElement *sym,  DataPartition *dP);
int readSyntaxElement_TotalZerosChromaDC(SyntaxElement *sym,  DataPartition *dP);
int readSyntaxElement_Run(SyntaxElement *sym,  DataPartition *dP);
int GetBits (byte buffer[],int totbitoffset,int *info, int bytecount, 
             int numbits);
int ShowBits (byte buffer[],int totbitoffset,int bytecount, int numbits);


#endif

