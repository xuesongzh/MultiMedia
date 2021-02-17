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
 * \file 
 *      erc_do_p.c
 *
 * \brief
 *      Inter (P) frame error concealment algorithms for decoder
 *
 *  \author
 *      - Viktor Varsa                     <viktor.varsa@nokia.com>
 *      - Ye-Kui Wang                   <wyk@ieee.org>
 *
 *************************************************************************************
 */

#include <stdlib.h>
#include <assert.h>
#include "mbuffer.h"
#include "global.h"
#include "memalloc.h"
#include "erc_do.h"
#include "image.h"

extern int erc_mvperMB;
struct img_par *erc_img;

// static function declarations
static int concealByCopy(frame *recfr, int currMBNum,
  objectBuffer_t *object_list, int32 picSizeX);
static int concealByTrial(frame *recfr, byte *predMB, 
                          int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                          int32 picSizeX, int32 picSizeY, int *yCondition);
static int edgeDistortion (int predBlocks[], int currYBlockNum, byte *predMB, 
                           byte *recY, int32 picSizeX, int32 regionSize);
static void copyBetweenFrames (frame *recfr, 
   int currYBlockNum, int32 picSizeX, int32 regionSize);
static void buildPredRegionYUV(struct img_par *img, int32 *mv, int x, int y, byte *predMB);
static void copyPredMB (int currYBlockNum, byte *predMB, frame *recfr, 
                        int32 picSizeX, int32 regionSize);

/*!
 ************************************************************************
 * \brief
 *      The main function for Inter (P) frame concealment.
 * \return
 *      0, if the concealment was not successful and simple concealment should be used
 *      1, otherwise (even if none of the blocks were concealed)
 * \param recfr
 *      Reconstructed frame buffer
 * \param object_list
 *      Motion info for all MBs in the frame
 * \param picSizeX
 *      Width of the frame in pixels
 * \param picSizeY
 *      Height of the frame in pixels
 * \param errorVar   
 *      Variables for error concealment
 ************************************************************************
 */
int ercConcealInterFrame(frame *recfr, objectBuffer_t *object_list, 
                         int32 picSizeX, int32 picSizeY, ercVariables_t *errorVar ) 
{
  int lastColumn = 0, lastRow = 0, predBlocks[8];
  int lastCorruptedRow = -1, firstCorruptedRow = -1, currRow = 0, 
    row, column, columnInd, areaHeight = 0, i = 0;
  byte *predMB;
  
  /* if concealment is on */
  if ( errorVar && errorVar->concealment ) 
  {
    /* if there are segments to be concealed */
    if ( errorVar->nOfCorruptedSegments ) 
    {
      
      predMB = (byte *) malloc(DEF_REGION_SIZE);
      if ( predMB == NULL ) no_mem_exit("ercConcealInterFrame: predMB");
      
      lastRow = (int) (picSizeY>>4);
      lastColumn = (int) (picSizeX>>4);
      
      for ( columnInd = 0; columnInd < lastColumn; columnInd ++) 
      {
        
        column = ((columnInd%2) ? (lastColumn - columnInd/2 -1) : (columnInd/2));
        
        for ( row = 0; row < lastRow; row++) 
        {
          
          if ( errorVar->yCondition[MBxy2YBlock(column, row, 0, picSizeX)] <= ERC_BLOCK_CORRUPTED ) 
          {                           // ERC_BLOCK_CORRUPTED (1) or ERC_BLOCK_EMPTY (0)
            firstCorruptedRow = row;
            /* find the last row which has corrupted blocks (in same continuous area) */
            for ( lastCorruptedRow = row+1; lastCorruptedRow < lastRow; lastCorruptedRow++) 
            {
              /* check blocks in the current column */
              if (errorVar->yCondition[MBxy2YBlock(column, lastCorruptedRow, 0, picSizeX)] > ERC_BLOCK_CORRUPTED) 
              {
                /* current one is already OK, so the last was the previous one */
                lastCorruptedRow --;
                break;
              }
            }
            if ( lastCorruptedRow >= lastRow ) 
            {
              /* correct only from above */
              lastCorruptedRow = lastRow-1;
              for ( currRow = firstCorruptedRow; currRow < lastRow; currRow++ ) 
              {
                
                ercCollect8PredBlocks (predBlocks, (currRow<<1), (column<<1), 
                  errorVar->yCondition, (lastRow<<1), (lastColumn<<1), 2, 0);      
                
                if(erc_mvperMB >= MVPERMB_THR)
                  concealByTrial(recfr, predMB, 
                    currRow*lastColumn+column, object_list, predBlocks, 
                    picSizeX, picSizeY,
                    errorVar->yCondition);
                else 
                  concealByCopy(recfr, currRow*lastColumn+column, 
                    object_list, picSizeX);
                
                ercMarkCurrMBConcealed (currRow*lastColumn+column, -1, picSizeX, errorVar);
              }
              row = lastRow;
            } 
            else if ( firstCorruptedRow == 0 ) 
            {
              /* correct only from below */
              for ( currRow = lastCorruptedRow; currRow >= 0; currRow-- ) 
              {
                
                ercCollect8PredBlocks (predBlocks, (currRow<<1), (column<<1), 
                  errorVar->yCondition, (lastRow<<1), (lastColumn<<1), 2, 0);      
                
                if(erc_mvperMB >= MVPERMB_THR)
                  concealByTrial(recfr, predMB, 
                    currRow*lastColumn+column, object_list, predBlocks, 
                    picSizeX, picSizeY,
                    errorVar->yCondition);
                else 
                  concealByCopy(recfr, currRow*lastColumn+column, 
                    object_list, picSizeX);
                
                ercMarkCurrMBConcealed (currRow*lastColumn+column, -1, picSizeX, errorVar);
              }
              
              row = lastCorruptedRow+1;
            }
            else 
            {
              /* correct bi-directionally */
              
              row = lastCorruptedRow+1;
              
              areaHeight = lastCorruptedRow-firstCorruptedRow+1;
              
              /* 
              *  Conceal the corrupted area switching between the up and the bottom rows 
              */
              for ( i = 0; i < areaHeight; i++) 
              {
                if ( i % 2 ) 
                {
                  currRow = lastCorruptedRow;
                  lastCorruptedRow --;
                }
                else 
                {
                  currRow = firstCorruptedRow;
                  firstCorruptedRow ++; 
                }
                
                ercCollect8PredBlocks (predBlocks, (currRow<<1), (column<<1), 
                  errorVar->yCondition, (lastRow<<1), (lastColumn<<1), 2, 0);      
                
                if(erc_mvperMB >= MVPERMB_THR)
                  concealByTrial(recfr, predMB, 
                    currRow*lastColumn+column, object_list, predBlocks, 
                    picSizeX, picSizeY,
                    errorVar->yCondition);
                else
                  concealByCopy(recfr, currRow*lastColumn+column, 
                    object_list, picSizeX);
                
                ercMarkCurrMBConcealed (currRow*lastColumn+column, -1, picSizeX, errorVar);
                
              }
            }
            lastCorruptedRow = -1;
            firstCorruptedRow = -1;
          }
        }
      }
    
      free(predMB);
    }
    return 1;
  }
  else
    return 0;
}

/*!
 ************************************************************************
 * \brief
 *      It conceals a given MB by simply copying the pixel area from the reference image 
 *      that is at the same location as the macroblock in the current image. This correcponds 
 *      to COPY MBs. 
 * \return
 *      Always zero (0).
 * \param recfr
 *      Reconstructed frame buffer
 * \param currMBNum
 *      current MB index
 * \param object_list
 *      Motion info for all MBs in the frame
 * \param picSizeX
 *      Width of the frame in pixels
 ************************************************************************
 */
static int concealByCopy(frame *recfr, int currMBNum,
  objectBuffer_t *object_list, int32 picSizeX)
{
  objectBuffer_t *currRegion;
   
  currRegion = object_list+(currMBNum<<2);
  currRegion->regionMode = REGMODE_INTER_COPY;
   
  currRegion->xMin = (xPosMB(currMBNum,picSizeX)<<4);
  currRegion->yMin = (yPosMB(currMBNum,picSizeX)<<4);
   
  copyBetweenFrames (recfr, MBNum2YBlock(currMBNum,0,picSizeX), picSizeX, 16);
   
  return 0;
}

/*!
 ************************************************************************
 * \brief
 *      Copies the co-located pixel values from the reference to the current frame. 
 *      Used by concealByCopy
 * \param recfr
 *      Reconstructed frame buffer
 * \param currYBlockNum
 *      index of the block (8x8) in the Y plane
 * \param picSizeX
 *      Width of the frame in pixels
 * \param regionSize      
 *      can be 16 or 8 to tell the dimension of the region to copy
 ************************************************************************
 */
static void copyBetweenFrames (frame *recfr, 
   int currYBlockNum, int32 picSizeX, int32 regionSize)
{
  int j, k, location, xmin, ymin;
  StorablePicture* refPic = listX[0][0];
   
  /* set the position of the region to be copied */
  xmin = (xPosYBlock(currYBlockNum,picSizeX)<<3);
  ymin = (yPosYBlock(currYBlockNum,picSizeX)<<3);
   
  for (j = ymin; j < ymin + regionSize; j++)
    for (k = xmin; k < xmin + regionSize; k++)
    {
      location = j * picSizeX + k; 
//th      recfr->yptr[location] = dec_picture->imgY[j][k];
      recfr->yptr[location] = refPic->imgY[j][k];
    }
     
    for (j = ymin / 2; j < (ymin + regionSize) / 2; j++)
      for (k = xmin / 2; k < (xmin + regionSize) / 2; k++)
      {
        location = j * picSizeX / 2 + k;
//th        recfr->uptr[location] = dec_picture->imgUV[0][j][k];
//th        recfr->vptr[location] = dec_picture->imgUV[1][j][k];
        recfr->uptr[location] = refPic->imgUV[0][j][k];
        recfr->vptr[location] = refPic->imgUV[1][j][k];
      }                                
}

/*!
 ************************************************************************
 * \brief
 *      It conceals a given MB by using the motion vectors of one reliable neighbor. That MV of a 
 *      neighbor is selected wich gives the lowest pixel difference at the edges of the MB 
 *      (see function edgeDistortion). This corresponds to a spatial smoothness criteria.
 * \return
 *      Always zero (0).
 * \param recfr
 *      Reconstructed frame buffer
 * \param predMB
 *      memory area for storing temporary pixel values for a macroblock
 *      the Y,U,V planes are concatenated y = predMB, u = predMB+256, v = predMB+320
 * \param currMBNum
 *      current MB index
 * \param object_list
 *      array of region structures storing region mode and mv for each region
 * \param predBlocks
 *      status array of the neighboring blocks (if they are OK, concealed or lost)
 * \param picSizeX
 *      Width of the frame in pixels
 * \param picSizeY
 *      Height of the frame in pixels
 * \param yCondition
 *      array for conditions of Y blocks from ercVariables_t
 ************************************************************************
 */
static int concealByTrial(frame *recfr, byte *predMB, 
                          int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                          int32 picSizeX, int32 picSizeY, int *yCondition)
{
  int predMBNum = 0, numMBPerLine,
      compSplit1 = 0, compSplit2 = 0, compLeft = 1, comp = 0, compPred, order = 1,
      fInterNeighborExists, numIntraNeighbours,
      fZeroMotionChecked, predSplitted = 0,
      threshold = ERC_BLOCK_OK,
      minDist, currDist, i, k, bestDir;
  int32 regionSize;
  objectBuffer_t *currRegion;
  int32 mvBest[3] , mvPred[3], *mvptr;
  
  numMBPerLine = (int) (picSizeX>>4);
  
  comp = 0;
  regionSize = 16;
  
  do 
  { /* 4 blocks loop */
    
    currRegion = object_list+(currMBNum<<2)+comp;
    
    /* set the position of the region to be concealed */
    
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    
    do 
    { /* reliability loop */
      
      minDist = 0; 
      fInterNeighborExists = 0; 
      numIntraNeighbours = 0; 
      fZeroMotionChecked = 0;
      
      /* loop the 4 neighbours */
      for (i = 4; i < 8; i++) 
      {
        
        /* if reliable, try it */
        if (predBlocks[i] >= threshold) 
        {
          switch (i) 
          {
          case 4:
            predMBNum = currMBNum-numMBPerLine;
            compSplit1 = 2;
            compSplit2 = 3;
            break;
              
          case 5:
            predMBNum = currMBNum-1;
            compSplit1 = 1;
            compSplit2 = 3;
            break;
              
          case 6:
            predMBNum = currMBNum+numMBPerLine;
            compSplit1 = 0;
            compSplit2 = 1;
            break;
              
          case 7:
            predMBNum = currMBNum+1;
            compSplit1 = 0;
            compSplit2 = 2;
            break;
          }
          
          /* try the concealment with the Motion Info of the current neighbour
          only try if the neighbour is not Intra */
          if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
            isBlock(object_list,predMBNum,compSplit2,INTRA))
          {            
            numIntraNeighbours++;
          } 
          else 
          {
            /* if neighbour MB is splitted, try both neighbour blocks */
            for (predSplitted = isSplitted(object_list, predMBNum), 
              compPred = compSplit1;
              predSplitted >= 0;
              compPred = compSplit2,
              predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
            {
              
              /* if Zero Motion Block, do the copying. This option is tried only once */
              if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
              {
                
                if (fZeroMotionChecked) 
                {
                  continue;
                }
                else 
                {
                  fZeroMotionChecked = 1;

                  mvPred[0] = mvPred[1] = 0;
                  mvPred[2] = 0;
                  
                  buildPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB);
                }
              }
              /* build motion using the neighbour's Motion Parameters */
              else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
              {
                continue;
              }
              else 
              {
                mvptr = getParam(object_list, predMBNum, compPred, mv);
                
                mvPred[0] = mvptr[0];
                mvPred[1] = mvptr[1];
                mvPred[2] = mvptr[2];

                buildPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB);
              }
              
              /* measure absolute boundary pixel difference */
              currDist = edgeDistortion(predBlocks, 
                MBNum2YBlock(currMBNum,comp,picSizeX),
                predMB, recfr->yptr, picSizeX, regionSize);
              
              /* if so far best -> store the pixels as the best concealment */
              if (currDist < minDist || !fInterNeighborExists) 
              {
                
                minDist = currDist;
                bestDir = i;
                
                for (k=0;k<3;k++) 
                  mvBest[k] = mvPred[k];
                
                currRegion->regionMode = 
                  (isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
                  ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
                  ((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);
                
                copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, 
                  picSizeX, regionSize);
              }
              
              fInterNeighborExists = 1;
            }
          }
        }
    }
    
    threshold--;
    
    } while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
    /* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0;
      mvPred[2] = 0;

      buildPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB);
      
      currDist = edgeDistortion(predBlocks, 
        MBNum2YBlock(currMBNum,comp,picSizeX),
        predMB, recfr->yptr, picSizeX, regionSize);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {
        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = 
          ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);
        
        copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, 
          picSizeX, regionSize);
      }
    }
    
    for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];
    
    yCondition[MBNum2YBlock(currMBNum,comp,picSizeX)] = ERC_BLOCK_CONCEALED;
    comp = (comp+order+4)%4;
    compLeft--;
    
    } while (compLeft);
    
    return 0;
}

/*!
 ************************************************************************
 * \brief
 *      Builds the motion prediction pixels from the given location (in 1/4 pixel units) 
 *      of the reference frame. It not only copies the pixel values but builds the interpolation 
 *      when the pixel positions to be copied from is not full pixel (any 1/4 pixel position).
 *      It copies the resulting pixel vlaues into predMB.
 * \param img
 *      The pointer of img_par struture of current frame
 * \param mv
 *      The pointer of the predicted MV of the current (being concealed) MB
 * \param x
 *      The x-coordinate of the above-left corner pixel of the current MB
 * \param y
 *      The y-coordinate of the above-left corner pixel of the current MB
 * \param predMB
 *      memory area for storing temporary pixel values for a macroblock
 *      the Y,U,V planes are concatenated y = predMB, u = predMB+256, v = predMB+320
 ************************************************************************
 */
static void buildPredRegionYUV(struct img_par *img, int32 *mv, int x, int y, byte *predMB)
{
  int tmp_block[BLOCK_SIZE][BLOCK_SIZE];
  int i=0,j=0,ii=0,jj=0,i1=0,j1=0,j4=0,i4=0;
  int jf=0;
  int uv;
  int vec1_x=0,vec1_y=0;
  int ioff,joff;
  byte *pMB = predMB;

  int ii0,jj0,ii1,jj1,if1,jf1,if0,jf0;
  int mv_mul,f1,f2,f3,f4;

  int ref_frame = mv[2];

  /* Update coordinates of the current concealed macroblock */
  img->mb_x = x/MB_BLOCK_SIZE;
  img->mb_y = y/MB_BLOCK_SIZE;
  img->block_y = img->mb_y * BLOCK_SIZE;
  img->pix_c_y = img->mb_y * MB_BLOCK_SIZE/2;
  img->block_x = img->mb_x * BLOCK_SIZE;
  img->pix_c_x = img->mb_x * MB_BLOCK_SIZE/2;

  mv_mul=4;
  f1=8;
  f2=7;

  f3=f1*f1;
  f4=f3/2;

  // luma *******************************************************

  for(j=0;j<MB_BLOCK_SIZE/BLOCK_SIZE;j++)
  {
    joff=j*4;
    j4=img->block_y+j;
    for(i=0;i<MB_BLOCK_SIZE/BLOCK_SIZE;i++)
    {
      ioff=i*4;
      i4=img->block_x+i;

      vec1_x = i4*4*mv_mul + mv[0];
      vec1_y = j4*4*mv_mul + mv[1];

      get_block(ref_frame, listX[0], vec1_x,vec1_y,img,tmp_block);

      for(ii=0;ii<BLOCK_SIZE;ii++)
        for(jj=0;jj<MB_BLOCK_SIZE/BLOCK_SIZE;jj++)
          img->mpr[ii+ioff][jj+joff]=tmp_block[ii][jj];
    }
  }

  for (i = 0; i < 16; i++)
  {
    for (j = 0; j < 16; j++)
    {
      pMB[i*16+j] = img->mpr[j][i];
    }
  }
  pMB += 256;

  // chroma *******************************************************
  for(uv=0;uv<2;uv++)
  {
    for (j=4;j<6;j++)
    {
      joff=(j-4)*4;
      j4=img->pix_c_y+joff;
      for(i=0;i<2;i++)
      {
        ioff=i*4;
        i4=img->pix_c_x+ioff;
        for(jj=0;jj<4;jj++)
        {
          jf=(j4+jj)/2;
          for(ii=0;ii<4;ii++)
          {
            if1=(i4+ii)/2;
            i1=(img->pix_c_x+ii+ioff)*f1+mv[0];
            j1=(img->pix_c_y+jj+joff)*f1+mv[1];


            ii0=max (0, min (i1/f1, dec_picture->size_x_cr-1));
            jj0=max (0, min (j1/f1, dec_picture->size_y_cr-1));
            ii1=max (0, min ((i1+f2)/f1, dec_picture->size_x_cr-1));
            jj1=max (0, min ((j1+f2)/f1, dec_picture->size_y_cr-1));

            if1=(i1 & f2);
            jf1=(j1 & f2);
            if0=f1-if1;
            jf0=f1-jf1;
            img->mpr[ii+ioff][jj+joff]=(if0*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii0]+
              if1*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii1]+
              if0*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii0]+
              if1*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii1]+f4)/f3;

          }
        }
      }
    }

    for (i = 0; i < 8; i++)
    {
      for (j = 0; j < 8; j++)
      {
        pMB[i*8+j] = img->mpr[j][i];
      }
    }
    pMB += 64;

  }
}
/*!
 ************************************************************************
 * \brief
 *      Copies pixel values between a YUV frame and the temporary pixel value storage place. This is
 *      used to save some pixel values temporarily before overwriting it, or to copy back to a given 
 *      location in a frame the saved pixel values.
 * \param currYBlockNum   
 *      index of the block (8x8) in the Y plane
 * \param predMB          
 *      memory area where the temporary pixel values are stored
 *      the Y,U,V planes are concatenated y = predMB, u = predMB+256, v = predMB+320
 * \param recfr           
 *      pointer to a YUV frame
 * \param picSizeX        
 *      picture width in pixels
 * \param regionSize      
 *      can be 16 or 8 to tell the dimension of the region to copy
 ************************************************************************
 */
static void copyPredMB (int currYBlockNum, byte *predMB, frame *recfr, 
                        int32 picSizeX, int32 regionSize) 
{
  
  int j, k, xmin, ymin, xmax, ymax;
  int32 locationTmp, locationPred;
  
  xmin = (xPosYBlock(currYBlockNum,picSizeX)<<3);
  ymin = (yPosYBlock(currYBlockNum,picSizeX)<<3);
  xmax = xmin + regionSize -1;
  ymax = ymin + regionSize -1;
  
  for (j = ymin; j <= ymax; j++) 
  {
    for (k = xmin; k <= xmax; k++)
    {
      locationPred = j * picSizeX + k;
      locationTmp = (j-ymin) * 16 + (k-xmin);
      dec_picture->imgY[j][k] = predMB[locationTmp];
    }
  }
  
  for (j = (ymin>>1); j <= (ymax>>1); j++) 
  {
    for (k = (xmin>>1); k <= (xmax>>1); k++)
    {
      locationPred = j * picSizeX / 2 + k;
      locationTmp = (j-(ymin>>1)) * 8 + (k-(xmin>>1)) + 256;
      dec_picture->imgUV[0][j][k] = predMB[locationTmp];
      
      locationTmp += 64;
      
      dec_picture->imgUV[1][j][k] = predMB[locationTmp];
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *      Calculates a weighted pixel difference between edge Y pixels of the macroblock stored in predMB
 *      and the pixels in the given Y plane of a frame (recY) that would become neighbor pixels if 
 *      predMB was placed at currYBlockNum block position into the frame. This "edge distortion" value
 *      is used to determine how well the given macroblock in predMB would fit into the frame when
 *      considering spatial smoothness. If there are correctly received neighbor blocks (status stored 
 *      in predBlocks) only they are used in calculating the edge distorion; otherwise also the already
 *      concealed neighbor blocks can also be used.
 * \return 
 *      The calculated weighted pixel difference at the edges of the MB.
 * \param predBlocks      
 *      status array of the neighboring blocks (if they are OK, concealed or lost)
 * \param currYBlockNum   
 *      index of the block (8x8) in the Y plane
 * \param predMB          
 *      memory area where the temporary pixel values are stored
 *      the Y,U,V planes are concatenated y = predMB, u = predMB+256, v = predMB+320
 * \param recY            
 *      pointer to a Y plane of a YUV frame
 * \param picSizeX        
 *      picture width in pixels
 * \param regionSize      
 *      can be 16 or 8 to tell the dimension of the region to copy
 ************************************************************************
 */
static int edgeDistortion (int predBlocks[], int currYBlockNum, byte *predMB, 
                           byte *recY, int32 picSizeX, int32 regionSize)
{
  int i, j, distortion, numOfPredBlocks, threshold = ERC_BLOCK_OK;
  byte *currBlock = NULL, *neighbor = NULL;
  int32 currBlockOffset = 0;
  
  currBlock = recY + (yPosYBlock(currYBlockNum,picSizeX)<<3)*picSizeX + (xPosYBlock(currYBlockNum,picSizeX)<<3);
  
  do 
  {
    
    distortion = 0; numOfPredBlocks = 0;
    
    /* loop the 4 neighbours */
    for (j = 4; j < 8; j++) 
    {
      /* if reliable, count boundary pixel difference */
      if (predBlocks[j] >= threshold) 
      {
        
        switch (j) 
        {
        case 4:
          neighbor = currBlock - picSizeX;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs(predMB[i] - neighbor[i]);
          }
          break;          
        case 5:
          neighbor = currBlock - 1;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs(predMB[i*16] - neighbor[i*picSizeX]);
          }
          break;                
        case 6:
          neighbor = currBlock + regionSize*picSizeX;
          currBlockOffset = (regionSize-1)*16;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs(predMB[i+currBlockOffset] - neighbor[i]);
          }
          break;                
        case 7:
          neighbor = currBlock + regionSize;
          currBlockOffset = regionSize-1;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs(predMB[i*16+currBlockOffset] - neighbor[i*picSizeX]);
          }
          break;
        }
        
        numOfPredBlocks++;
      }
    }
    
    threshold--;
    if (threshold < ERC_BLOCK_CONCEALED)
      break;
  } while (numOfPredBlocks == 0);
  
  if(numOfPredBlocks == 0)
    assert (numOfPredBlocks != 0);
  return (distortion/numOfPredBlocks);
}

