//--------------------------------------------------------------------------------------
// Copyright 2013 Intel Corporation
// All Rights Reserved
//
// Permission is granted to use, copy, distribute and prepare derivative works of this
// software for any purpose and without fee, provided, that the above copyright notice
// and this statement appear in all copies.  Intel makes no representations about the
// suitability of this software for any purpose.  THIS SOFTWARE IS PROVIDED "AS IS."
// INTEL SPECIFICALLY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, AND ALL LIABILITY,
// INCLUDING CONSEQUENTIAL AND OTHER INDIRECT DAMAGES, FOR THE USE OF THIS SOFTWARE,
// INCLUDING LIABILITY FOR INFRINGEMENT OF ANY PROPRIETARY RIGHTS, AND INCLUDING THE
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  Intel does not
// assume any responsibility for any errors which may appear in this software nor any
// responsibility to update it.
//
//--------------------------------------------------------------------------------------

#ifndef AABBOXRASTERIZER_H
#define AABBOXRASTERIZER_H

#include "../common/SoftOccludeeScalar.h"

class AABBoxRasterizer
{
   public:
      AABBoxRasterizer(RasterizerData* rasterData);
      virtual ~AABBoxRasterizer();
      virtual void TransformAABBoxAndDepthTest(SoftFrustum *pFrustum, float pFov, uint idx) = 0;
      virtual void WaitForTaskToFinish(uint idx) = 0;
      virtual void ReleaseTaskHandles(UINT idx) = 0;

      virtual void ResetInsideFrustum() = 0; 
      virtual void SetViewProjMatrix(idRenderMatrix *viewMatrix, idRenderMatrix *projMatrix, UINT idx) = 0;
      virtual void SetCPURenderTargetPixels(UINT *pRenderTargetPixels, UINT idx) = 0;
      virtual void SetDepthSummaryBuffer(const float *pDepthSummary, UINT idx) = 0;
      virtual void SetDepthTestTasks(UINT numTasks) = 0;
      virtual void SetOccludeeSizeThreshold(float occludeeSizeThreshold) = 0;
      virtual void SetEnableFCulling(bool enableFCulling) = 0;

      virtual UINT GetNumOccludees() = 0;
      virtual UINT GetNumCulled(UINT idx) = 0;
      virtual UINT GetNumTriangles() = 0;
      virtual UINT GetNumCulledTriangles(UINT idx) = 0;
      virtual UINT GetNumTrisRendered() = 0;
      virtual UINT GetNumFCullCount() = 0;

      // andrewmac:
      virtual SoftOccludeeScalar* AddOccludee() = 0;
      virtual bool IsVisible(UINT idx, UINT modelIdx) = 0;

   protected:
      RasterizerData* mRasterData;
};

#endif //AABBOXRASTERIZER_H