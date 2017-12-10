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

#ifndef HELPERSCALAR_H
#define HELPERSCALAR_H
#include "precompiled.h"
//#include "../common/SoftMath.h"

class HelperScalar
{
   public:
      HelperScalar();
      ~HelperScalar();

   protected:

      struct int4
      {
         int x, y, z, w;

         int4(const idVec4& f4)
         {
            x = (int)(f4.x + 0.5);
            y = (int)(f4.y + 0.5);
            z = (int)(f4.z + 0.5);
            w = (int)(f4.w + 0.5);
         }
      };

	  idVec4 TransformCoords(const idVec3& v, const idVec4& m);
	  idVec4 TransformCoords(const idVec4& v, const idVec4& m);
};

struct float4x4; 

struct BoxTestSetupScalar : public HelperScalar
{
	idRenderMatrix mViewProjViewport;
   float radiusThreshold;

   void Init(const idRenderMatrix &viewMatrix, const idRenderMatrix &projMatrix, const idRenderMatrix &viewportMatix, float fov, float sizeThreshold);
}; 

#endif