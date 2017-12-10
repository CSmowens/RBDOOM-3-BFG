//--------------------------------------------------------------------------------------
// Copyright 2012 Intel Corporation
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
//--------------------------------------------------------------------------------------
// Generated by ShaderGenerator.exe version 0.1
//--------------------------------------------------------------------------------------

// -------------------------------------
cbuffer cbPerModelValues
{
    row_major float4x4 World : WORLD;
    row_major float4x4 WorldViewProjection : WORLDVIEWPROJECTION;
    row_major float4x4 InverseWorld : INVERSEWORLD;
              float3   LightDirection  : Direction < string UIName = "Light Direction";  string Object = "TargetLight"; int Ref_ID=0; >;
              float4   EyePosition;
    row_major float4x4 LightWorldViewProjection;
};

// -------------------------------------
// TODO: Note: nothing sets these values yet
cbuffer cbPerFrameValues
{
    row_major float4x4  View;
    row_major float4x4  Projection;
              float3    AmbientColor;
              float3    LightColor;
              float3    TotalTimeInSeconds;
};

struct VS_INPUT
{
    float3 Position : POSITION; // Projected position
    float3 Normal   : NORMAL;
    float3 Tangent  : TANGENT;
    float3 Binormal : BINORMAL;
    float2 UV0      : TEXCOORD0;
};

// -------------------------------------
struct PS_INPUT
{
    float4 Position : SV_POSITION;
    float3 Normal   : NORMAL;
    float3 Tangent  : TANGENT;
    float3 Binormal : BINORMAL;
    float2 UV0      : TEXCOORD0;
};

// -------------------------------------
#ifdef _CPUT
    SamplerState SAMPLER0 : register( s0 );
    SamplerComparisonState SHADOW_SAMPLER : register( s1);
    Texture2D texture0 : register( t0 );
    Texture2D texture1 : register( t1 );
    Texture2D _Shadow : register( t2 );
#else
    texture2D texture0 < string Name = "texture0"; string UIName = "texture0"; string ResourceType = "2D";>;
    sampler2D SAMPLER0 = sampler_state{ texture = (texture0);};
    texture2D texture1 < string Name = "texture1"; string UIName = "texture1"; string ResourceType = "2D";>;
    sampler2D SAMPLER1 = sampler_state{ texture = (texture1);};
#endif

// -------------------------------------
float4 DIFFUSE( PS_INPUT input )
{
    return 
#ifdef _CPUT
texture0.Sample( SAMPLER0, (((input.UV0)) *(1)) )
#else
tex2D( SAMPLER0, (((input.UV0)) *(1)) )
#endif
;
}

// -------------------------------------
float4 SPECULAR( PS_INPUT input )
{
    return (DIFFUSE(input)) *(0.5);
}

// -------------------------------------
float4 NORMAL( PS_INPUT input )
{
    return ((
#ifdef _CPUT
texture1.Sample( SAMPLER0, (((input.UV0)) *(1)) )
#else
tex2D( SAMPLER1, (((input.UV0)) *(1)) )
#endif
) *(2)) -(1);
}

// -------------------------------------
float4 AMBIENT( PS_INPUT input )
{
    return DIFFUSE(input);
}

// -------------------------------------
PS_INPUT VSMain( VS_INPUT input )
{
    PS_INPUT output = (PS_INPUT)0;

    output.Position      = mul( float4( input.Position, 1.0f), WorldViewProjection );
    
    // TODO: transform the light into object space instead of the normal into world space
    output.Normal   = mul( input.Normal, (float3x3)World );
    output.Tangent  = mul( input.Tangent, (float3x3)World );
    output.Binormal = mul( input.Binormal, (float3x3)World );
    output.UV0 = input.UV0;
    
    return output;
}

// -------------------------------------
float4 PSMain( PS_INPUT input ) : SV_Target
{
    float4 result = float4(0,0,0,1);

    float3 normal   = input.Normal;
    float3 tangent  = input.Tangent;
    float3 binormal = input.Binormal;
    float3x3 worldToTangent = float3x3(tangent, binormal, normal);
    normal = normalize( mul( NORMAL(input), worldToTangent ));

    // Ambient-related computation
    float3 ambient = AmbientColor * AMBIENT(input);
    result.xyz +=  ambient;
#ifdef _CPUT
   float3 lightDirection = -LightDirection;
#else
   float3 lightDirection = LightDirection;
#endif

    // Diffuse-related computation
    float  nDotL         = saturate( dot( normal, lightDirection ) );
    float3 diffuse       = LightColor * nDotL * DIFFUSE(input);
    result.xyz += diffuse;

    float alpha = step(0.5,DIFFUSE(input).a);
    clip(alpha ? 1 : -1);
    result.a = alpha;
    
	return result;
}

// -------------------------------------
technique DefaultTechnique
{
    pass pass1
    {
        VertexShader        = compile vs_3_0 VSMain();
        PixelShader         = compile ps_3_0 PSMain();
        ZWriteEnable        = true;
    }
}

