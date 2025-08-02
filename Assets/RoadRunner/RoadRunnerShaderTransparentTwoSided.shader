// Copyright 2020 The MathWorks, Inc.

// Shader for transparent two-sided materials (typically leaves).
// Disables backface culling.
Shader "Custom/RoadRunnerShaderTransparentTwoSided" {
	Properties{
		_MainTex("Diffuse Map", 2D) = "white" {}
		_Color("Diffuse Color", Color) = (1,1,1,1)
		_BumpMap("Normal Map", 2D) = "bump" {}
		_RoughnessMap("Specular Map", 2D) = "white" {}
		_RoughnessFactor("Roughness Factor", Float) = 1.0
		_Emission("Emission", Float) = 0.0
		_EmissionColor("Emission Color", Color) = (0,0,0,1)
		_Cutoff("Alpha Cutoff", Range(0.01,1)) = 0.5
	}
	SubShader {
		Tags { "Queue" = "Transparent" "IgnoreProjector" = "True" "RenderType" = "TransparentCutout" }
		Blend SrcAlpha OneMinusSrcAlpha
		LOD 200
		ZWrite Off
		Cull Off

		// Write to the z-buffer as if cutout
		Pass {
			ColorMask 0
			ZWrite On

			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag

			#include "UnityCG.cginc"
			#include "RoadRunnerDepth.cginc"

			ENDCG
		}

		CGPROGRAM
		// Specular based lighting to closely match RoadRunner
		#pragma surface surf Standard fullforwardshadows alpha:fade nolightmap
		#pragma target 3.0
		#pragma multi_compile __ RR_TRANSPARENT RR_LEAVES
		
		#include "UnityPBSLighting.cginc"
		#include "RoadRunnerSurface.cginc"

		ENDCG
	}
	FallBack "Standard"
}
