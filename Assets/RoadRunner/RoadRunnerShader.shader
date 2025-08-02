// Copyright 2020 The MathWorks, Inc.

// Basic shader for opaque materials.
Shader "Custom/RoadRunnerShader" {
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

	SubShader{
		Tags { "Queue" = "Geometry" "RenderType" = "Opaque" }
		Blend Off
		LOD 200
		ZWrite On
		Cull Back

		CGPROGRAM
		// Actually roughness based
		#pragma surface surf Standard fullforwardshadows
		#pragma target 3.0
		#pragma multi_compile __ RR_TRANSPARENT RR_LEAVES

		#include "UnityPBSLighting.cginc"
		#include "RoadRunnerSurface.cginc"
		
		ENDCG
	}
	FallBack "Diffuse"
}
