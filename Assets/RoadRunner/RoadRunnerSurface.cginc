// Copyright 2020 The MathWorks, Inc.

#ifndef ROADRUNNER_SURFACE_INCLUDED
#define ROADRUNNER_SURFACE_INCLUDED

// Surface shader to match RoadRunner materials.

struct Input {
	float2 uv_MainTex;
	fixed facing : VFACE;
	fixed4 color : COLOR;
};

// Properties match Unity built-in ones so it can match fallback when needed.
sampler2D _MainTex;
sampler2D _BumpMap;
sampler2D _RoughnessMap;
half _RoughnessFactor;
fixed4 _Color;
float _Emission;
fixed _Cutoff;
fixed4 _EmissionColor;


void surf(Input IN, inout SurfaceOutputStandard o) {
	fixed4 color = tex2D(_MainTex, IN.uv_MainTex) * _Color * IN.color;
	o.Albedo = color.rgb;
	
#if RR_LEAVES
	o.Alpha = saturate(color.a / _Cutoff);
#elif RR_TRANSPARENT
	o.Alpha = saturate(color.a);
#else
	o.Alpha = 1.0;
#endif
	
	o.Metallic = 0.0;
	o.Smoothness = 1.0 - tex2D(_RoughnessMap, IN.uv_MainTex) * _RoughnessFactor;
	o.Normal = UnpackNormal(tex2D(_BumpMap, IN.uv_MainTex));
	o.Emission = _EmissionColor.rgb * _Emission * color.rgb;
	
	// Reverse normals for backfaces
	if (IN.facing < 0.5)
		o.Normal *= -1.0;
}

#endif // ROADRUNNER_SURFACE_INCLUDED