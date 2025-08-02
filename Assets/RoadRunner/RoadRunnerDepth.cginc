// Copyright 2020 The MathWorks, Inc.

#ifndef ROADRUNNER_DEPTH_INCLUDED
#define ROADRUNNER_DEPTH_INCLUDED

// Simple vertex and fragment shader that filters out values below
// the transparency threshold.

struct v2f 
{
	float4 vertex : SV_POSITION;
	float2 texcoord : TEXCOORD0;
	fixed4 color : COLOR;
};

sampler2D _MainTex;
float4 _MainTex_ST;
fixed4 _Color;
fixed _Cutoff;

v2f vert(appdata_full v)
{
	v2f o;
	o.vertex = UnityObjectToClipPos(v.vertex);
	o.texcoord = TRANSFORM_TEX(v.texcoord, _MainTex);
	o.color = v.color;
	return o;
}

fixed4 frag(v2f i) : SV_Target
{
	fixed4 color = tex2D(_MainTex, i.texcoord) * _Color * i.color;
	clip(color.a - _Cutoff);
	return 0;
}

#endif // ROADRUNNER_DEPTH_INCLUDED