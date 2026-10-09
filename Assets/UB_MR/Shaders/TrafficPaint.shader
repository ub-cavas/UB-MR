Shader "UB-MR/Traffic Paint"
{
    Properties
    {
        [MainTexture] _BaseMap("Original body/livery", 2D) = "white" {}
        [MainColor] _BaseColor("Paint", Color) = (1,1,1,1)
        _PaintMask("Paint mask (white = paint, black = preserve)", 2D) = "white" {}
        _BumpMap("Normal", 2D) = "bump" {}
        _BumpScale("Normal scale", Float) = 1
        _Smoothness("Smoothness", Range(0,1)) = 0.65
        _Metallic("Metallic", Range(0,1)) = 0.35
        [HideInInspector] _MetallicGlossMap("Metallic", 2D) = "white" {}
        [HideInInspector] _SpecColor("Specular", Color) = (0.2,0.2,0.2,1)
        [HideInInspector] _SpecGlossMap("Specular", 2D) = "white" {}
        [HideInInspector] _OcclusionMap("Occlusion", 2D) = "white" {}
        [HideInInspector] _OcclusionStrength("Occlusion strength", Float) = 1
        [HideInInspector] _EmissionMap("Emission", 2D) = "white" {}
        [HideInInspector] _EmissionColor("Emission color", Color) = (0,0,0,0)
        [HideInInspector] _Cutoff("Cutoff", Float) = 0.5
        [HideInInspector] _Surface("Surface", Float) = 0
        [HideInInspector] _Cull("Cull", Float) = 2
        [HideInInspector] _AlphaClip("Alpha clip", Float) = 0
        [HideInInspector] _SrcBlend("Source blend", Float) = 1
        [HideInInspector] _DstBlend("Destination blend", Float) = 0
        [HideInInspector] _ZWrite("Depth write", Float) = 1
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "RenderPipeline"="UniversalPipeline" "UniversalMaterialType"="Lit" }
        Pass
        {
            Name "ForwardLit"
            Tags { "LightMode"="UniversalForwardOnly" }
            Cull Back
            ZWrite On
            HLSLPROGRAM
            #pragma target 3.0
            #pragma vertex LitPassVertex
            #pragma fragment LitPassFragment
            #pragma shader_feature_local _NORMALMAP
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE _MAIN_LIGHT_SHADOWS_SCREEN
            #pragma multi_compile _ _ADDITIONAL_LIGHTS_VERTEX _ADDITIONAL_LIGHTS
            #pragma multi_compile_fragment _ _ADDITIONAL_LIGHT_SHADOWS
            #pragma multi_compile_fragment _ _SHADOWS_SOFT
            #pragma multi_compile _ _FORWARD_PLUS
            #pragma multi_compile_fog
            #pragma multi_compile_instancing
            #include "Packages/com.unity.render-pipelines.universal/Shaders/LitInput.hlsl"
            TEXTURE2D(_PaintMask);
            SAMPLER(sampler_PaintMask);
            void InitializeTrafficSurfaceData(float2 uv, out SurfaceData surfaceData)
            {
                InitializeStandardLitSurfaceData(uv, surfaceData);
                half3 original = SampleAlbedoAlpha(uv, TEXTURE2D_ARGS(_BaseMap, sampler_BaseMap)).rgb;
                half mask = SAMPLE_TEXTURE2D(_PaintMask, sampler_PaintMask, uv).r;
                surfaceData.albedo = lerp(original, _BaseColor.rgb, mask);
            }
            #define InitializeStandardLitSurfaceData InitializeTrafficSurfaceData
            #include "Packages/com.unity.render-pipelines.universal/Shaders/LitForwardPass.hlsl"
            ENDHLSL
        }
        UsePass "Universal Render Pipeline/Lit/ShadowCaster"
        UsePass "Universal Render Pipeline/Lit/DepthOnly"
        UsePass "Universal Render Pipeline/Lit/DepthNormals"
    }
    FallBack "Hidden/Universal Render Pipeline/FallbackError"
}
