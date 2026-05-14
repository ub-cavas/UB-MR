using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering.Universal;

/// <summary>
/// Renderer feature that captures the scene depth into RenderTextures registered
/// by DepthCamera components. Keeps material/RT management in one place and uses
/// a URP render pass instead of Camera.AddCommandBuffer.
/// </summary>
public class DepthCaptureRenderFeature : ScriptableRendererFeature
{
    public class Request
    {
        public RenderTexture Target;
        public Material Material;
    }

    static readonly Dictionary<Camera, Request> s_Requests = new Dictionary<Camera, Request>();

    DepthPass depthPass;
    Material sharedDepthMaterial;

    public static void Register(Camera camera, RenderTexture target, Material material)
    {
        if (camera == null || target == null || material == null)
            return;

        s_Requests[camera] = new Request { Target = target, Material = material };
    }

    public static void Unregister(Camera camera)
    {
        if (camera == null)
            return;

        s_Requests.Remove(camera);
    }

    public override void Create()
    {
        // Create a shared material from the depth shader if one isn't supplied by the caller.
        sharedDepthMaterial = Shader.Find("Hidden/DepthCapture") != null
            ? new Material(Shader.Find("Hidden/DepthCapture"))
            : null;

        depthPass = new DepthPass(s_Requests);
    }

    public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
    {
        // If a request didn't provide its own material, fall back to the shared instance.
        if (sharedDepthMaterial != null &&
            s_Requests.TryGetValue(renderingData.cameraData.camera, out var request) &&
            request.Material == null)
        {
            request.Material = sharedDepthMaterial;
        }

        renderer.EnqueuePass(depthPass);
    }

    protected override void Dispose(bool disposing)
    {
        if (sharedDepthMaterial != null)
        {
#if UNITY_EDITOR
            Object.DestroyImmediate(sharedDepthMaterial);
#else
            Object.Destroy(sharedDepthMaterial);
#endif
            sharedDepthMaterial = null;
        }
    }
}
