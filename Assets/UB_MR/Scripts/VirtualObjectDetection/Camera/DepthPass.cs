using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

/// <summary>
/// URP render pass that blits the scene depth into user-provided RenderTextures.
/// DepthCamera registers its RenderTexture/Material per camera; this pass executes
/// AfterRendering and fills those targets so CPU readback returns real depth.
/// </summary>
public class DepthPass : ScriptableRenderPass
{
    Dictionary<Camera, DepthCaptureRenderFeature.Request> requests;

    public DepthPass(Dictionary<Camera, DepthCaptureRenderFeature.Request> requests)
    {
        this.requests = requests;
        renderPassEvent = RenderPassEvent.AfterRendering;
        ConfigureInput(ScriptableRenderPassInput.Depth); // ensure depth texture is available
    }

    public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData)
    {
        if (requests == null)
            return;

        var camera = renderingData.cameraData.camera;
        if (!requests.TryGetValue(camera, out var request))
            return;

        if (request == null || request.Material == null || request.Target == null)
            return;

        var cmd = CommandBufferPool.Get("DepthCapture");
        // Blit from depth texture (sampled in material) into the requested RT
        cmd.Blit(source: default, dest: request.Target, request.Material);
        context.ExecuteCommandBuffer(cmd);
        CommandBufferPool.Release(cmd);
    }
}
