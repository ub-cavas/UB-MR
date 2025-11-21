using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class DepthPass : ScriptableRenderPass
{
    Material depthCaptureMaterial;
    RenderTargetIdentifier source;
    RTHandle tempTexture;
    RenderTexture depthRenderTexture;

    public DepthPass()
    {
        tempTexture = RTHandles.Alloc("_TempDepthTexture", name: "_TempDepthTexture");
    }

    public void Setup(Material material, RenderTargetIdentifier source)
    {
        this.depthCaptureMaterial = material;
        this.source = source;
    }

    public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData)
    {
        if (depthCaptureMaterial == null)
            return;

        CommandBuffer cmd = CommandBufferPool.Get("DepthCapture");
        RenderTextureDescriptor descriptor = renderingData.cameraData.cameraTargetDescriptor;
        descriptor.colorFormat = RenderTextureFormat.RFloat; // Single channel float for depth
        descriptor.depthBufferBits = 0;
        // Get or create the depth render texture from the manager
        RenderTexture depthRT = GetOrCreateDepthTexture(descriptor.width, descriptor.height);
        // Render the depth to our texture
        cmd.Blit(source, depthRT, depthCaptureMaterial);
        context.ExecuteCommandBuffer(cmd);
        CommandBufferPool.Release(cmd);
    }

    RenderTexture GetOrCreateDepthTexture(int width, int height)
    {
        if (depthRenderTexture == null || depthRenderTexture.width != width || depthRenderTexture.height != height)
        {
            if (depthRenderTexture != null)
                depthRenderTexture.Release();
                
            depthRenderTexture = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat);
            depthRenderTexture.name = "DepthCaptureTexture";
            depthRenderTexture.filterMode = FilterMode.Point; // No filtering for accurate depth values
            depthRenderTexture.Create();  
        }
        return depthRenderTexture;
    }

    
}
