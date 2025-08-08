using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class DepthCaptureRenderFeature : ScriptableRendererFeature
{
    public class DepthCapturePass : ScriptableRenderPass
    {
        private Material depthCaptureMaterial;
        private RenderTargetIdentifier source;
        private RTHandle tempTexture;

        public DepthCapturePass()
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
            if (depthCaptureMaterial == null || DepthCaptureManager.Instance == null)
                return;
            
            CommandBuffer cmd = CommandBufferPool.Get("DepthCapture");
            
            RenderTextureDescriptor descriptor = renderingData.cameraData.cameraTargetDescriptor;
            descriptor.colorFormat = RenderTextureFormat.RFloat; // Single channel float for depth
            descriptor.depthBufferBits = 0;
            
            // Get or create the depth render texture from the manager
            RenderTexture depthRT = DepthCaptureManager.Instance.GetOrCreateDepthTexture(descriptor.width, descriptor.height);
            
            // Render the depth to our texture
            cmd.Blit(source, depthRT, depthCaptureMaterial);
            
            context.ExecuteCommandBuffer(cmd);
            CommandBufferPool.Release(cmd);
        }
    }
    
    private DepthCapturePass depthPass; 
    private Material depthCaptureMaterial;
    
    public override void Create()
    {
        depthPass = new DepthCapturePass();
        depthPass.renderPassEvent = RenderPassEvent.AfterRenderingTransparents;
        
        // Create the material from the shader
        Shader depthShader = Shader.Find("Hidden/DepthCapture");
        if (depthShader != null)
        {
            depthCaptureMaterial = new Material(depthShader);
        }
    }

    public override void SetupRenderPasses(ScriptableRenderer renderer, in RenderingData renderingData)
    {
        depthPass.Setup(depthCaptureMaterial, renderer.cameraColorTargetHandle);
    }
    
    public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
    {
        if (depthCaptureMaterial == null)
            return;

        renderer.EnqueuePass(depthPass);
    }
    
    protected override void Dispose(bool disposing)
    {
        if (depthCaptureMaterial != null)
        {
            #if UNITY_EDITOR
            DestroyImmediate(depthCaptureMaterial);
            #else
            Destroy(depthCaptureMaterial);
            #endif
        }
    }
}