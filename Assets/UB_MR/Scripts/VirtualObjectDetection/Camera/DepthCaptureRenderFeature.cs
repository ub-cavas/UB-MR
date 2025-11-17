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
        static RenderTexture depthRenderTexture;

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

        public static RenderTexture GetDepthRenderTexture()
        {
            return depthRenderTexture;
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