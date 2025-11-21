using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class DepthCaptureRenderFeature : ScriptableRendererFeature
{
    private DepthPass depthPass; 
    private Material depthCaptureMaterial;
    
    public override void Create()
    {
        depthPass = new DepthPass();
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
