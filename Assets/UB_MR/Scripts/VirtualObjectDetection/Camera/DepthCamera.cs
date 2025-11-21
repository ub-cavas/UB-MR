using UnityEngine;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection
{
    public class DepthCamera
    {
        int width;
        int height;
        UnityEngine.Camera camera;
        RenderTexture depthTexture;

        public DepthCamera(UnityEngine.Camera inCamera, int inWidth, int inHeight)
        {
            camera = inCamera;
            width = inWidth;
            height = inHeight;
            
            camera.depthTextureMode |= DepthTextureMode.Depth;

            depthTexture = new RenderTexture(width, height, 24, RenderTextureFormat.RFloat);
            depthTexture.enableRandomWrite = false;
            depthTexture.Create();

            camera.targetTexture = depthTexture;
        }


    }
}
