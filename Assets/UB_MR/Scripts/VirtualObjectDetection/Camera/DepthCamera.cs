using System.Collections;
using UnityEngine;

namespace CAVAS.UB_MR.DT.Sensors.Camera
{
    public class DepthCamera
    {
        UnityEngine.Camera renderCamera;
        Shader depthShader;
        Material depthMaterial;
        RenderTexture depthRT;

        public DepthCamera(UnityEngine.Camera inCamera)
        {
            renderCamera = inCamera;
            renderCamera.depthTextureMode |= DepthTextureMode.Depth;

            depthRT = new RenderTexture(renderCamera.pixelWidth, renderCamera.pixelHeight, 24, RenderTextureFormat.RFloat);
            depthRT.name = $"{renderCamera.name}_DepthRT";
            depthRT.Create();

            depthShader = Resources.Load<Shader>("Scripts/DepthCapture");
            depthMaterial = new Material(depthShader);

            DepthCaptureRenderFeature.Register(renderCamera, depthRT, depthMaterial);
        }

        public void CleanUp()
        {
            DepthCaptureRenderFeature.Unregister(renderCamera);

            if (depthRT != null)
                depthRT.Release();

            if (depthMaterial != null)
                GameObject.Destroy(depthMaterial);
        }

        float[,] ReadDepthNow()
        {
            // Make the depthRT active so we can read from it
            RenderTexture prev = RenderTexture.active;
            RenderTexture.active = depthRT;

            // Use RFloat to match the RenderTexture format
            Texture2D tex = new Texture2D(renderCamera.pixelWidth, renderCamera.pixelHeight, TextureFormat.RFloat, false, true);
            tex.ReadPixels(new Rect(0, 0, renderCamera.pixelWidth, renderCamera.pixelHeight), 0, 0);
            tex.Apply();

            // Map 1D raw buffer into 2D [y, x] array
            var raw = tex.GetRawTextureData<float>();
            float[,] depthMeters = new float[renderCamera.pixelHeight, renderCamera.pixelWidth];
            for (int y = 0; y < renderCamera.pixelHeight; y++)
            {
                int rowOffset = y * renderCamera.pixelWidth;
                for (int x = 0; x < renderCamera.pixelWidth; x++)
                    depthMeters[y, x] = raw[rowOffset + x];
            }

            RenderTexture.active = prev;
            GameObject.Destroy(tex);
            return depthMeters;
        }

        IEnumerator CaptureDepthAtEndOfFrame()
        {
            // Wait until all cameras & command buffers have rendered this frame
            yield return new WaitForEndOfFrame();

            float[,] depth = ReadDepthNow();
            if (depth != null)
            {
                int h = depth.GetLength(0);
                int w = depth.GetLength(1);
                float centerDepth = depth[h / 2, w / 2];
                Debug.Log($"Depth at center: {centerDepth} meters");
            }
        }
    }
}
