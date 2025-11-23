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
        Texture2D depthReadTex;
        float[] depthBuffer;
        int currentWidth;
        int currentHeight;

        public DepthCamera(UnityEngine.Camera inCamera)
        {
            renderCamera = inCamera;
            renderCamera.depthTextureMode |= DepthTextureMode.Depth;

            currentWidth = renderCamera.pixelWidth;
            currentHeight = renderCamera.pixelHeight;

            depthShader = Resources.Load<Shader>("Scripts/DepthCapture");
            depthMaterial = new Material(depthShader);

            CreateDepthTargets(currentWidth, currentHeight);
        }

        public void CleanUp()
        {
            DepthCaptureRenderFeature.Unregister(renderCamera);

            if (depthRT != null)
                depthRT.Release();

            if (depthReadTex != null)
                GameObject.Destroy(depthReadTex);

            if (depthMaterial != null)
                GameObject.Destroy(depthMaterial);
        }

        void CreateDepthTargets(int width, int height)
        {
            DepthCaptureRenderFeature.Unregister(renderCamera);

            if (depthRT != null)
                depthRT.Release();

            depthRT = new RenderTexture(width, height, 24, RenderTextureFormat.RFloat);
            depthRT.name = $"{renderCamera.name}_DepthRT";
            depthRT.Create();

            if (depthReadTex != null)
                GameObject.Destroy(depthReadTex);

            depthReadTex = new Texture2D(width, height, TextureFormat.RFloat, false, true);
            depthBuffer = new float[width * height];

            DepthCaptureRenderFeature.Register(renderCamera, depthRT, depthMaterial);
        }

        public void EnsureResolution(int width, int height)
        {
            if (width == currentWidth && height == currentHeight && depthRT != null)
                return;

            currentWidth = width;
            currentHeight = height;
            CreateDepthTargets(width, height);
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

        public float[] CaptureDepth(int width, int height)
        {
            EnsureResolution(width, height);

            RenderTexture prev = RenderTexture.active;
            RenderTexture.active = depthRT;

            depthReadTex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            depthReadTex.Apply(false, false);

            var raw = depthReadTex.GetRawTextureData<float>();
            if (depthBuffer == null || depthBuffer.Length != raw.Length)
                depthBuffer = new float[raw.Length];
            raw.CopyTo(depthBuffer);

            RenderTexture.active = prev;
            return depthBuffer;
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
