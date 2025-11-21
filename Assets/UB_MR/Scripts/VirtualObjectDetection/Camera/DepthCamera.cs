using System.Collections;
using UnityEngine;
using UnityEngine.Rendering;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection
{
    public class DepthCamera : MonoBehaviour
    {
        [SerializeField] UnityEngine.Camera renderCamera;
        public Shader depthShader;
        Material depthMaterial;
        RenderTexture depthRT;
        CommandBuffer commandBuffer;

        void Start()
        {
            SetupDepthCamera(renderCamera);
        }

        //TODO Call this when spawning this Component.
        public void SetupDepthCamera(UnityEngine.Camera inCamera)
        {
            //renderCamera = inCamera;
            renderCamera.depthTextureMode |= DepthTextureMode.Depth;

            depthRT = new RenderTexture(renderCamera.pixelWidth, renderCamera.pixelHeight, 24, RenderTextureFormat.RFloat);
            depthRT.name = $"{renderCamera.name}_DepthRT";
            depthRT.Create();

            //depthShader = Resources.Load<Shader>("Scripts/DepthCapture");
            depthMaterial = new Material(depthShader);

            commandBuffer = new CommandBuffer();
            commandBuffer.name = commandBuffer.name + " Depth Capture Pass";
            commandBuffer.Blit(null, depthRT, depthMaterial);

            renderCamera.AddCommandBuffer(CameraEvent.AfterForwardOpaque, commandBuffer);
        }

        void OnDestroy()
        {
            if (commandBuffer != null && renderCamera != null)
                renderCamera.RemoveCommandBuffer(CameraEvent.AfterForwardOpaque, commandBuffer);

            if (depthRT != null)
                depthRT.Release();

            if (depthMaterial != null)
                Destroy(depthMaterial);
        }

        public float[,] ReadDepthNow()
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
                {
                    depthMeters[y, x] = raw[rowOffset + x];
                }
            }

            RenderTexture.active = prev;
            Destroy(tex);
            return depthMeters;
        }

        void Update()
        {
            if (Input.GetKeyDown(KeyCode.D))
            {
                StartCoroutine(CaptureDepthAtEndOfFrame());
            }
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
