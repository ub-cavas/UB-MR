using System;
using UnityEngine;
using UnityEngine.UI;
using sensor_msgs.msg;

namespace CAVAS.UB_MR.Modules.Sandbox
{
    public class CameraRenderer : MonoBehaviour
    {
        [SerializeField] RawImage rawImage;
        Texture2D rosTexture;
        byte[] conversionBuffer;

        void Start()
        {
            // Flip RawImage to cleanly display a ROSImage 
            rawImage.uvRect = new Rect(0, 1, 1, -1);
        }

        public void Render(RenderTexture renderTexture)
        {
            if (rawImage == null || renderTexture == null)
                return; 

            rawImage.texture = renderTexture;
        }

        /// <summary> 
        /// Render a ROS2 sensor_msgs/Image onto the RawImage.
        /// Supports rgba8, bgra8, rgb8, bgr8, and mono8 encodings.
        /// </summary>
        public void Render(sensor_msgs.msg.Image rosImage)
        {
            if (rawImage == null || rosImage == null || rosImage.Data == null)
                return;

            int width = (int)rosImage.Width;
            int height = (int)rosImage.Height;
            if (width <= 0 || height <= 0)
                return;

            EnsureRosTexture(width, height);

            if (TryCopyRosImageToTexture(rosImage, width, height))
                rawImage.texture = rosTexture;
        }

        void EnsureRosTexture(int width, int height)
        {
            if (rosTexture != null && rosTexture.width == width && rosTexture.height == height)
                return;

            rosTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);
            rosTexture.wrapMode = TextureWrapMode.Clamp;
            rosTexture.filterMode = FilterMode.Bilinear;
        }

        bool TryCopyRosImageToTexture(sensor_msgs.msg.Image  rosImage, int width, int height)
        {
            string encoding = rosImage.Encoding ?? string.Empty;
            int pixelCount = width * height;
            byte[] data = rosImage.Data;

            bool IsEncoding(string value) => string.Equals(encoding, value, StringComparison.OrdinalIgnoreCase);

            if (IsEncoding("rgba8"))
            {
                if (data.Length < pixelCount * 4)
                    return false;

                rosTexture.LoadRawTextureData(data);
                rosTexture.Apply(false);
                return true;
            }

            if (IsEncoding("bgra8"))
            {
                int expectedLength = pixelCount * 4;
                if (data.Length < expectedLength)
                    return false;

                EnsureBufferSize(expectedLength);
                for (int i = 0; i < expectedLength; i += 4)
                {
                    conversionBuffer[i] = data[i + 2];     // R
                    conversionBuffer[i + 1] = data[i + 1]; // G
                    conversionBuffer[i + 2] = data[i];     // B
                    conversionBuffer[i + 3] = data[i + 3]; // A
                }

                rosTexture.LoadRawTextureData(conversionBuffer);
                rosTexture.Apply(false);
                return true;
            }

            if (IsEncoding("rgb8") || IsEncoding("bgr8") || IsEncoding("mono8"))
            {
                int sourceChannels = IsEncoding("mono8") ? 1 : 3;
                int expectedLength = pixelCount * sourceChannels;
                if (data.Length < expectedLength)
                    return false;

                EnsureBufferSize(pixelCount * 4);

                bool isBgr = IsEncoding("bgr8");
                for (int src = 0, dst = 0; src < expectedLength; src += sourceChannels, dst += 4)
                {
                    byte r, g, b;
                    if (sourceChannels == 1)
                    {
                        r = g = b = data[src];
                    }
                    else if (isBgr)
                    {
                        b = data[src];
                        g = data[src + 1];
                        r = data[src + 2];
                    }
                    else
                    {
                        r = data[src];
                        g = data[src + 1];
                        b = data[src + 2];
                    }

                    conversionBuffer[dst] = r;
                    conversionBuffer[dst + 1] = g;
                    conversionBuffer[dst + 2] = b;
                    conversionBuffer[dst + 3] = 255;
                }

                rosTexture.LoadRawTextureData(conversionBuffer);
                rosTexture.Apply(false);
                return true;
            }

            Debug.LogWarning($"Unsupported ROS2 image encoding '{encoding}'.");
            return false;
        }

        void EnsureBufferSize(int length)
        {
            if (conversionBuffer == null || conversionBuffer.Length < length)
                conversionBuffer = new byte[length];
        }
    }
}
