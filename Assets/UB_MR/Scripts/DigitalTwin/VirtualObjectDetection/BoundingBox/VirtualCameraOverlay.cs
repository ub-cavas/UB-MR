using ROS2;
using sensor_msgs.msg;
using UnityEngine;
using System;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection
{
    public class VirtualCameraOverlay
    {
        public static int IMAGE_WIDTH = 640;
        public static int IMAGE_HEIGHT = 480;
        ROS2Node mNode;
        Camera targetCamera;
        // IMAGE
        IPublisher<CompressedImage> compressedImagePublisher;
        IPublisher<Image> imagePublisher;
        RenderTexture imageRenderTexture;
        Texture2D imageTexture2D;
        byte[] imageByteData;
        byte[] depthByteData;

        // DEPTH
        IPublisher<CompressedImage> compressedDepthImagePublisher;
        IPublisher<Image> depthPublisher;
        Texture2D depthTexture2D;
        
        string frameId = "camera_link"; // Default frame ID for the camera

        public VirtualCameraOverlay(DigitalTwin inDT, string inImageTopic, string inDepthTopic, ROS2Node inNode, Camera inCamera, int inImageWidth = 640, int inImageHeight = 480)
        {
            UpdateCameraResolution(inDT, inImageWidth, inImageHeight);
            this.targetCamera = inCamera;
            // IMAGE
            this.compressedImagePublisher = inNode.CreatePublisher<CompressedImage>(inImageTopic + "/compressed");
            this.imagePublisher = inNode.CreatePublisher<Image>(inImageTopic);
            this.imageRenderTexture = new RenderTexture(IMAGE_WIDTH, IMAGE_HEIGHT, 24);
            this.imageTexture2D = new Texture2D(IMAGE_WIDTH, IMAGE_HEIGHT, TextureFormat.RGB24, false);
            // DEPTH
            this.compressedDepthImagePublisher = inNode.CreatePublisher<CompressedImage>(inDepthTopic + "/compressed");
            this.depthPublisher = inNode.CreatePublisher<Image>(inDepthTopic);
            this.depthTexture2D = new Texture2D(IMAGE_WIDTH, IMAGE_HEIGHT, TextureFormat.RFloat, false);
        }

        public void UpdateCameraResolution(DigitalTwin inDT, int inImageWidth, int inImageHeight)
        {
            if (inDT.IsOwner)
            {
                IMAGE_WIDTH = inImageWidth;
                IMAGE_HEIGHT = inImageHeight;
            }
        }


        // TODO: Implement compressed depth image capture
        public void CaptureAndPublishCompressedImage(int inCompressionQuality = 75)
        {
            if (compressedImagePublisher == null || depthPublisher == null || targetCamera == null)
                return;

            builtin_interfaces.msg.Time time = GetTimestamp();
            // Capture Camera Image
            CompressedImage compressedImage = CaptureCompressedImage(inCompressionQuality);
            compressedImage.Header.Stamp = time;
            compressedImage.Header.Frame_id = frameId;
            // Capture Depth Image
            CompressedImage compressedDepthImage = CaptureCompressedDepthImage();
            compressedDepthImage.Header.Stamp = time;
            compressedDepthImage.Header.Frame_id = frameId;
            // Publish both images
            compressedImagePublisher.Publish(compressedImage);
            //depthImagePublisher.Publish(compressedDepthImage);
        }

        public void CaptureAndPublishImage()
        {
            builtin_interfaces.msg.Time time = GetTimestamp();
            Image image = null;
            Image depthImage = null;
            if (imagePublisher != null && targetCamera != null)
            {
                // Capture Camera Image
                image = CaptureImage();
                image.Header.Stamp = time;
                image.Header.Frame_id = frameId;
            }
            if (depthPublisher != null && targetCamera != null)
            {
                // Capture Depth Image
                depthImage = CaptureDepthImage();
                depthImage.Header.Stamp = time;
                depthImage.Header.Frame_id = frameId;
            }

            // Publish both images
            if (imagePublisher != null && image != null)
            {
                imagePublisher.Publish(image);
            }
            if (depthPublisher != null && depthImage != null)
            {
                depthPublisher.Publish(depthImage);
            }
               
        }

        builtin_interfaces.msg.Time GetTimestamp()
        {
            builtin_interfaces.msg.Time time = new builtin_interfaces.msg.Time();
            time.Sec = (int)UnityEngine.Time.timeSinceLevelLoad;
            return time;
        }

        CompressedImage CaptureCompressedImage(int inCompressionQuality = 75)
        {
            if (compressedImagePublisher == null || targetCamera == null)
                return null;
            RenderTexture currentRT = RenderTexture.active;
            targetCamera.targetTexture = imageRenderTexture;
            targetCamera.Render();
            RenderTexture.active = imageRenderTexture;
            imageTexture2D.ReadPixels(new Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT), 0, 0);
            imageTexture2D.Apply();
            // Restore render texture
            targetCamera.targetTexture = null;
            RenderTexture.active = currentRT;
            // Convert to JPEG and publish 
            byte[] imageBytes = imageTexture2D.EncodeToJPG(inCompressionQuality);
            var compressedImage = new CompressedImage();
            compressedImage.Format = "jpeg";
            compressedImage.Data = imageBytes;
            return compressedImage;
        }

        CompressedImage CaptureCompressedDepthImage()
        {
            Debug.LogError("Compressed depth image capture not implemented yet. Currently only supports raw depth images.");
            return null;
        }

        Image CaptureImage()
        {
            if (imagePublisher == null || targetCamera == null)
                return null;

            RenderTexture currentRT = RenderTexture.active;
            targetCamera.targetTexture = imageRenderTexture;
            targetCamera.Render();
            RenderTexture.active = imageRenderTexture;
            
            imageTexture2D.ReadPixels(new Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT), 0, 0);
            imageTexture2D.Apply();
            
            // Restore render texture
            targetCamera.targetTexture = null;
            RenderTexture.active = currentRT;
            
            
            int dataSize = IMAGE_WIDTH * IMAGE_HEIGHT * 3;
            if (imageByteData == null || imageByteData.Length != dataSize)
            {
                imageByteData = new byte[dataSize];
            }
            var rawData = imageTexture2D.GetRawTextureData();
            
            // Flip the image vertically while copying
            int bytesPerRow = IMAGE_WIDTH * 3;
            for (int row = 0; row < IMAGE_HEIGHT; row++)
            {
                int sourceRow = IMAGE_HEIGHT - 1 - row;  // Read from bottom to top
                int sourceIndex = sourceRow * bytesPerRow;
                int destIndex = row * bytesPerRow;
                System.Buffer.BlockCopy(rawData, sourceIndex, imageByteData, destIndex, bytesPerRow);
            }
            
            var image = new Image();
            image.Height = (uint)IMAGE_HEIGHT;
            image.Width = (uint)IMAGE_WIDTH;
            image.Encoding = "rgb8";  // RGB24 in Unity maps to "rgb8" in ROS2
            image.Is_bigendian = 0; // false
            image.Step = (uint)(IMAGE_WIDTH * 3);  // 3 bytes per pixel for RGB
            image.Data = imageByteData;
            return image;
        }

        Image CaptureDepthImage()
        {
            if (depthPublisher == null || targetCamera == null)
                return null;

            RenderTexture currentRT = RenderTexture.active;
            RenderTexture.active = DepthCaptureRenderFeature.DepthCapturePass.GetDepthRenderTexture(); // This probably needs to get fixed
            depthTexture2D.ReadPixels(new Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT), 0, 0);
            depthTexture2D.Apply();
            RenderTexture.active = currentRT;

           int dataSize = IMAGE_WIDTH * IMAGE_HEIGHT * 4;
            if (depthByteData == null || depthByteData.Length != dataSize)
            {
                depthByteData = new byte[dataSize];
            }
            byte[] rawData = depthTexture2D.GetRawTextureData();
            int bytesPerRow = IMAGE_WIDTH * 4; // 4 bytes per float pixel
            for (int row = 0; row < IMAGE_HEIGHT; row++)
            {
                int sourceRow = IMAGE_HEIGHT - 1 - row;  // Read from bottom to top
                int sourceIndex = sourceRow * bytesPerRow;
                int destIndex = row * bytesPerRow;
                System.Buffer.BlockCopy(rawData, sourceIndex, depthByteData, destIndex, bytesPerRow);
            }
            
            var image = new Image();
            image.Height = (uint)IMAGE_HEIGHT;
            image.Width = (uint)IMAGE_WIDTH;
            image.Encoding = "32FC1";  // Single channel float for depth
            image.Is_bigendian = BitConverter.IsLittleEndian ? (byte)0 : (byte)1;
            image.Step = (uint)(IMAGE_WIDTH * 4);  // 4 bytes per pixel for float
            image.Data = depthByteData;
            return image;
        }

        public void CleanUp()
        {
            // Clean up resources
            if (imageRenderTexture != null)
                imageRenderTexture.Release();
            if (imageTexture2D != null)
                GameObject.Destroy(imageTexture2D);
        }
    }

}
