using ROS2;
using sensor_msgs.msg;
using UnityEngine;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection
{
    public class VirtualCameraOverlay
    {
        ROS2Node mNode;
        Camera targetCamera;
        IPublisher<CompressedImage> imagePublisher;
        RenderTexture renderTexture;
        Texture2D texture2D;
        string frameId = "camera_link"; // Default frame ID for the camera

        public VirtualCameraOverlay(string inTopicName, ROS2Node inNode, Camera inCamera, int inImageWidth = 640, int inImageHeight = 480)
        {
            this.targetCamera = inCamera;
            this.imagePublisher = inNode.CreatePublisher<CompressedImage>(inTopicName);
            // Create render texture and texture2D for image capture
            this.renderTexture = new RenderTexture(inImageWidth, inImageHeight, 24);
            this.texture2D = new Texture2D(inImageWidth, inImageHeight, TextureFormat.RGB24, false);
        }

        public void CaptureAndPublishImage(int inImageWidth = 640, int inImageHeight = 480)
        {
            if (imagePublisher == null || targetCamera == null)
                return;
                
            // Capture camera image (your existing code)
            RenderTexture currentRT = RenderTexture.active;
            targetCamera.targetTexture = renderTexture;
            targetCamera.Render();
            RenderTexture.active = renderTexture;
            texture2D.ReadPixels(new Rect(0, 0, inImageWidth, inImageHeight), 0, 0);
            texture2D.Apply();
            
            // Restore render texture
            targetCamera.targetTexture = null;
            RenderTexture.active = currentRT;
            
            // Convert to JPEG and publish 
            byte[] imageBytes = texture2D.EncodeToJPG(75);
            var compressedImage = new CompressedImage();
            builtin_interfaces.msg.Time time = new builtin_interfaces.msg.Time();
            time.Sec = (int)UnityEngine.Time.timeSinceLevelLoad; // Use Time.timeSinceLevelLoad for simulation time
            compressedImage.Header.Stamp = time;
            compressedImage.Header.Frame_id = frameId;
            compressedImage.Format = "jpeg";
            compressedImage.Data = imageBytes;
            imagePublisher.Publish(compressedImage);
        }

        public void CleanUp()
        {
            // Clean up resources
            if (renderTexture != null)
                renderTexture.Release();
            if (texture2D != null)
                GameObject.Destroy(texture2D);
        }
    }

}
