using CAVAS.UB_MR;
using CAVAS.UB_MR.Modules.Sandbox;
using CAVAS.UB_MR.ROS2;
using ROS2;
using sensor_msgs.msg;
using UnityEngine;

namespace CAVAS.UB_MR.Modules
{
    public class SandboxModule : Module
    {
        [SerializeField] string imageTopic; 
        [SerializeField] CameraRenderer cameraRenderer;
        ROS2Node rosNode;
        ISubscription<Image> imageSubscription;
        Image latestImage;
        readonly object imageLock = new();

        protected override void Start()
        {
            base.Start();
            if (!ROS2_Bridge.ROS_CORE.Ok() || string.IsNullOrEmpty(imageTopic) || cameraRenderer == null)
                return;

            rosNode ??= ROS2_Bridge.ROS_CORE.CreateNode("sandbox_module");
            imageSubscription = rosNode.CreateSubscription<Image>(imageTopic, OnImageReceived);
        }

        void Update()
        {
            Image imageToRender = null;
            lock (imageLock)
            {
                if (latestImage != null)
                {
                    imageToRender = latestImage;
                    latestImage = null;
                }
            }

            if (imageToRender != null)
                cameraRenderer.Render(imageToRender);
        }

        void OnImageReceived(Image image)
        {
            if (image == null)
                return;

            lock (imageLock)
                latestImage = image;
        }
        
    }
}
