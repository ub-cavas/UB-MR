using System;
using ROS2;
using sensor_msgs.msg;
using UnityEngine;

namespace CAVAS.UB_MR.DT.Sensors.Camera
{
    public class CameraModifier : SensorModifier
    {
        Agent owner;
        UnityEngine.Camera rgbCamera;
        DepthCamera depthCamera;

        ISubscription<sensor_msgs.msg.Image> rgbSubscriber;
        ISubscription<sensor_msgs.msg.Image> depthSubscriber;

        IPublisher<sensor_msgs.msg.Image> mixedRealityFramePublisher;

        public CameraModifier(Agent inOwner, string inTopicName, UnityEngine.Camera inCamera)
        {
            owner = inOwner;
            // Virtual Camera
            rgbCamera = inCamera;
            depthCamera = new DepthCamera(inCamera);
            // Physical Camera
            rgbSubscriber = owner.ROSNode().CreateSubscription<sensor_msgs.msg.Image>(inTopicName, OnRGB_Receive);
            mixedRealityFramePublisher = owner.ROSNode().CreatePublisher<sensor_msgs.msg.Image>("mixed_reality_frame"); 
        }

        void OnRGB_Receive(sensor_msgs.msg.Image inImage)
        {
            Debug.Log("RECEIVED RGB IMAGE!");
            // TODO: Match the incoming physical RGB frame with the corresponding physical depth frame (same timestep)
        }

        void OnDepth_Receive(sensor_msgs.msg.Image inImage)
        {
            Debug.Log("RECEIVED DEPTH IMAGE!");
            // TODO: Match the incoming physical depth frame with the corresponding physical RGB frame (same timestep)
        }

        public override void Publish()
        {
            sensor_msgs.msg.Image mrFrame = ComputeMixedRealityFrame();
            // TODO: Publish mrFrame
        }

        sensor_msgs.msg.Image ComputeMixedRealityFrame()
        {
            //TODO: Combine virtual camera RGB frame with matching physical RGB frame (from ROS) by 
            // updating each pixel of the physical frame to selectively render the pixel which is closer to the camera (for occlusions)
            return null;
        }

        public override void CleanUp()
        {
            // Virtual Camera
            depthCamera.CleanUp();

            // Physical Camera
            if (Ros2cs.Ok())
            {
                owner.ROSNode().RemoveSubscription<PointCloud2>(rgbSubscriber);
                rgbSubscriber = null; 
            }
        }

        void Update()
        {
            if (Input.GetKeyDown(KeyCode.D))
            {
                //StartCoroutine(CaptureDepthAtEndOfFrame());
            }
        }
    }
}
