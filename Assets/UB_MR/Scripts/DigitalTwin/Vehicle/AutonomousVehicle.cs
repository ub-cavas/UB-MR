using System;
using UnityEngine;
using ROS2;
using System.Collections;
using Unity.Cinemachine;
using System.Collections.Generic;
using CAVAS.UB_MR.DT.VirtualObjectDetection;
using CAVAS.UB_MR.DT.VirtualObjectDetection.Camera;
using CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar;
using CAVAS.UB_MR.ROS2;

namespace CAVAS.UB_MR.DT.Vehicle
{
    public class AutonomousVehicle : Agent
    {
        [Header("Cameras")]
        [SerializeField] CinemachineCamera dashCam;
        [SerializeField] CinemachineCamera followCam;

        [Space]

        [Header("Object Detection Settings")]
        [SerializeField] bool enableBoundingBoxCapture = true; // Enable detection of virtual objects
        [SerializeField] bool enableImageCapture = true; // Enable image capture
        [SerializeField] bool enableLidarModifier = true; // Enable Lidar modifier for virtual objects
        [Space]
        [SerializeField] string worldTransformationTopicName = "/world_transform"; // Topic name for world transformation updates
        [Space]
        [SerializeField] string boundingBoxTopicName = "/virtual_obstacles"; // Topic name for publishing virtual object bounding boxes
        [Space]
        [SerializeField] string virtualCameraImageTopicName = "/virtual_camera/image_raw"; // Topic name for publishing virtual camera images
        [SerializeField] string virtualCameraDepthTopicName = "/virtual_camera/depth"; // Topic name for publishing virtual camera depth images
        [Space]
        [SerializeField] string lidarTopicName = "/sensing/lidar/top/aw_points"; // Topic name for Lidar scans

        [Space]

        [Header("Bounding Box Parameters")]
        [SerializeField] float detectionRadius = 30.0f;

        [Header("Image Capture Parameters")]
        [SerializeField] int imageWidth = 640;
        [SerializeField] int imageHeight = 480;
        [SerializeField] float publishRate = 1.0f; // 1 FPS


        [Header("LiDAR Capture Parameters")] 
        [SerializeField] int raysPerScan = 60_000;
        [SerializeField] Transform mLidar;
        [SerializeField] List<SDFTexture> mSdfs;
        [SerializeField] ComputeShader mLidarComputeShader;
        [SerializeField] ReliabilityPolicy reliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT;
        [SerializeField] HistoryPolicy historyPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST;
        [SerializeField] int historyDepth = 2;
        [SerializeField] DurabilityPolicy durabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE;
        // More LiDAR variables (Don't touch these unless you know what you're doing)
        float maxRaytraceDistance = 100.0f;
        float hitThreshold = 0.0001f;
        int maxIterations = 64;
        
        protected Vector3 mWorldPosition = Vector3.zero;
        protected Quaternion mWorldRotation = Quaternion.identity;

        ROS2Node mNode;
        VirtualBoundingBoxDetector mVirtualBoundingBoxDetector;
        VirtualCameraOverlay mVirtualCameraOverlay;
        LidarModifier mLidarModifier;
        ISubscription<nav_msgs.msg.Odometry> mWorldTransformationSubscriber;
        
        protected override void OnEnable()
        {
            base.OnEnable();

            ConnectToROS();
            this.mLidarModifier.UpdateSDFRaytraceParameters(maxRaytraceDistance, hitThreshold, maxIterations);
            StartCoroutine(PublishVirtualObjects());
        }

        protected override void OnDisable()
        {
            base.OnDisable();

            if (this.mVirtualCameraOverlay != null)
                    this.mVirtualCameraOverlay.CleanUp();

            if (this.mVirtualBoundingBoxDetector != null)
                this.mVirtualBoundingBoxDetector.CleanUp();

            if (this.mLidarModifier != null)
                this.mLidarModifier.CleanUp();
            

            if (this.mNode != null && this.mWorldTransformationSubscriber != null)
            {
                Debug.Log("Destroying Node: " + this.mNode.name);
                if (Ros2cs.Ok())
                {
                    // Unsubscribe from the world transformation topic
                    this.mNode.RemoveSubscription<nav_msgs.msg.Odometry>(this.mWorldTransformationSubscriber);
                    this.mWorldTransformationSubscriber = null;
                }
                ROS2_Bridge.ROS_CORE.RemoveNode(this.mNode);
                this.mNode = null;
            }
        }

        protected virtual void Update()
        {
            if (this.mLidarModifier != null && enableLidarModifier)
            {
                if (this.mLidarModifier.TryModify(this.mLidar.transform))
                    this.mLidarModifier.PublishPCD();
            }
        }

        void ConnectToROS()
        {
            if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
            {
                string name = gameObject.name.Replace("(Clone)", "");
                name = name.Replace(" Variant", "");
                // This is sort of cheating but ROS2_Bridge is not immediately deleting nodes so this avoids a collision (~99% of the time)
                int randomSuffix = UnityEngine.Random.Range(0, 1000);
                this.mNode = ROS2_Bridge.ROS_CORE.CreateNode(name + "_Digital_Twin_" + randomSuffix.ToString());
                // World Transformation Subscriber
                this.mWorldTransformationSubscriber = this.mNode.CreateSubscription<nav_msgs.msg.Odometry>(worldTransformationTopicName, WorldTransformationUpdate);
                // Bounding Box (Ground Truth)
                this.mVirtualBoundingBoxDetector = new VirtualBoundingBoxDetector(boundingBoxTopicName, this.mNode, this.transform);
                // Camera Modifier
                this.mVirtualCameraOverlay = new VirtualCameraOverlay(virtualCameraImageTopicName, virtualCameraDepthTopicName, this.mNode, FindFirstObjectByType<Camera>(), imageWidth, imageHeight);
                // LiDAR Modifier
                QualityOfServiceProfile qosProfile = new QualityOfServiceProfile();
                qosProfile.SetReliability(reliabilityPolicy);
                qosProfile.SetHistory(historyPolicy, historyDepth);
                qosProfile.SetDurability(durabilityPolicy);
                //TODO: dynamic lists of SDFS... currently just supports 1 sdf
                this.mSdfs[0] = FindFirstObjectByType<SDFTexture>();
                this.mLidarModifier = new LidarModifier(this, lidarTopicName, raysPerScan, this.mLidarComputeShader, this.mNode, qosProfile, this.mSdfs[0]);
            }
        }

        public IEnumerator PublishVirtualObjects()
        {
            while (true)
            {
                yield return new WaitForEndOfFrame();
                if (enableBoundingBoxCapture)
                    this.mVirtualBoundingBoxDetector.PublishNearbyVirtualObjects(detectionRadius);
                if (enableImageCapture)
                {
                    this.mVirtualCameraOverlay.UpdateCameraResolution(imageWidth, imageHeight);
                    this.mVirtualCameraOverlay.CaptureAndPublishImage();
                }
                yield return new WaitForSeconds(1.0f / publishRate);
            }
        }

        public void EnableDashCam(bool inEnable)
        {
            DisableAllSpectatorCameras();
            if (dashCam is not null)
                EnableSpectatorCamera(dashCam, true);
        }

        public void EnableFollowCam(bool inEnable)
        {
            DisableAllSpectatorCameras();
            if (followCam is not null)
                EnableSpectatorCamera(followCam, true);
        }

        public void SetLayerCulling(Camera camera, string layerName, bool shouldRender)
        {
            int layerIndex = LayerMask.NameToLayer(layerName);
            if (layerIndex == -1)
            {
                Debug.LogWarning($"Layer '{layerName}' does not exist.");
                return;
            }

            int layerMask = 1 << layerIndex;
            if (shouldRender)
            {
                camera.cullingMask |= layerMask;
                camera.clearFlags = CameraClearFlags.Skybox;
            }
            else
            {
                camera.cullingMask &= ~layerMask;
                camera.clearFlags = CameraClearFlags.SolidColor;
                camera.backgroundColor = Color.black;
            }
        }

        void WorldTransformationUpdate(nav_msgs.msg.Odometry msg)
        {
            this.mWorldPosition = Ros2Utility.Ros2ToUnityPosition(msg.Pose.Pose.Position);
            this.mWorldRotation = Ros2Utility.Ros2ToUnityRotation(msg.Pose.Pose.Orientation);
        }


    }

}
