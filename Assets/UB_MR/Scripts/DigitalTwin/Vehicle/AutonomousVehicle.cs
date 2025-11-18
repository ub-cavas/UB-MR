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
        string worldTransformationTopicName = "/world_transform";
        string virtualCameraImageTopicName = "/virtual_camera/image_raw";
        string virtualCameraDepthTopicName = "/virtual_camera/depth";
        string lidarTopicName = "/sensing/lidar/top/aw_points";

        [Header("Image Capture Parameters")]
        [SerializeField] int imageWidth = 640;
        [SerializeField] int imageHeight = 480;

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
        
        VirtualCameraOverlay mVirtualCameraOverlay;
        LidarModifier mLidarModifier;
        ISubscription<nav_msgs.msg.Odometry> odometrySubscriber;
        
        public override void Setup(Config.Agent inAgent)
        {
            base.Setup(inAgent);

            // Odometry
            if (ROS2_Bridge.ROS_CORE.Ok())
                this.odometrySubscriber = ROSNode().CreateSubscription<nav_msgs.msg.Odometry>(worldTransformationTopicName, OdometryUpdate);
            
            this.mLidarModifier.UpdateSDFRaytraceParameters(maxRaytraceDistance, hitThreshold, maxIterations);
        }

        public override void Teardown()
        {
            if (this.mVirtualCameraOverlay != null)
                    this.mVirtualCameraOverlay.CleanUp();

            if (this.mLidarModifier != null)
                this.mLidarModifier.CleanUp();
            
            // Odometry
            if (ROSNode() is not null && odometrySubscriber is not null)
            {
                ROSNode().RemoveSubscription<nav_msgs.msg.Odometry>(this.odometrySubscriber);
                this.odometrySubscriber = null;
            }

            base.Teardown();
        }

        protected virtual void Update()
        {
            if (this.mLidarModifier is not null)
            {
                if (this.mLidarModifier.TryModify(this.mLidar.transform))
                    this.mLidarModifier.PublishPCD();
            }
        }

        protected override void ConnectToROS()
        {
            base.ConnectToROS();
            // Camera Modifier
            this.mVirtualCameraOverlay = new VirtualCameraOverlay(virtualCameraImageTopicName, virtualCameraDepthTopicName, ROSNode(), FindFirstObjectByType<Camera>(), imageWidth, imageHeight);
            // LiDAR Modifier
            QualityOfServiceProfile qosProfile = new QualityOfServiceProfile();
            qosProfile.SetReliability(reliabilityPolicy);
            qosProfile.SetHistory(historyPolicy, historyDepth);
            qosProfile.SetDurability(durabilityPolicy);
            //TODO: dynamic lists of SDFS... currently just supports 1 sdf
            this.mSdfs[0] = FindFirstObjectByType<SDFTexture>();
            this.mLidarModifier = new LidarModifier(this, lidarTopicName, raysPerScan, this.mLidarComputeShader, ROSNode(), qosProfile, this.mSdfs[0]);
        }

    }

}
