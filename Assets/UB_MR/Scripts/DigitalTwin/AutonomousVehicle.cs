using UnityEngine;
using ROS2;
using System.Collections;
using Unity.Cinemachine;
using CAVAS.UB_MR.DT.VirtualObjectDetection;

namespace CAVAS.UB_MR.DT
{
    public class AutonomousVehicle : DigitalTwin
    {
        [Header("Cameras")]
        [SerializeField] CinemachineCamera dashCam;
        [SerializeField] CinemachineCamera followCam;

        [Space]

        [Header("Object Detection Settings")]
        [SerializeField] bool enableBoundingBoxCapture = true; // Enable detection of virtual objects
        [SerializeField] bool enableImageCapture = true; // Enable image capture
        [SerializeField] bool enableLidarModifier = true; // Enable Lidar modifier for virtual objects
        [SerializeField] string worldTransformationTopicName = "/world_transform"; // Topic name for world transformation updates
        [SerializeField] string boundingBoxTopicName = "/virtual_obstacles"; // Topic name for publishing virtual object bounding boxes
        [SerializeField] string virtualCameraTopicName = "/virtual_camera/image_raw/compressed"; // Topic name for publishing virtual camera images
        [SerializeField] string lidarTopicName = "/lidar/scan"; // Topic name for Lidar scans

        [Space]

        [Header("Bounding Box Parameters")]
        [SerializeField] float detectionRadius = 30.0f; // Height of

        [Header("Image Capture Parameters")]
        [SerializeField] int imageWidth = 640;
        [SerializeField] int imageHeight = 480;
        [SerializeField] float publishRate = 1.0f; // 1 FPS


        protected Vector3 mWorldPosition = Vector3.zero;
        protected Vector3 mAngularVelocity = Vector3.zero;
        protected Vector3 mLinearVelocity = Vector3.zero;
        protected Quaternion mWorldRotation = Quaternion.identity;

        ROS2Node mNode;
        VirtualBoundingBoxDetector mVirtualBoundingBoxDetector;
        VirtualCameraOverlay mVirtualCameraOverlay;
        LidarModifier mLidarModifier;
        ISubscription<nav_msgs.msg.Odometry> mWorldTransformationSubscriber;

        

        public override void OnNetworkSpawn()
        {
            base.OnNetworkSpawn();
            if (IsOwner)
                ConnectToROS();
            StartCoroutine(PublishVirtualObjects());
        }

        public override void OnDestroy()
        {
            base.OnDestroy();
            if (IsOwner)
            {
                if (this.mVirtualCameraOverlay != null)
                    this.mVirtualCameraOverlay.CleanUp();

                if (this.mVirtualBoundingBoxDetector != null)
                    this.mVirtualBoundingBoxDetector.CleanUp();

                if (this.mLidarModifier != null)
                    this.mLidarModifier.CleanUp();
            }
            

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

        void Update()
        {
            if (enableLidarModifier && this.mLidarModifier != null)
                this.mLidarModifier.UpdateMovingObjects();
        }

        void ConnectToROS()
        {
            if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
            {
                string name = gameObject.name.Replace("(Clone)", "");
                name = name.Replace(" Variant", "");
                // This is sort of cheating but ROS2_Bridge is not immediately deleting nodes so this avoids a collision
                int randomSuffix = UnityEngine.Random.Range(0, 1000);
                this.mNode = ROS2_Bridge.ROS_CORE.CreateNode(name + "_Digital_Twin_" + randomSuffix.ToString());
                // World Transformation Subscriber
                this.mWorldTransformationSubscriber = this.mNode.CreateSubscription<nav_msgs.msg.Odometry>(worldTransformationTopicName, WorldTransformationUpdate);
                // Obstacle Bounding Box Publisher
                this.mVirtualBoundingBoxDetector = new VirtualBoundingBoxDetector(boundingBoxTopicName, this.mNode, this.transform);
                this.mVirtualCameraOverlay = new VirtualCameraOverlay(virtualCameraTopicName, this.mNode, FindFirstObjectByType<Camera>());
                this.mLidarModifier = new LidarModifier(this.transform, this.mNode, lidarTopicName);
            }
        }

        public IEnumerator PublishVirtualObjects()
        {
            while (IsOwner)
            {
                if (enableBoundingBoxCapture)
                    this.mVirtualBoundingBoxDetector.PublishNearbyVirtualObjects(detectionRadius);
                if (enableImageCapture)
                    this.mVirtualCameraOverlay.CaptureAndPublishImage(imageWidth, imageHeight);
                yield return new WaitForSeconds(1.0f / publishRate);
            }
        }

        public virtual Vector3 GetLinearVelocity()
        {
            return this.mLinearVelocity;
        }

        public virtual Vector3 GetAngularVelocity()
        {
            return this.mAngularVelocity;
        }

        public void EnableDashCam(bool inEnable)
        {
            if (IsOwner)
            {
                base.EnableCameras(false);
                if (dashCam != null)
                    dashCam.gameObject.SetActive(inEnable);
            }
        }

        public void EnableFollowCam(bool inEnable)
        {
            if (IsOwner)
            {
                base.EnableCameras(false);
                if (followCam != null)
                    followCam.gameObject.SetActive(inEnable);
            }
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
            this.mWorldPosition = new Vector3(
                -(float)msg.Pose.Pose.Position.Y,
                (float)msg.Pose.Pose.Position.Z,
                (float)msg.Pose.Pose.Position.X
            );

            // Build a C# quaternion from the raw ROS values
            var q_ros = new Quaternion(
                (float)msg.Pose.Pose.Orientation.X,
                (float)msg.Pose.Pose.Orientation.Y,
                (float)msg.Pose.Pose.Orientation.Z,
                (float)msg.Pose.Pose.Orientation.W
            );
            // Remap axes: FLU → URF
            Quaternion q_unity = new Quaternion(
                 -q_ros.y,    // Unity X = ROS Y
                 q_ros.z,    // Unity Y =  ROS Z
                 q_ros.x,    // Unity Z =  ROS X
                 -q_ros.w
            );
            q_unity.Normalize(); // Normalize the quaternion to ensure it's a valid rotation
            this.mWorldRotation = q_unity;

            this.mAngularVelocity = new Vector3(
                -(float)msg.Twist.Twist.Angular.Y,
                (float)msg.Twist.Twist.Angular.Z,
                (float)msg.Twist.Twist.Angular.X
            );
            this.mLinearVelocity = new Vector3(
                -(float)msg.Twist.Twist.Linear.Y,
                (float)msg.Twist.Twist.Linear.Z,
                (float)msg.Twist.Twist.Linear.X
            );
        }


    }

}
