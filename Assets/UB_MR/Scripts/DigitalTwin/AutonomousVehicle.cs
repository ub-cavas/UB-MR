using UnityEngine;
using ROS2;
using System.Collections.Generic;
using vision_msgs.msg;

namespace CAVAS.UB_MR.DT
{

    public class AutonomousVehicle : DigitalTwin
    {
        [Header("Object Detection Settings")]
        [SerializeField] bool enableVirtualObjectDetection = true; // Enable detection of virtual objects
        [SerializeField] float virtualObjectDetectionRadius = 30f; // Radius in which virtual objects are detected
        [SerializeField] string virtualObjectsTopicName = "/virtual_obstacles"; // Topic name for publishing virtual objects
        [Space]

        ROS2Node mNode;
        VirtualObjectDetector mVirtualObjectDetector;
        ISubscription<nav_msgs.msg.Odometry> mWorldTransformationSubscriber;
        IPublisher<BoundingBox3DArray> mObstacleBoundingBoxPublisher;

        protected Vector3 mWorldPosition = Vector3.zero;
        protected Vector3 mAngularVelocity = Vector3.zero;
        protected Vector3 mLinearVelocity = Vector3.zero;
        protected Quaternion mWorldRotation = Quaternion.identity;

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
                this.mWorldTransformationSubscriber = this.mNode.CreateSubscription<nav_msgs.msg.Odometry>("/world_transform", WorldTransformationUpdate);
                // Obstacle Bounding Box Publisher
                this.mVirtualObjectDetector = new VirtualObjectDetector(this.transform);
                this.mObstacleBoundingBoxPublisher = this.mNode.CreatePublisher<BoundingBox3DArray>(virtualObjectsTopicName);
            }
        }

        public override void OnNetworkSpawn()
        {
            base.OnNetworkSpawn();
            if (IsOwner)
            {
                ConnectToROS();
            }
        }

        public override void OnDestroy()
        {
            base.OnDestroy();

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
            if (IsOwner && enableVirtualObjectDetection)
            {
                PublishNearbyVirtualObjects();
            }
        }

        void PublishNearbyVirtualObjects()
        {
            // TODO: Bounding Boxes position and orientation should be relative to the Ego-Vehicle, not the world
            vision_msgs.msg.BoundingBox3DArray msg = new vision_msgs.msg.BoundingBox3DArray();
            // Header
            msg.Header = new std_msgs.msg.Header();
            msg.Header.Frame_id = "world";
            builtin_interfaces.msg.Time time = new builtin_interfaces.msg.Time();
            time.Sec = (int)UnityEngine.Time.timeSinceLevelLoad; // Use Time.timeSinceLevelLoad for simulation time
            msg.Header.Stamp = time;

            List<VirtualObject> virtualObjects = this.mVirtualObjectDetector.GetNearbyObstacles(virtualObjectDetectionRadius);
            var boxes = new BoundingBox3D[virtualObjects.Count];
            for (int i = 0; i < boxes.Length; i++)
            {
                VirtualObject virtualObject = virtualObjects[i];
                Bounds bounds = virtualObject.GetBoundingBox();
                BoundingBox3D bbox = new BoundingBox3D();
                bbox.Center = new geometry_msgs.msg.Pose();
                // Position
                bbox.Center.Position = new geometry_msgs.msg.Point();
                bbox.Center.Position.X = bounds.center.x;
                bbox.Center.Position.Y = bounds.center.y;
                bbox.Center.Position.Z = bounds.center.z;
                // Orientation
                bbox.Center.Orientation = new geometry_msgs.msg.Quaternion();
                bbox.Center.Orientation.X = virtualObject.transform.rotation.x;
                bbox.Center.Orientation.Y = virtualObject.transform.rotation.y;
                bbox.Center.Orientation.Z = virtualObject.transform.rotation.z;
                bbox.Center.Orientation.W = virtualObject.transform.rotation.w;
                // Size
                bbox.Size = new geometry_msgs.msg.Vector3();
                bbox.Size.X = bounds.size.x;
                bbox.Size.Y = bounds.size.y;
                bbox.Size.Z = bounds.size.z;

                boxes[i] = bbox;
            }
            msg.Boxes = boxes;
            msg.WriteNativeMessage();
            this.mObstacleBoundingBoxPublisher.Publish(msg);

            //Debug.Log("Published " + virtualObjects.Count + " virtual objects to topic: " + virtualObjectsTopicName);
        }


        public virtual Vector3 GetLinearVelocity()
        {
            return this.mLinearVelocity;
        }

        public virtual Vector3 GetAngularVelocity()
        {
            return this.mAngularVelocity;
        }

        void WorldTransformationUpdate(nav_msgs.msg.Odometry msg)
        {
            this.mWorldPosition = new Vector3(
                (float)msg.Pose.Pose.Position.Z,
                (float)msg.Pose.Pose.Position.Y,
                -(float)msg.Pose.Pose.Position.X
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
