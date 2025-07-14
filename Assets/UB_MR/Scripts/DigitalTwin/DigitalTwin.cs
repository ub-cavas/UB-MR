using ROS2;
using UnityEngine;
using Unity.Netcode;

namespace CAVAS.UB_MR.DT
{
    public class DigitalTwin : NetworkBehaviour
    {
        ISubscription<nav_msgs.msg.Odometry> mWorldTransformationSubscriber;
        ROS2Node mNode;
        protected Vector3 mWorldPosition = Vector3.zero;
        protected Vector3 mAngularVelocity = Vector3.zero;
        protected Vector3 mLinearVelocity = Vector3.zero;
        protected Quaternion mWorldRotation = Quaternion.identity;

      
        public override void OnNetworkSpawn()
        {
            if (IsOwner)
            {
                ConnectToROS();
            }
        }

        void ConnectToROS()
        {
            if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
            {
                string name = gameObject.name.Replace("(Clone)", "");
                name = name.Replace(" Variant", "");
                this.mNode = ROS2_Bridge.ROS_CORE.CreateNode(name + "_Digital_Twin");
                this.mWorldTransformationSubscriber = this.mNode.CreateSubscription<nav_msgs.msg.Odometry>("/world_transform", WorldTransformationUpdate);
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
