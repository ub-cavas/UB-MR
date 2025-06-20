using ROS2;
using UnityEngine;

[RequireComponent(typeof(TransformInterpolater))]
public class DigitalTwin : MonoBehaviour
{
    ROS2Node mNode;

    ISubscription<nav_msgs.msg.Odometry> mWorldTransformationSubscriber;

    TransformInterpolater mTransformInterpolater;

    // TODO: Refactor subscriber to ROS2_Bridge?
    void Start()
    {
        if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
        {
            this.mNode = ROS2_Bridge.ROS_CORE.CreateNode("Digital_Twin");
            this.mWorldTransformationSubscriber = this.mNode.CreateSubscription<nav_msgs.msg.Odometry>("world_transform", WorldTransformationUpdate);
            this.mTransformInterpolater = GetComponent<TransformInterpolater>();
        }
    }

    void Update()
    {
        
        
    }

    void WorldTransformationUpdate(nav_msgs.msg.Odometry msg)
    {
        Vector3 position = new Vector3(
            (float)msg.Pose.Pose.Position.X,
            (float)msg.Pose.Pose.Position.Y,
            (float)msg.Pose.Pose.Position.Z
        );
        Quaternion rotation = new Quaternion(
            (float)msg.Pose.Pose.Orientation.X,
            (float)msg.Pose.Pose.Orientation.Y,
            (float)msg.Pose.Pose.Orientation.Z,
            (float)msg.Pose.Pose.Orientation.W
        );
        this.mTransformInterpolater.OnReceiveNetworkPosition(position);
    }

    

    
}
