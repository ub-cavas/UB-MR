using ROS2;
using UnityEngine;

public class DigitalTwin : MonoBehaviour
{
    ROS2Node mNode;

    ISubscription<nav_msgs.msg.Odometry> mWorldTransformationSubscriber;

    Vector3 mWorldPosition = Vector3.zero;
    Quaternion mWorldRotation = Quaternion.identity;

    // TODO: Refactor subscriber to ROS2_Bridge?
    void Start()
    {
        if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
        {
            this.mNode = ROS2_Bridge.ROS_CORE.CreateNode("Digital_Twin");
            this.mWorldTransformationSubscriber = this.mNode.CreateSubscription<nav_msgs.msg.Odometry>("world_transform", WorldTransformationUpdate);
        }
    }

    void Update()
    {
        UpdateWorldTransformation();
    }

    void WorldTransformationUpdate(nav_msgs.msg.Odometry msg)
    {
        this.mWorldPosition = new Vector3(
            (float)msg.Pose.Pose.Position.X,
            (float)msg.Pose.Pose.Position.Y,
            (float)msg.Pose.Pose.Position.Z
        );
        this.mWorldRotation = new Quaternion(
            (float)msg.Pose.Pose.Orientation.X,
            (float)msg.Pose.Pose.Orientation.Y,
            (float)msg.Pose.Pose.Orientation.Z,
            (float)msg.Pose.Pose.Orientation.W
        );
        Debug.Log("World Coordinates: [" + this.mWorldPosition.ToString() + "]");
    }



    // Read World Position from WorldTransformation ROS2 Node
    void UpdateWorldTransformation()
    {
        this.gameObject.transform.position = this.mWorldPosition;
        this.gameObject.transform.rotation = this.mWorldRotation;
        // TODO: integrate linear velocity and angular velocity
    }

    
}
