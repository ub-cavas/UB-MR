using ROS2;
using System.Collections.Generic;
using UnityEngine;

public class DigitalTwin : MonoBehaviour
{
    [SerializeField] float virtualObjectDetectionRadius = 30f; // Radius in which virtual objects are detected


    ROS2Node mNode;

    ISubscription<nav_msgs.msg.Odometry> mWorldTransformationSubscriber;

    VirtualObjectDetector mVirtualObjectDetector;
    //IPublisher<vision_msgs.msg.BoundingBox3D> mObstacleBoundingBoxPublisher; // TODO: integrate BoundingBox3D message type

    Vector3 mWorldPosition = Vector3.zero;
    Quaternion mWorldRotation = Quaternion.identity;

    // TODO: Refactor subscriber to ROS2_Bridge?
    void Start()
    {
        if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
        {
            if (this.mNode == null)
            {
                this.mNode = ROS2_Bridge.ROS_CORE.CreateNode("Digital_Twin");
                // World Transformation Subscriber
                this.mWorldTransformationSubscriber = this.mNode.CreateSubscription<nav_msgs.msg.Odometry>("world_transform", On_WorldTransformationUpdate);
                // Obstacle Bounding Box Publisher
                this.mVirtualObjectDetector = new VirtualObjectDetector(this.transform);
                //this.mObstacleBoundingBoxPublisher = this.mNode.CreatePublisher<sensor_msgs.msg.NavSatFix>(vision_msgs.msg.BoundingBox3D);
            }
            
        }
    }

    void Update()
    {
        UpdateWorldTransformation();
    }

    void On_WorldTransformationUpdate(nav_msgs.msg.Odometry msg)
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

    void PublishNearbyVirtualObjects()
    {
        List<VirtualObject> virtualObjects = this.mVirtualObjectDetector.GetNearbyObstacles(virtualObjectDetectionRadius);
        foreach (VirtualObject virtualObject in virtualObjects)
        {
            Bounds bounds = virtualObject.GetBoundingBox();
            // TODO: Create a 3D bounding box for the nearby virtual objects
        }
        //this.mObstacleBoundingBoxPublisher.Publish(new vision_msgs.msg.BoundingBox3D); // TODO

    }
}
