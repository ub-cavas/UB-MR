using ROS2;
using System.Collections.Generic;
using UnityEngine;

public class DigitalTwin : MonoBehaviour
{
    [SerializeField] float virtualObjectDetectionRadius = 30f; // Radius in which virtual objects are detected


    ROS2Node mNode;

    ISubscription<nav_msgs.msg.Odometry> mWorldTransformationSubscriber;

    VirtualObjectDetector mVirtualObjectDetector;
    IPublisher<vision_msgs.msg.BoundingBox3DArray> mObstacleBoundingBoxPublisher; // TODO: integrate BoundingBox3D message type

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
                this.mObstacleBoundingBoxPublisher = this.mNode.CreatePublisher<vision_msgs.msg.BoundingBox3DArray>("virtual_obstacles");
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
        vision_msgs.msg.BoundingBox3DArray msg = new vision_msgs.msg.BoundingBox3DArray();
        // TODO: Populate Header
        List<VirtualObject> virtualObjects = this.mVirtualObjectDetector.GetNearbyObstacles(virtualObjectDetectionRadius);
        foreach (VirtualObject virtualObject in virtualObjects)
        {
            Bounds bounds = virtualObject.GetBoundingBox();
            vision_msgs.msg.BoundingBox3D bbox = new vision_msgs.msg.BoundingBox3D();
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
            // TODO: Add to array
            //msg.Boxes.Initialize(virtualObjects.Count);
        }
        this.mObstacleBoundingBoxPublisher.Publish(msg); // TODO

    }
}
