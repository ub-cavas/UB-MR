using ROS2;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class VehicleMirror : MonoBehaviour
{
    [SerializeField] string odometryTopic = "/ego_odometry";
    [SerializeField] string transformTopic = "/ego_transform";

    Rigidbody rb;
    ROS2Node mNode;
    ISubscription<nav_msgs.msg.Odometry> mWorldTransformationSubscriber;
    Vector3 egoPosition;
    Quaternion egoRotation;
    Vector3 egoLinearVelocity;
    Vector3 egoAngularVelocity;
    bool hasNewState = false;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.isKinematic = false;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
    }

    void Start()
    {
        if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
        {
            this.mNode = ROS2_Bridge.ROS_CORE.CreateNode("Vehicle_Mirror");
            this.mWorldTransformationSubscriber = this.mNode.CreateSubscription<nav_msgs.msg.Odometry>(odometryTopic, OdometryUpdate);
        }
    }

    public void UpdateVehicleOdometry(Vector3 pos, Quaternion rot, Vector3 linearVel, Vector3 angularVel)
    {
        egoPosition = pos;
        egoRotation = rot;
        egoLinearVelocity = linearVel;
        egoAngularVelocity = angularVel;
        hasNewState = true;
    }

    void FixedUpdate()
    {
        if (!hasNewState) return;

        // 1) Warp the body to the true position & rotation
        if (egoPosition != null)
            rb.MovePosition(egoPosition);
        if (egoRotation != null)
            rb.MoveRotation(egoRotation);

        // 2) Feed it the true velocities so physics takes over until next update
        rb.linearVelocity = egoLinearVelocity;
        rb.angularVelocity = egoAngularVelocity;

        hasNewState = false;
    }

    void OdometryUpdate(nav_msgs.msg.Odometry msg)
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
        Vector3 linearVelocity = new Vector3(
            (float)msg.Twist.Twist.Linear.Z,
            (float)msg.Twist.Twist.Linear.Y,
            (float)msg.Twist.Twist.Linear.X
        );
        Vector3 angularVelocity = new Vector3(
            (float)msg.Twist.Twist.Angular.X,
            (float)msg.Twist.Twist.Angular.Y,
            (float)msg.Twist.Twist.Angular.Z
        );

        Debug.Log("Linear:" + linearVelocity);
        UpdateVehicleOdometry(position, rotation, linearVelocity, angularVelocity);

    }
}
