using ROS2;
using Unity.VisualScripting;
using UnityEditor.UI;
using UnityEngine;
using static UnityEngine.UI.Image;

[RequireComponent(typeof(Rigidbody))]
public class DigitalTwin : MonoBehaviour
{
    [SerializeField] bool smoothUpdate;

    ROS2Node mNode;

    ISubscription<nav_msgs.msg.Odometry> mWorldTransformationSubscriber;

    [SerializeField] float velocityGain = 1; // Proportional gain for position error correction
    [SerializeField] float angularGain = 0.03f;
    bool updated = false;
    Rigidbody rb;
    Vector3 mPrevWorldPosition = Vector3.zero;
    Vector3 mWorldPosition = Vector3.zero;
    Vector3 mAngularVelocity = Vector3.zero;
    Vector3 mLinearVelocity = Vector3.zero;
    Quaternion mWorldRotation = Quaternion.identity;

    Vector3 error = Vector3.zero;

    void Awake()
    {
        rb = this.GetComponent<Rigidbody>();
    }

    // TODO: Refactor subscriber to ROS2_Bridge?
    void Start()
    {
        if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
        {
            this.mNode = ROS2_Bridge.ROS_CORE.CreateNode(gameObject.name + "_Digital_Twin");
            this.mWorldTransformationSubscriber = this.mNode.CreateSubscription<nav_msgs.msg.Odometry>("world_transform", WorldTransformationUpdate);
        }
    }

    void Update()
    {
        
    }

    void FixedUpdate()
    {
        if (!smoothUpdate)
            SnapUpdate();
        if (smoothUpdate)
            SmoothUpdate();




    }

    void WorldTransformationUpdate(nav_msgs.msg.Odometry msg)
    {
        this.mPrevWorldPosition = this.mWorldPosition;
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


    void SnapUpdate()
    {
        rb.position = this.mWorldPosition;
        rb.rotation = this.mWorldRotation;

    }

    void SmoothUpdate()
    {
        // 1) compute position error
        Vector3 error = this.mWorldPosition - rb.position;
        if (error.magnitude > 5f) // If the error is too large, teleport
        {
            rb.Move(this.mWorldPosition, this.mWorldRotation);
            return;
        }
        else
        {
            Vector3 errorVel = error;
            Vector3 localVel = transform.InverseTransformDirection(this.mLinearVelocity);
            Vector3 avg = (localVel + errorVel) / 2f;
            rb.linearVelocity = velocityGain * avg;
            Debug.Log(rb.linearVelocity);
            rb.MoveRotation(this.mWorldRotation);
        }
    }
}
