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

    [SerializeField] float velocityGain = 0.03f; // Proportional gain for position error correction
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
        if (smoothUpdate)
            SmoothUpdate();
    }

    void FixedUpdate()
    {
        if (!smoothUpdate)
            SnapUpdate();
  
            
            
        
    }

    void WorldTransformationUpdate(nav_msgs.msg.Odometry msg)
    {
        this.mPrevWorldPosition = this.mWorldPosition;
        this.mWorldPosition = new Vector3(
            (float)msg.Pose.Pose.Position.Z,
            (float)msg.Pose.Pose.Position.Y,
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
        this.mWorldRotation = new Quaternion(
             -q_ros.y,    // Unity X = ROS Y
             q_ros.z,    // Unity Y =  ROS Z
            -q_ros.x,    // Unity Z =  ROS X
             q_ros.w     // w stays the same
        );


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
        // 1) get the raw ROS twist
        Vector3 msgVel = this.mLinearVelocity;
        // 2) compute position error
        Vector3 error = this.mWorldPosition - rb.position;
        // 3) turn that into a velocity to close the gap in one physics frame
        Vector3 errorVel = velocityGain * error / Time.fixedDeltaTime;
        // 4) blend real and corrective velocities
        Vector3 correctedVel = msgVel + errorVel;

        // 5) apply to the Rigidbody
        rb.linearVelocity = correctedVel;
        //rb.angularVelocity = this.mAngularVelocity;
        rb.rotation = this.mWorldRotation; // TODO: Smooth rotation
        /*
        // Rotation
        Vector3 msgRot = this.mAngularVelocity;
        error = this.mWorldRotation.eulerAngles - rb.rotation.eulerAngles;
        errorVel = angularGain * error / Time.fixedDeltaTime;*/

    }
}
