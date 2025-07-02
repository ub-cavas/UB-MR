using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DT_Predict : DigitalTwin
{
    [SerializeField] float mPositionErrorThreshold = 5f; // Position in the world to predict
    [SerializeField] float mLinearVelocity_P_Gain = 1; // Proportional gain for position error correction
    [SerializeField] float mAngularVelocity_P_Gain = 1; // Proportional gain for rotation error correction
    Rigidbody mRigidbody;
    void Awake()
    {
        this.mRigidbody = this.GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        SmoothUpdate();
    }

    void SmoothUpdate()
    {
        // 1) compute position error
        Vector3 error = this.mWorldPosition - this.mRigidbody.position;
        if (error.magnitude > mPositionErrorThreshold) // If the error is too large, teleport
        {
            mRigidbody.Move(this.mWorldPosition, this.mWorldRotation);
            return;
        }
        else
        {
            Vector3 vehicleVelocity = transform.TransformDirection(this.mLinearVelocity);
            this.mRigidbody.linearVelocity = vehicleVelocity;
        }

        // 2) angle error
        Vector3 angVel = this.mAngularVelocity;
        if (angVel.y >= -0.05f && angVel.y <= 0.05f) // If the angular velocity is too small, set it to zero
            angVel.y = 0;
        angVel.y = -angVel.y;

        float angleError = Quaternion.Angle(mRigidbody.rotation, this.mWorldRotation);
        Quaternion errorQuat = this.mWorldRotation * Quaternion.Inverse(mRigidbody.rotation);
        errorQuat.ToAngleAxis(out float axisAngle, out Vector3 axis);
        float errorRad = angleError * Mathf.Deg2Rad;
        Vector3 desiredAngularVel = axis * (mAngularVelocity_P_Gain * errorRad);
        Vector3 avgRot = (angVel + desiredAngularVel) / 2f;
        this.mRigidbody.angularVelocity = avgRot;
    }

    public override Vector3 GetLinearVelocity()
    {
        return this.mRigidbody.linearVelocity;
    }

    public override Vector3 GetAngularVelocity()
    {
        return this.mRigidbody.angularVelocity;
    }
}
