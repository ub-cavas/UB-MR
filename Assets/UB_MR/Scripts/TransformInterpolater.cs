using UnityEngine;

public class TransformInterpolater : MonoBehaviour
{
    // positions for interpolation
    Vector3 mStartPosition;
    Vector3 mCurrentPosition;
    Vector3 mTargetPosition;

    // timing
    float mStartTime;
    float mDuration;          // how long to interpolate over
    float mLastReceiveTime;   // when we got the previous packet
    float simTime;

    void Start()
    {
        mStartPosition = transform.position;
        mTargetPosition = transform.position;
        mLastReceiveTime = Time.time;
        mDuration = 0.1f;     // a small default to avoid div0
        mStartTime = mLastReceiveTime;
    }

    void Update()
    {
        this.simTime = Time.time;
        // how far between start & target we are
        float t = (Time.time - mStartTime) / mDuration;
        t = Mathf.Clamp01(t);
        transform.position = Vector3.Lerp(mStartPosition, mTargetPosition, t);
        this.mCurrentPosition = transform.position;
    }

    // Call this whenever a new network message arrives:
    public void OnReceiveNetworkPosition(Vector3 newPos)
    {
        // capture “where we are right now”
        mStartPosition = mCurrentPosition;
        mTargetPosition = newPos;

        // compute how long we should take to reach the new target
        mDuration = Mathf.Max(0.0001f, this.simTime - mLastReceiveTime);
        mStartTime = this.simTime;
        mLastReceiveTime = this.simTime;
    }
}
