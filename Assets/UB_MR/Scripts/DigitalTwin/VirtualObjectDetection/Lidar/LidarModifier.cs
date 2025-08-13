using ROS2;
using UnityEngine;


namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    //TODO: Support 3D angles
    public struct UnityLaserScan
    {
        public Vector3 position;
        public uint hasIntersection;
    }
    
    
    public class LidarModifier
    {
        ROS2Node mNode;
        ISubscription<sensor_msgs.msg.LaserScan> mLidarSubscriber;
        Transform mLidarTransform;
        ComputeShader mLiDARComputeShader;
        ComputeBuffer mBuffer;
        UnityLaserScan[] mData;
        UnityLaserScan[] mModifiedData;
        bool mIsUpdatingBuffer;
        int mKernel;

        SDFTexture mSDF;
        // Raymarching Parameters (Global to all SDFs)
        float maxDistance = 10f;
        float hitThreshold = 0.001f;
        int maxIterations = 128;
        float stepScale = 0.9f;
        Vector3 mWorldPosition = Vector3.zero;
        Vector3 mRotation = Vector3.zero;
        

        public LidarModifier(Transform inLidarSensorTransform, string inTopicName, ComputeShader inComputeShader, ROS2Node inNode, QualityOfServiceProfile inQoSProfile, SDFTexture inSDFs)
        {
            this.mLiDARComputeShader = inComputeShader;
            this.mLidarTransform = inLidarSensorTransform;
            this.mNode = inNode;
            this.mLidarSubscriber = this.mNode.CreateSubscription<sensor_msgs.msg.LaserScan>(inTopicName, ReadLiDAR, inQoSProfile);
            this.mIsUpdatingBuffer = false;

            // TODO: Support multiple SDFs
            this.mKernel = this.mLiDARComputeShader.FindKernel("LiDAR_Modifier");
            this.mSDF = inSDFs;
            // This part should be done for each SDF
            this.mLiDARComputeShader.SetTexture(this.mKernel, "_SDFTexture", this.mSDF.sdf);
            this.mLiDARComputeShader.SetMatrix("_SDFToWorld", this.mSDF.worldToSDFTexCoords.inverse);
            this.mLiDARComputeShader.SetMatrix("_WorldToSDF", this.mSDF.worldToSDFTexCoords);
            // This part is global for all SDFs
            this.mLiDARComputeShader.SetFloat("_MaxDistance", maxDistance);
            this.mLiDARComputeShader.SetFloat("_HitThreshold", hitThreshold);
            this.mLiDARComputeShader.SetInt("_MaxIterations", maxIterations);
            this.mLiDARComputeShader.SetFloat("_StepScale", stepScale);
        }

        public UnityLaserScan[] GetScan()
        {
            return this.mData;
        }

        void ReadLiDAR(sensor_msgs.msg.LaserScan inLaserScan)
        {
            if (this.mData == null)
                this.mData = new UnityLaserScan[inLaserScan.Ranges.Length];
            if (this.mModifiedData== null)
                this.mModifiedData = new UnityLaserScan[this.mData.Length];
            PreprocessLiDAR(inLaserScan); 
        }

        public UnityLaserScan[] ModifyLiDAR(Transform inTransform)
        {
            // Update position + rotation of LiDAR
            this.mWorldPosition = inTransform.position;
            this.mRotation = inTransform.rotation.eulerAngles;
            
            if (this.mData == null || this.mModifiedData == null)
                return null;
            if (this.mBuffer == null)
            {
                int size = (2 * sizeof(float) * 3) + sizeof(float) + sizeof(uint);
                this.mBuffer = new ComputeBuffer(this.mData.Length, size);
            }
                

            if (this.mIsUpdatingBuffer)
                return this.mModifiedData;
            
            this.mBuffer.SetData(this.mData);
            this.mLiDARComputeShader.SetBuffer(this.mKernel, "laserScanBuffer", this.mBuffer);
            this.mLiDARComputeShader.Dispatch(this.mKernel, this.mData.Length, 1, 1);
            this.mBuffer.GetData(this.mModifiedData);  
            return this.mModifiedData;
        }

        void PreprocessLiDAR(sensor_msgs.msg.LaserScan inLaserScan)
        {
            this.mIsUpdatingBuffer = true;
            float currentAngle = inLaserScan.Angle_min;
            int j = 0;
            for (int i = 0; i < inLaserScan.Ranges.Length; i++)
            {
                this.mData[j].position = this.mWorldPosition + GetPointPosition(currentAngle, inLaserScan.Ranges[i]);
                this.mData[j].hasIntersection = 0;
                
                // TODO: Insert Logic to keep track of the bad LiDAR Readings (index) and reinstate them after GPU modification
                if (!IsValidMeasurement(inLaserScan.Ranges[i], inLaserScan.Range_min, inLaserScan.Range_max))
                    this.mData[j].position = new Vector3(0,0,0);
                
                currentAngle += inLaserScan.Angle_increment;
                j++;
            }
            this.mIsUpdatingBuffer = false;
        }

        Vector3 GetPointPosition(float inAngle, float inRange)
        {
            return new Vector3(inRange * Mathf.Cos(inAngle), 0f, inRange * Mathf.Sin(inAngle));
        }
        
        bool IsValidMeasurement(float range, float rangeMin, float rangeMax)
        {
            return !float.IsNaN(range) && !float.IsInfinity(range) && range >= rangeMin && range <= rangeMax && range > 0;
        }

        public void CleanUp()
        {
            this.mBuffer?.Dispose();
            this.mBuffer?.Release();
        }

    }
}
