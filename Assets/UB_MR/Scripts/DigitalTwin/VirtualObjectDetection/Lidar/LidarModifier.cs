using ROS2;
using UnityEngine;


namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    public class LidarModifier
    {
        ROS2Node mNode;
        ISubscription<sensor_msgs.msg.LaserScan> mLidarSubscriber;
        ComputeShader mLiDARComputeShader;
        ComputeBuffer mBuffer;
        Vector4[] mData;
        Vector4[] mModifiedData;
        int mKernel;

        SDFTexture mSDF;
        // Raymarching Parameters (Global to all SDFs)
        float maxDistance = 10f;
        float hitThreshold = 0.01f;
        int maxIterations = 128;
        float stepScale = 0.9f;
        Vector3 mWorldPosition = Vector3.zero;
        private bool mIsDirty;
        

        public LidarModifier(string inTopicName, ComputeShader inComputeShader, ROS2Node inNode, QualityOfServiceProfile inQoSProfile, SDFTexture inSDFs)
        {
            this.mLiDARComputeShader = inComputeShader;
            this.mNode = inNode;
            this.mLidarSubscriber = this.mNode.CreateSubscription<sensor_msgs.msg.LaserScan>(inTopicName, ReadLiDAR, inQoSProfile);

            // TODO: Support multiple SDFs
            this.mKernel = this.mLiDARComputeShader.FindKernel(inComputeShader.name);
            this.mSDF = inSDFs;
            // This part should be done for each SDF
            this.mLiDARComputeShader.SetTexture(this.mKernel, "_SDF", this.mSDF.sdf);
            this.mLiDARComputeShader.SetMatrix("_WorldToSDFSpace", this.mSDF.worldToSDFTexCoords);
            // This part is global for all SDFs
            this.mLiDARComputeShader.SetFloat("_Margin", 0.0f);
            this.mLiDARComputeShader.SetFloat("_MaxDistance", maxDistance);
            this.mLiDARComputeShader.SetFloat("_HitThreshold", hitThreshold);
            this.mLiDARComputeShader.SetInt("_MaxIterations", maxIterations);
            this.mLiDARComputeShader.SetVector("_Origin", this.mWorldPosition);
        }

        public Vector4[] GetScan()
        {
            return this.mData;
        }
        
        public Vector4[] GetModifiedScan(Transform inTransform)
        {
            if (this.mIsDirty || this.mData == null || this.mModifiedData == null)
                return null;
            if (this.mBuffer == null)
                this.mBuffer = new ComputeBuffer(this.mData.Length, sizeof(float) * 4);
            
            // Update position of LiDAR
            // TODO: ROTATION?
            this.mLiDARComputeShader.SetVector("_Origin", inTransform.position);
            this.mLiDARComputeShader.SetMatrix("_WorldToSDFSpace", this.mSDF.worldToSDFTexCoords);
            // Prepare data for GPU
            this.mBuffer.SetData(this.mData);
            this.mLiDARComputeShader.SetBuffer(this.mKernel, "_Points", this.mBuffer);
            // TODO: Add logic for scans with more than 1210 rays
            this.mLiDARComputeShader.Dispatch(this.mKernel, 19, 1, 1);
            this.mBuffer.GetData(this.mModifiedData);  
            return this.mModifiedData;
        }

        void ReadLiDAR(sensor_msgs.msg.LaserScan inLaserScan)
        {
            this.mIsDirty = true;
            if (this.mData == null)
                this.mData = new Vector4[inLaserScan.Ranges.Length];
            if (this.mModifiedData== null)
                this.mModifiedData = new Vector4[this.mData.Length];
            // Preprocess Data
            float currentAngle = inLaserScan.Angle_min;
            int j = 0;
            for (int i = 0; i < inLaserScan.Ranges.Length; i++)
            {
                if (!IsValidMeasurement(inLaserScan.Ranges[i], inLaserScan.Range_min, inLaserScan.Range_max))
                    this.mData[j] = new Vector4(0,0,0,0);
                else
                {
                    Vector3 pos = this.mWorldPosition + new Vector3(inLaserScan.Ranges[i] * Mathf.Cos(currentAngle), 0f, inLaserScan.Ranges[i] * Mathf.Sin(currentAngle));
                    this.mData[j] = new Vector4(pos.x, pos.y, pos.z, 0); // TODO: Do this calculation on GPU
                }
                    
                currentAngle += inLaserScan.Angle_increment;
                j++;
            }
            this.mIsDirty = false;
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
