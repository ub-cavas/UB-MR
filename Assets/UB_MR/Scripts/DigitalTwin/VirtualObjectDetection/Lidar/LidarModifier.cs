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
            this.mLiDARComputeShader.SetMatrix("_SDFToWorldSpace", this.mSDF.worldToSDFTexCoords.inverse);
            
            // Global for ALL SDFs
            // (Default Parameters)
            float maxDistance = 10f;
            float hitThreshold = 0.0001f;
            int maxIterations = 128;
            this.UpdateSDFRaytraceParameters(maxDistance, hitThreshold, maxIterations);
            this.mLiDARComputeShader.SetVector("_Origin", this.mWorldPosition);
        }
        
        /// <summary>
        /// This is mostly a Debugging Method ... don't call it in Update() if in production 
        /// </summary>
        /// <param name="inMaxDistance"></param>
        /// <param name="inHitThreshold"></param>
        /// <param name="inMaxIterations"></param>
        /// <param name="inMargin"></param>
        public void UpdateSDFRaytraceParameters(float inMaxDistance, float inHitThreshold, int inMaxIterations, float inMargin = 0.1f)
        {
            this.mLiDARComputeShader.SetFloat("_Margin", inMargin);
            this.mLiDARComputeShader.SetFloat("_MaxDistance", inMaxDistance);
            this.mLiDARComputeShader.SetFloat("_HitThreshold", inHitThreshold);
            this.mLiDARComputeShader.SetInt("_MaxIterations", inMaxIterations);
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
            
            // Update SDF Matrices
            this.mLiDARComputeShader.SetMatrix("_WorldToSDFSpace", this.mSDF.worldToSDFTexCoords);
            this.mLiDARComputeShader.SetMatrix("_SDFToWorldSpace", this.mSDF.worldToSDFTexCoords.inverse);
            // Update LiDAR Transform on Compute Shader
            this.mLiDARComputeShader.SetVector("_RayOriginWorld", inTransform.position);
            Matrix4x4 rs = Matrix4x4.TRS(Vector3.zero, inTransform.rotation, inTransform.lossyScale);
            this.mLiDARComputeShader.SetMatrix("_LocalToWorldRS", rs);
            this.mLiDARComputeShader.SetMatrix("_WorldToLocalRS", rs.inverse);
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
