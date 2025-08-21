using ROS2;
using UnityEngine;
using System.Linq;
using sensor_msgs.msg;


namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    public enum LidarType
    {
        TwoD,
        ThreeD
    }
    
    public class LidarModifier
    {
        ROS2Node mNode;
        ISubscription<sensor_msgs.msg.LaserScan> mLidarSubscriber2D;
        ISubscription<sensor_msgs.msg.PointCloud2> mLidarSubscriber3D;
        ComputeShader mLiDARComputeShader;
        ComputeBuffer mInputBuffer;
        ComputeBuffer mOutputBuffer;
        Vector3[] mData;
        Vector4[] mModifiedData;
        int mKernel;
        int mPoints;

        SDFTexture mSDF;
        Vector3 mWorldPosition = Vector3.zero;
        private bool mIsDirty;
        

        public LidarModifier(LidarType inType, string inTopicName, ComputeShader inComputeShader, ROS2Node inNode, QualityOfServiceProfile inQoSProfile, SDFTexture inSDFs)
        {
            this.mLiDARComputeShader = inComputeShader;
            this.mNode = inNode;
            if (inType == LidarType.TwoD)
                this.mLidarSubscriber2D = this.mNode.CreateSubscription<sensor_msgs.msg.LaserScan>(inTopicName, ReadLiDAR, inQoSProfile);
            else
                this.mLidarSubscriber3D = this.mNode.CreateSubscription<sensor_msgs.msg.PointCloud2>(inTopicName, ReadLiDAR, inQoSProfile);

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
        public void UpdateSDFRaytraceParameters(float inMaxDistance, float inHitThreshold, int inMaxIterations, float inMargin = 0.05f)
        {
            this.mLiDARComputeShader.SetFloat("_Margin", inMargin);
            this.mLiDARComputeShader.SetFloat("_MaxDistance", inMaxDistance);
            this.mLiDARComputeShader.SetFloat("_HitThreshold", inHitThreshold);
            this.mLiDARComputeShader.SetInt("_MaxIterations", inMaxIterations);
        }

        public Vector4[] GetScan()
        {
            return this.mData.Select(v => new Vector4(v.x, v.y, v.z, 0f)).ToArray();
        }
        
        public Vector4[] GetModifiedTwoDimensionalScan(Transform inTransform)
        {
            if (this.mIsDirty || this.mData == null || this.mModifiedData == null)
                return null;
            if (this.mInputBuffer == null)
                this.mInputBuffer = new ComputeBuffer(this.mData.Length, sizeof(float) * 3);
            if (this.mOutputBuffer == null)
                this.mOutputBuffer = new ComputeBuffer(this.mData.Length, sizeof(float) * 4);  
            
            // Update SDF Matrices
            this.mLiDARComputeShader.SetMatrix("_WorldToSDFSpace", this.mSDF.worldToSDFTexCoords);
            this.mLiDARComputeShader.SetMatrix("_SDFToWorldSpace", this.mSDF.worldToSDFTexCoords.inverse);
            // Update LiDAR Transform on Compute Shader
            this.mLiDARComputeShader.SetVector("_RayOriginWorld", inTransform.position);
            Matrix4x4 rs = Matrix4x4.TRS(Vector3.zero, inTransform.rotation, inTransform.lossyScale);
            this.mLiDARComputeShader.SetMatrix("_LocalToWorldRS", rs);
            this.mLiDARComputeShader.SetMatrix("_WorldToLocalRS", rs.inverse);
            // Prepare input + output buffers for GPU
            this.mInputBuffer.SetData(this.mData);
            this.mLiDARComputeShader.SetBuffer(this.mKernel, "_Points", this.mInputBuffer);
            this.mLiDARComputeShader.SetBuffer(this.mKernel, "_ModifiedPoints", this.mOutputBuffer);
            // TODO: Add logic for scans with more than 1210 rays
            this.mLiDARComputeShader.Dispatch(this.mKernel, 19, 1, 1);
            this.mOutputBuffer.GetData(this.mModifiedData);  
            return this.mModifiedData;
        }

        public Vector4[] GetModifiedThreeDimensionalScan(Transform inTransform)
        {
            // Update SDF Matrices
            this.mLiDARComputeShader.SetMatrix("_WorldToSDFSpace", this.mSDF.worldToSDFTexCoords);
            this.mLiDARComputeShader.SetMatrix("_SDFToWorldSpace", this.mSDF.worldToSDFTexCoords.inverse);
            // Update LiDAR Transform on Compute Shader
            this.mLiDARComputeShader.SetVector("_RayOriginWorld", inTransform.position);
            Matrix4x4 rs = Matrix4x4.TRS(Vector3.zero, inTransform.rotation, inTransform.lossyScale);
            this.mLiDARComputeShader.SetMatrix("_LocalToWorldRS", rs);
            this.mLiDARComputeShader.SetMatrix("_WorldToLocalRS", rs.inverse);
            
            this.mLiDARComputeShader.SetBuffer(this.mKernel, "_ModifiedPoints", this.mOutputBuffer);
            
            const int THREADS = 64; // must match shader
            int groupsX = (this.mPoints + THREADS - 1) / THREADS;
            this.mLiDARComputeShader.Dispatch(this.mKernel, groupsX, 1, 1);
            this.mOutputBuffer.GetData(this.mModifiedData);  
            return this.mModifiedData;
        }

        void ReadLiDAR(sensor_msgs.msg.LaserScan inLaserScan)
        {
            this.mIsDirty = true;
            if (this.mData == null)
                this.mData = new Vector3[inLaserScan.Ranges.Length];
            if (this.mModifiedData== null)
                this.mModifiedData = new Vector4[this.mData.Length];
            // Preprocess Data
            float currentAngle = inLaserScan.Angle_min;
            int j = 0;
            for (int i = 0; i < inLaserScan.Ranges.Length; i++)
            {
                if (!IsValidMeasurement(inLaserScan.Ranges[i], inLaserScan.Range_min, inLaserScan.Range_max))
                    this.mData[j] = new Vector3(0,0,0);
                else
                {
                    this.mData[j] = this.mWorldPosition + new Vector3(inLaserScan.Ranges[i] * Mathf.Cos(currentAngle), 0f, inLaserScan.Ranges[i] * Mathf.Sin(currentAngle)); // TODO: Do this calculation on GPU
                }
                currentAngle += inLaserScan.Angle_increment;
                j++;
            }
            this.mIsDirty = false;
        }

        // Assumes data "is dense" (no NaN's)
        void ReadLiDAR(sensor_msgs.msg.PointCloud2 inPointCloud)
        {
            if (inPointCloud == null || inPointCloud.Data == null || inPointCloud.Fields == null)
            {
                Debug.LogWarning("Invalid PointCloud2.");
                return;
            }
            
            // --- Find x,y,z fields (expect FLOAT32s) ---
            const byte FLOAT32 = PointField.FLOAT32; // 7
            int xOff = -1, yOff = -1, zOff = -1;
            foreach (var f in inPointCloud.Fields)
            {
                if (f == null || f.Datatype != FLOAT32) continue;
                switch (f.Name)
                {
                    case "x": xOff = (int)f.Offset; break;
                    case "y": yOff = (int)f.Offset; break;
                    case "z": zOff = (int)f.Offset; break;
                }
            }
            if (xOff < 0 || yOff < 0 || zOff < 0)
            {
                Debug.LogWarning("PointCloud2 is missing float32 x/y/z fields.");
                return;
            }
            
            // --- Dimensions & sanity ---
            int width  = (int)inPointCloud.Width;
            int height = (int)inPointCloud.Height;
            int count  = width * height;
            int pointStep = (int)inPointCloud.Point_step; // bytes per point
            int rowStep   = (int)inPointCloud.Row_step;   // bytes per row
            if (count <= 0 || pointStep <= 0 || rowStep < pointStep)
            {
                Debug.LogWarning("PointCloud2 has invalid dimensions/steps.");
                return;
            }
            
            long expectedBytes = (long)(height - 1) * rowStep + (long)width * pointStep;
            if (inPointCloud.Data.LongLength < expectedBytes)
            {
                Debug.LogWarning("PointCloud2 data buffer is smaller than expected.");
                return;
            }
            
            // Some publishers may set row_step == width*point_step; tolerate both
            if (inPointCloud.Data.Length < (height - 1) * rowStep + width * pointStep)
            {
                Debug.LogWarning("PointCloud2 data buffer is smaller than expected.");
                return;
            }
            
            
            // --- Ensure GPU buffer (stride = 12 bytes for float3) ---
            EnsureBuffer(count);
            
            // --- Read function (handles endianness) ---
            unsafe float ReadF32(byte* ptr)
            {
                if (!inPointCloud.Is_bigendian)
                    return *(float*)ptr;
                else
                {
                    // Big-endian: byte-swap
                    uint u = ((uint)ptr[0] << 24) | ((uint)ptr[1] << 16) | ((uint)ptr[2] <<  8) | ((uint)ptr[3] <<  0);
                    return *(float*)&u;
                }
            }
            
            // --- Write directly into the ComputeBuffer (no managed array) ---
            var writer = this.mInputBuffer.BeginWrite<Vector3>(0, count);
            unsafe
            {
                fixed (byte* pBase = inPointCloud.Data)
                {
                    int idx = 0;
                    for (int rIdx = 0; rIdx < height; rIdx++)
                    {
                        byte* row = pBase + (long)rIdx * rowStep;
                        for (int cIdx = 0; cIdx < width; cIdx++)
                        {
                            byte* pt = row + (long)cIdx * pointStep;
                            float x = ReadF32(pt + xOff);
                            float y = ReadF32(pt + yOff);
                            float z = ReadF32(pt + zOff);
                            writer[idx] = new Vector3(x, y, z);
                        }
                    }
                }
            }
            this.mInputBuffer.EndWrite<Vector3>(count);
            this.mPoints = count;
            Debug.Log("Wrote " + this.mPoints + " to Graphics Buffer.");
        }

        void EnsureBuffer(int count)
        {
            const int stride = 12;
            if (this.mInputBuffer == null || this.mInputBuffer.count != count || this.mInputBuffer.stride != stride)
            {
                this.mInputBuffer?.Release();
                this.mInputBuffer = new ComputeBuffer(count, stride, ComputeBufferType.Structured);
            }
        }
        
        bool IsValidMeasurement(float range, float rangeMin, float rangeMax)
        {
            return !float.IsNaN(range) && !float.IsInfinity(range) && range >= rangeMin && range <= rangeMax && range > 0;
        }

        public void CleanUp()
        {
            this.mInputBuffer?.Dispose();
            this.mInputBuffer?.Release();
        }

    }
}
