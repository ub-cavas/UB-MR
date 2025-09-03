using System.Collections;
using ROS2;
using UnityEngine;
using System.Linq;
using sensor_msgs.msg;
using System;
using std_msgs.msg;


namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    public enum LidarType
    {
        LaserScan,
        PointCloud2
    }
    
    public class LidarModifier
    {
        ROS2Node mNode;
        ISubscription<LaserScan> mLidarSubscriber2D;
        ISubscription<PointCloud2> mLidarSubscriber3D;
        IPublisher<PointCloud2> mModifiedLidarPublisher;
        ComputeShader mLiDARComputeShader;
        ComputeBuffer mInputBuffer;
        ComputeBuffer mOutputBuffer;
        Vector3[] mData;
        Vector4[] mModifiedData;
        int mKernel;
        int mPoints;

        SDFTexture mSDF;
        Vector3 mWorldPosition = Vector3.zero;
        private bool mIsWritingToBuffer;
        

        public LidarModifier(MonoBehaviour inOwner, LidarType inType, string inTopicName, int inRaysPerScan, ComputeShader inComputeShader, ROS2Node inNode, QualityOfServiceProfile inQoSProfile, SDFTexture inSDFs)
        {
            this.mLiDARComputeShader = inComputeShader;
            this.mNode = inNode;
            if (inType == LidarType.LaserScan)
                this.mLidarSubscriber2D = this.mNode.CreateSubscription<sensor_msgs.msg.LaserScan>(inTopicName, ReadLaserScan, inQoSProfile);
            else
            {
                inOwner.StartCoroutine(CreateBuffersAndSubscribe(inTopicName, inQoSProfile, inRaysPerScan));
                this.mModifiedLidarPublisher = inNode.CreatePublisher<PointCloud2>(inTopicName + "_modified");
            }
                
            
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

        IEnumerator CreateBuffersAndSubscribe(string inTopicName, QualityOfServiceProfile inQoSProfile, int inCount)
        {
            const int inputStride = sizeof(float) * 3;
            const int outputStride = sizeof(float) * 4;
            this.mData = new Vector3[inCount];
            this.mModifiedData = new Vector4[inCount];
            Debug.Log("CPU Buffers Created!");
            yield return new WaitForSeconds(0.5f);
            this.mInputBuffer = new ComputeBuffer(inCount, inputStride, ComputeBufferType.Structured);
            this.mOutputBuffer = new ComputeBuffer(inCount, outputStride);
            Debug.Log("GPU Buffers Created!");
            yield return null;
            this.mLidarSubscriber3D = this.mNode.CreateSubscription<sensor_msgs.msg.PointCloud2>(inTopicName, ReadPointCloud2, inQoSProfile);
            Debug.Log("Subscribed to LiDAR!");
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

        public Vector4[] GetOriginalScan()
        {
            return this.mData.Select(v => new Vector4(v.x, v.y, v.z, 0f)).ToArray();
        }
        
        public Vector4[] GetModifiedLaserScan(Transform inTransform)
        {
            if (this.mIsWritingToBuffer || this.mData == null || this.mModifiedData == null)
                return GetOriginalScan(); // Return the Original Scan
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
            const int THREADS = 64; // must match shader
            int groupsX = (this.mData.Length + THREADS - 1) / THREADS;
            this.mLiDARComputeShader.Dispatch(this.mKernel, groupsX, 1, 1);
            this.mOutputBuffer.GetData(this.mModifiedData);  
            return this.mModifiedData;
        }

        public Vector4[] GetModifiedPointCloud2(Transform inTransform)
        {
            if (this.mInputBuffer == null || this.mOutputBuffer == null)
            {
                Debug.LogError("Compute Buffers Not Initialized!");
                return GetOriginalScan();
            }

            if (this.mData == null || this.mPoints <= 0)
            {
                if (this.mData == null)
                    Debug.LogError("Data Buffer Not Initialized!");
                else
                    Debug.LogError("No Points Received! Publishing 0 Points!");
                return Array.Empty<Vector4>();
            }

            if (this.mIsWritingToBuffer)
            {
                // TODO: Should I Send Previous Modified Scan or Original Scan?????
                Debug.LogError("GPU Took Too Long, Sending Last Valid PCD"); 
                return GetOriginalScan();
            }
            
            
            // Update SDF Matrices
            this.mLiDARComputeShader.SetMatrix("_WorldToSDFSpace", this.mSDF.worldToSDFTexCoords);
            this.mLiDARComputeShader.SetMatrix("_SDFToWorldSpace", this.mSDF.worldToSDFTexCoords.inverse);
            // Update LiDAR Transform on Compute Shader
            this.mLiDARComputeShader.SetVector("_RayOriginWorld", inTransform.position);
            Matrix4x4 rs = Matrix4x4.TRS(Vector3.zero, inTransform.rotation, inTransform.lossyScale);
            this.mLiDARComputeShader.SetMatrix("_LocalToWorldRS", rs);
            this.mLiDARComputeShader.SetMatrix("_WorldToLocalRS", rs.inverse);
            // Set Buffers
            this.mInputBuffer.SetData(this.mData);
            this.mLiDARComputeShader.SetBuffer(this.mKernel, "_Points", this.mInputBuffer);
            this.mLiDARComputeShader.SetBuffer(this.mKernel, "_ModifiedPoints", this.mOutputBuffer);
            
            const int THREADS = 64; // must match shader
            int groupsX = (this.mPoints + THREADS - 1) / THREADS;
            this.mLiDARComputeShader.Dispatch(this.mKernel, groupsX, 1, 1);
            this.mOutputBuffer.GetData(this.mModifiedData, managedBufferStartIndex: 0, computeBufferStartIndex: 0, count: this.mPoints);  
            
            Debug.Log("MODIFIED: " + this.mPoints + " points");
            return this.mModifiedData;
        }

        public Vector4[] PublishModifiedPointCloud2(Transform inTransform, string inLidarFrameID = "base_link")
        {
            Vector4[] pcd = GetModifiedLaserScan(inTransform); 
            if (pcd == null)
                return null;
            string frameId = inLidarFrameID;
            var msg = new sensor_msgs.msg.PointCloud2();
            msg.Header.Frame_id = inLidarFrameID;
            builtin_interfaces.msg.Time time = new builtin_interfaces.msg.Time();
            time.Sec = (int)UnityEngine.Time.timeSinceLevelLoad;
            msg.Header.Stamp = time;
            
            PointField[] fields = new PointField[3];
            // X Data
            fields[0] = new PointField();
            fields[0].Name = "x";
            fields[0].Offset = 0;
            fields[0].Datatype = PointField.FLOAT32;
            fields[0].Count = 1;
            // Y DATA
            fields[1] = new PointField();
            fields[1].Name = "y";
            fields[1].Offset = 4;
            fields[1].Datatype = PointField.FLOAT32;
            fields[1].Count = 1;
            // Z DATA
            fields[2] = new PointField();
            fields[2].Name = "z";
            fields[2].Offset = 8;
            fields[2].Datatype = PointField.FLOAT32;
            fields[2].Count = 1;
            // Point DATA
            const int pointStep = 3 * sizeof(float);
            uint width = (uint)pcd.Length;
            uint height = 1;
            uint rowStep = pointStep * width;
            byte[] data = new byte[pcd.Length * pointStep];
            int off = 0;
            for (int i = 0; i < pcd.Length; i++)
            {
                Vector3 p = Ros2UnityConversions.UnityToRosPoint(pcd[i]);
                Array.Copy(BitConverter.GetBytes(p.x), 0, data, off, 4); off += 4;
                Array.Copy(BitConverter.GetBytes(p.y), 0, data, off, 4); off += 4;
                Array.Copy(BitConverter.GetBytes(p.z), 0, data, off, 4); off += 4;
            }
            bool isBigEndian = !BitConverter.IsLittleEndian;

            msg.Width = width;
            msg.Height = height;
            msg.Fields = fields;
            msg.Is_bigendian = isBigEndian;
            msg.Point_step = (uint)pointStep;
            msg.Row_step = (uint)rowStep;
            msg.Data = data;
            msg.Is_dense = false;
            
            msg.WriteNativeMessage();
            this.mModifiedLidarPublisher.Publish(msg);
            return pcd; // return for downstream vizualization / analysis 
        }
        
        void ReadLaserScan(LaserScan inLaserScan)
        {
            if (this.mData == null)
                this.mData = new Vector3[inLaserScan.Ranges.Length];
            if (this.mModifiedData== null)
                this.mModifiedData = new Vector4[this.mData.Length];
            // Preprocess Data
            this.mIsWritingToBuffer = true;
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
            this.mIsWritingToBuffer = false;
        }

        void ReadPointCloud2(PointCloud2 inPointCloud)
        {
            if (inPointCloud == null || inPointCloud.Data == null || inPointCloud.Fields == null)
            {
                Debug.LogWarning("Invalid PointCloud2.");
                return;
            }

            this.mIsWritingToBuffer = true;
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
            
            // -- Write into mData --
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
                            this.mData[idx++] = Ros2UnityConversions.RosToUnityPoint(new Vector3(x, y, z));
                        }
                    }
                }
            }
            this.mPoints = count;
            this.mIsWritingToBuffer = false;
            //Debug.Log("Wrote " + this.mPoints + " points to Data Buffer.");
        }
        
        bool IsValidMeasurement(float range, float rangeMin, float rangeMax)
        {
            return !float.IsNaN(range) && !float.IsInfinity(range) && range >= rangeMin && range <= rangeMax && range > 0;
        }

        public void CleanUp()
        {
            this.mInputBuffer?.Dispose();
            this.mInputBuffer?.Release();
            this.mOutputBuffer?.Dispose();
            this.mOutputBuffer?.Release();

            if (Ros2cs.Ok())
            {
                if (this.mLidarSubscriber2D != null)
                    this.mNode.RemoveSubscription<LaserScan>(this.mLidarSubscriber2D);
                if (this.mLidarSubscriber3D != null)
                    this.mNode.RemoveSubscription<LaserScan>(this.mLidarSubscriber3D);
                this.mLidarSubscriber2D = null;
                this.mLidarSubscriber3D = null;
            }
            
        }

    }
}
