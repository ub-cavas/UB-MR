using System.Collections;
using ROS2;
using UnityEngine;
using sensor_msgs.msg;
using CAVAS.UB_MR.ROS2;
using System.Collections.Concurrent;


namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    public class LidarModifier : Sensor
    {
        ComputeShader mLiDARComputeShader;
        int mKernel;
        Vector3 mWorldPosition = Vector3.zero;

        #region Data Queues
        // Incoming Point Clouds
        ConcurrentQueue<PointCloud2> mInput_PCD_Queue;
        readonly object _staged_lock = new object();
        Vector3[] mStaged_PCD;
        // Outgoing Point Clouds
        ConcurrentQueue<PointCloud2> mOutput_PCD_Queue;
        readonly object _ready_lock = new object();
        Vector4[] mReadyPCD;
        // GPU Buffers
        ComputeBuffer mInput_GPU_Buffer;
        ComputeBuffer mOutput_GPU_Buffer;
        #endregion

        #region ROS2
        ROS2Node mNode;
        ISubscription<PointCloud2> mPointCloudSubscriber;
        IPublisher<PointCloud2> mPointCloudPublisher;
        #endregion
        
        #region SDF
        SDFTexture mSDF;
        #endregion

        public LidarModifier(MonoBehaviour inOwner, string inTopicName, int inRaysPerScan, ComputeShader inComputeShader, ROS2Node inNode, QualityOfServiceProfile inQoSProfile, SDFTexture inSDFs)
        {
            // -- Cache the provided compute shader --
            this.mLiDARComputeShader = inComputeShader;

            // -- Set up subscriptions to LiDAR topics and create topic for modified LiDAR data
            this.mNode = inNode;
            inOwner.StartCoroutine(Subscribe_To_PCD2(inTopicName, inQoSProfile, inRaysPerScan));
            this.mPointCloudPublisher = inNode.CreatePublisher<PointCloud2>(inTopicName + "_modified");
            // Set up Queues
            this.mInput_PCD_Queue = new ConcurrentQueue<PointCloud2>();
            this.mOutput_PCD_Queue = new ConcurrentQueue<PointCloud2>();

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

        #region ROS2 Subscription Initialization
        IEnumerator Subscribe_To_PCD2(string inTopicName, QualityOfServiceProfile inQoSProfile, int inCount)
        {
            // -- Optimization: Pre-allocate the "staging" CPU Buffers --
            this.mStaged_PCD = new Vector3[inCount];
            this.mReadyPCD = new Vector4[inCount];
            Debug.Log("CPU Buffers Created!");
            yield return new WaitForSeconds(0.5f);

            // -- Optimization: Pre-allocate compute buffers (matched with "staging" buffers) --
            const int inputStride = sizeof(float) * 3;
            const int outputStride = sizeof(float) * 4;
            this.mInput_GPU_Buffer = new ComputeBuffer(inCount, inputStride, ComputeBufferType.Structured);
            this.mOutput_GPU_Buffer = new ComputeBuffer(inCount, outputStride);
            Debug.Log("GPU Buffers Created!");
            yield return null;

            // LiDAR Queued for GPU Processing
            this.mPointCloudSubscriber = this.mNode.CreateSubscription<PointCloud2>(inTopicName, (inPointCloud) => EnqueuePCD(inPointCloud, this.mInput_PCD_Queue),inQoSProfile);
            Debug.Log("Subscribed to: " + inTopicName);
        }
        
        public void PublishPCD()
        {
            PointCloud2 msg = DequeuePCD(this.mOutput_PCD_Queue);
            if (msg is not null)
                this.mPointCloudPublisher.Publish(msg);
        }

        PointCloud2 ModifyPCD_InPlace(PointCloud2 inOriginalMessage, Vector4[] inNewData)
        {
            const byte FLOAT32 = PointField.FLOAT32;
            int xOff = -1, yOff = -1, zOff = -1;
            foreach (var f in inOriginalMessage.Fields)
            {
                if (f == null || f.Datatype != FLOAT32) continue;
                switch (f.Name)
                {
                    case "x": xOff = (int)f.Offset; break;
                    case "y": yOff = (int)f.Offset; break;
                    case "z": zOff = (int)f.Offset; break;
                }
            }

            int width = (int)inOriginalMessage.Width;
            int height = (int)inOriginalMessage.Height;
            int count = width * height;
            int pointStep = (int)inOriginalMessage.Point_step;
            int rowStep = (int)inOriginalMessage.Row_step;

            // --- Write function (handles endianness) ---
            unsafe void WriteF32(byte* ptr, float value)
            {
                if (!inOriginalMessage.Is_bigendian)
                {
                    *(float*)ptr = value;
                }
                else
                {
                    uint u = *(uint*)&value;
                    ptr[0] = (byte)(u >> 24);
                    ptr[1] = (byte)(u >> 16);
                    ptr[2] = (byte)(u >> 8);
                    ptr[3] = (byte)(u >> 0);
                }
            }

            // --- Write modified points back into PointCloud2.Data ---
            unsafe
                {
                    fixed (byte* pBase = inOriginalMessage.Data)
                    {
                        int idx = 0;
                        for (int rIdx = 0; rIdx < height; rIdx++)
                        {
                            byte* row = pBase + (long)rIdx * rowStep;
                            for (int cIdx = 0; cIdx < width; cIdx++)
                            {
                                byte* pt = row + (long)cIdx * pointStep;

                                // Convert back to ROS coordinate frame
                                Vector3 v3 = new Vector3(this.mReadyPCD[idx].x, this.mReadyPCD[idx].y, this.mReadyPCD[idx].z);
                                Vector3 rosPos = Ros2Utility.UnityToRos2Position(v3);

                                WriteF32(pt + xOff, rosPos.x);
                                WriteF32(pt + yOff, rosPos.y);
                                WriteF32(pt + zOff, rosPos.z);

                                idx++;
                            }
                        }
                    }
                }

            return inOriginalMessage;
        }

        #endregion

        #region Data Queue Operations
        void EnqueuePCD(PointCloud2 inPointCloud, ConcurrentQueue<PointCloud2> inQueue)
        {
            if (inPointCloud == null || inPointCloud.Data == null || inPointCloud.Fields == null)
            {
                Debug.LogWarning("Invalid PointCloud2!");
                return;
            }
            inQueue.Enqueue(inPointCloud);
        }
        
        /// <summary>
        /// Pops the MOST RECENT PCD, older messages are discarded 
        /// </summary>
        /// <param name="inQueue"></param>
        /// <returns></returns>
        PointCloud2 DequeuePCD(ConcurrentQueue<PointCloud2> inQueue)
        {
            PointCloud2 retPCD = null;
            while (inQueue.TryDequeue(out PointCloud2 pcd))
                retPCD = pcd;
            return retPCD;
        }


        void StagePCD(PointCloud2 inPointCloud)
        {
            if (inPointCloud == null || inPointCloud.Data == null || inPointCloud.Fields == null)
            {
                Debug.LogWarning("Invalid Incoming PointCloud2.");
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
            
            // --- Read function (handles endianness) ---
            unsafe float ReadF32(byte* ptr)
            {
                if (!inPointCloud.Is_bigendian)
                    return *(float*)ptr;
                else
                {
                    // Big-endian: byte-swap
                    uint u = ((uint)ptr[0] << 24) | ((uint)ptr[1] << 16) | ((uint)ptr[2] << 8) | ((uint)ptr[3] << 0);
                    return *(float*)&u;
                }
            }

            // -- Write into this.mStaged_PCD... use a lock to ensure mutual exclusion --
            lock (this._staged_lock)
            {
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
                                this.mStaged_PCD[idx++] = Ros2Utility.Ros2ToUnityPosition(new Vector3(x, y, z));
                            }
                        }
                    }
                }
            }
        }

        #endregion

        #region SDF Operations
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

        void UpdateSDFTransforms(Transform inTransform)
        {
            // Update SDF Matrices
            this.mLiDARComputeShader.SetMatrix("_WorldToSDFSpace", this.mSDF.worldToSDFTexCoords);
            this.mLiDARComputeShader.SetMatrix("_SDFToWorldSpace", this.mSDF.worldToSDFTexCoords.inverse);
            // Update LiDAR Transform on Compute Shader
            this.mLiDARComputeShader.SetVector("_RayOriginWorld", inTransform.position);
            Matrix4x4 rs = Matrix4x4.TRS(Vector3.zero, inTransform.rotation, inTransform.lossyScale);
            this.mLiDARComputeShader.SetMatrix("_LocalToWorldRS", rs);
            this.mLiDARComputeShader.SetMatrix("_WorldToLocalRS", rs.inverse);
        }
        #endregion

        /// <summary>
        /// Modifies the most recently received point cloud.
        /// This function should be called from the main thread.
        /// </summary>
        /// <param name="inTransform"></param>
        public bool TryModify(Transform inTransform)
        {
            // No-op if no data
            PointCloud2 originalPCD = DequeuePCD(this.mInput_PCD_Queue);
            if (originalPCD is null) { return false; }

            int count = (int)originalPCD.Width * (int)originalPCD.Height;
            const int THREADS = 128; // ** MUST MATCH COMPUTE SHADER **
            int groupsX = (count + THREADS - 1) / THREADS;

            // -- Update GPU cached transforms of SDFs --
            UpdateSDFTransforms(inTransform);

            // -- Prepare GPU data buffers with new data --
            StagePCD(originalPCD); // (atomically) updates this.mStaged_PCD
            this.mInput_GPU_Buffer.SetData(this.mStaged_PCD);
            this.mLiDARComputeShader.SetBuffer(this.mKernel, "_Points", this.mInput_GPU_Buffer);
            this.mLiDARComputeShader.SetBuffer(this.mKernel, "_ModifiedPoints", this.mOutput_GPU_Buffer);

            // -- Dispatch to GPU --
            this.mLiDARComputeShader.Dispatch(this.mKernel, groupsX, 1, 1);
            lock (_ready_lock) // NOTE: I suspect this is where the main thread spends most of its time
            {
                this.mOutput_GPU_Buffer.GetData(this.mReadyPCD, managedBufferStartIndex: 0, computeBufferStartIndex: 0, count: count);
                // -- Create new message with GPU results --
                PointCloud2 pcd = ModifyPCD_InPlace(originalPCD, this.mReadyPCD);
                EnqueuePCD(pcd, this.mOutput_PCD_Queue);
            }
            return true;
        }

        public override void CleanUp()
        {
            this.mInput_GPU_Buffer?.Dispose();
            this.mInput_GPU_Buffer?.Release();
            this.mOutput_GPU_Buffer?.Dispose();
            this.mOutput_GPU_Buffer?.Release();

            if (Ros2cs.Ok() && this.mPointCloudSubscriber != null)
            {
                this.mNode.RemoveSubscription<PointCloud2>(this.mPointCloudSubscriber);
                this.mPointCloudSubscriber = null; 
            }
            
        }

    }
}
