using ROS2;
using sensor_msgs.msg;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    struct LidarData
    {
        public float angle;
        public float range;
    }
    
    
    public class LidarModifier
    {
        ROS2Node mNode;
        ISubscription<LaserScan> mLidarSubscriber;
        Transform mLidarTransform;
        ComputeShader mLiDARComputeShader;
        ComputeBuffer mBuffer;
        LidarData[] mData;
        LidarData[] mModifiedData;
        bool mIsUpdatingBuffer;

        public LidarModifier(Transform inLidarSensorTransform, string inTopicName, ComputeShader inComputeShader, ROS2Node inNode, QualityOfServiceProfile inQoSProfile)
        {
            QualityOfServiceProfile qosProfile = new QualityOfServiceProfile();
            qosProfile.SetReliability(ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT);
            qosProfile.SetHistory(HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, 2);
            qosProfile.SetDurability(DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE);
            this.mLiDARComputeShader = inComputeShader;
            this.mLidarTransform =  inLidarSensorTransform;
            this.mNode = inNode;
            this.mLidarSubscriber = this.mNode.CreateSubscription<LaserScan>(inTopicName, ReadLiDAR, qosProfile);
            this.mIsUpdatingBuffer = false;
        }

        void ReadLiDAR(LaserScan inLaserScan)
        {
            if (this.mData == null)
                this.mData = new LidarData[inLaserScan.Ranges.Length];
            if (this.mModifiedData== null)
                this.mModifiedData = new LidarData[this.mData.Length];
            PreprocessLiDAR(inLaserScan); 
        }

        public void ModifyLiDAR()
        {
            if (this.mData == null || this.mModifiedData == null)
                return;
            if (this.mBuffer == null)
                this.mBuffer = new ComputeBuffer(this.mData.Length, sizeof(float) * 2); //Multiply by 2 because each scan contains 2 floats (angle, range)

            if (this.mIsUpdatingBuffer)
                return;
            
            this.mBuffer.SetData(this.mData);
            int kernel = this.mLiDARComputeShader.FindKernel("CartesianConvert");
            this.mLiDARComputeShader.SetBuffer(kernel, "lidarBuffer", this.mBuffer);
            this.mLiDARComputeShader.Dispatch(kernel, this.mData.Length, 1, 1);
            this.mBuffer.GetData(this.mModifiedData);
            
        }

        void PreprocessLiDAR(LaserScan inLaserScan)
        {
            this.mIsUpdatingBuffer = true;
            float currentAngle = inLaserScan.Angle_min;
            int j = 0;
            for (int i = 0; i < inLaserScan.Ranges.Length; i++)
            {
                this.mData[j].angle = currentAngle;
                this.mData[j].range = inLaserScan.Ranges[i];
                // TODO: Insert Logic to keep track of the bad LiDAR Readings (index) and reinstate them after GPU modification
                if (!IsValidMeasurement(this.mData[j].range, inLaserScan.Range_min, inLaserScan.Range_max))
                    this.mData[j].range = 0;
                currentAngle += inLaserScan.Angle_increment;
                j++;
            }
            this.mIsUpdatingBuffer = false;
        }
        
        bool IsValidMeasurement(float range, float rangeMin, float rangeMax)
        {
            return !float.IsNaN(range) && !float.IsInfinity(range) && range >= rangeMin && range <= rangeMax && range > 0;
        }

        public void CleanUp()
        {
            this.mBuffer.Dispose();
        }

    }
}
