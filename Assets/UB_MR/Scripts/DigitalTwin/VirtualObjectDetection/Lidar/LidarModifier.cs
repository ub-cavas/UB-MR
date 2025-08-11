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

        public LidarModifier(Transform inLidarSensorTransform, string inTopicName, ComputeShader inComputeShader, ROS2Node inNode, QualityOfServiceProfile inQoSProfile)
        {
            QualityOfServiceProfile qosProfile = new QualityOfServiceProfile();
            qosProfile.SetReliability(ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT);
            qosProfile.SetHistory(HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, 2);
            qosProfile.SetDurability(DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE);
            this.mLiDARComputeShader = inComputeShader;
            this.mLidarTransform =  inLidarSensorTransform;
            this.mNode = inNode;
            this.mLidarSubscriber = this.mNode.CreateSubscription<LaserScan>(inTopicName, GPU_LidarModification, qosProfile);
        }

        void GPU_LidarModification(LaserScan inLaserScan)
        {
            LidarData[] data = PreprocessLiDAR(inLaserScan);
            LidarData[] output = new LidarData[data.Length];
            ComputeBuffer buffer = new ComputeBuffer(data.Length, sizeof(float) * 2); //Multiply by 2 because each scan contains 2 floats (angle, range)
            buffer.SetData(data);
            int kernel = this.mLiDARComputeShader.FindKernel("CartesianConvert");
            this.mLiDARComputeShader.SetBuffer(kernel, "lidarBuffer", buffer);
            this.mLiDARComputeShader.Dispatch(kernel, data.Length, 1, 1);
            buffer.GetData(output);
            buffer.Dispose();
        }

        LidarData[] PreprocessLiDAR(LaserScan inLaserScan)
        {
            LidarData[] lidarScanData = new LidarData[inLaserScan.Ranges.Length];
            float currentAngle = inLaserScan.Angle_min;
            int j = 0;
            for (int i = 0; i < inLaserScan.Ranges.Length; i++)
            {
                LidarData data = new LidarData();
                data.angle = currentAngle;
                data.range = inLaserScan.Ranges[i];

                // TODO: Insert Logic to keep track of the bad LiDAR Readings (index) and reinstate them after GPU modification
                if (!IsValidMeasurement(data.range, inLaserScan.Range_min, inLaserScan.Range_max))
                    data.range = 0;
                lidarScanData[j] = data;
                currentAngle += inLaserScan.Angle_increment;
                j++;
            }
            return lidarScanData;
        }
        
        bool IsValidMeasurement(float range, float rangeMin, float rangeMax)
        {
            return !float.IsNaN(range) && !float.IsInfinity(range) && range >= rangeMin && range <= rangeMax && range > 0;
        }

        public void CleanUp()
        {
            
        }

    }
}
