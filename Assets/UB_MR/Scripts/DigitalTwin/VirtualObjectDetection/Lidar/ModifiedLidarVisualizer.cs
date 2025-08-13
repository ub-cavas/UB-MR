using ROS2;
using UnityEngine;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    public class ModifiedLidarVisualizer : LidarVisualizer
    {
        [SerializeField] ComputeShader lidarModComputeShader;
        [SerializeField] SDFTexture sdfTexture;
        LidarModifier mLidarModifier;


        protected override void Awake()
        {
            if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
            {
                string name = gameObject.name.Replace("(Clone)", "");
                name = name.Replace(" Variant", "");
                // This is sort of cheating but ROS2_Bridge is not immediately deleting nodes so this avoids a collision
                int randomSuffix = UnityEngine.Random.Range(0, 1000);
                this.mNode = ROS2_Bridge.ROS_CORE.CreateNode(name + "_Digital_Twin_Lidar_" + randomSuffix.ToString());
                // Lidar Scan Subscription
                QualityOfServiceProfile qosProfile = new QualityOfServiceProfile();
                qosProfile.SetReliability(ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT);
                qosProfile.SetHistory(HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, 2);
                qosProfile.SetDurability(DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE);
                
                this.mLidarModifier = new LidarModifier(this.transform, this.mLidarTopicName, lidarModComputeShader, this.mNode, qosProfile, sdfTexture);
            }
            
            // Initialize visualization components
            InitializeVisualization();
        }
        
        void VisualizeModifiedScan(UnityLaserScan[] inScan)
        {
            if (inScan == null || inScan.Length == 0)
            {
                Debug.LogWarning("No valid points to visualize in Lidar scan.");
                return;
            }
            this.mIsReading = true;
            if (inScan.Length > mLineRenderers.Count)
            {
                // Add more lines
                int newScans = inScan.Length - mLineRenderers.Count;
                for (int i = 0; i < newScans; i++)
                {
                    LineRenderer lr = CreateScanVisual(i);
                    mLineRenderers.Add(lr);
                }
                // Render them
                for (int i = 0; i < inScan.Length; i++)
                    SetVisual(true, this.mLineRenderers[i], (inScan[i].direction, inScan[i].range));
            }
            // "Remove" some lines
            else if (inScan.Length < mLineRenderers.Count)
            {
                for (int i = 0; i < inScan.Length; i++)
                    SetVisual(true, this.mLineRenderers[i], (inScan[i].direction, inScan[i].range));
                // Don't render excess lines
                for (int i = mLineRenderers.Count - 1; i >= inScan.Length; i--)
                    SetVisual(false, this.mLineRenderers[i], (Vector3.zero, 0f));
            }
            else
            {
                // Update existing lines
                for (int i = 0; i < inScan.Length; i++)
                    SetVisual(true, this.mLineRenderers[i], (inScan[i].direction, inScan[i].range));
            }
            this.mIsReading = false;
        }

        protected override void Update()
        {
            if (this.mLidarModifier != null)
            {
                //UnityLaserScan[] scan = this.mLidarModifier.ModifyLiDAR(this.transform);
                VisualizeModifiedScan(this.mLidarModifier.GetScan());
                Debug.Log("Modified Lidar");
            }
               
            //base.Update();
        }
        
        public void OnDestroy()
        {
            if (this.mLidarModifier != null)
                this.mLidarModifier.CleanUp();
        }
    }
}
