using ROS2;
using UnityEngine;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    public class ModifiedLidarVisualizer : LidarVisualizer
    {
        [SerializeField] private bool useGPU = false;
        [SerializeField] private float interval = 0.1f; // 1/10th of a second
        [SerializeField] ComputeShader lidarModComputeShader;
        [SerializeField] SDFTexture sdfTexture;
        LidarModifier mLidarModifier;
        private float timer = 0f;
        

        protected override void Update()
        {
            timer += Time.deltaTime;
        
            if (timer >= interval && this.mLidarModifier != null)
            {
                Vector4[] scan;
                if (this.useGPU)
                    scan = this.mLidarModifier.GetModifiedScan(this.transform);
                else
                    scan = this.mLidarModifier.GetScan();
                timer = 0f; // Reset timer
                VisualizeModifiedScan(scan);
            }
        }


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
                
                this.mLidarModifier = new LidarModifier(this.mLidarTopicName, lidarModComputeShader, this.mNode, qosProfile, sdfTexture);
            }
            
            // Initialize visualization components
            InitializeVisualization();
        }
        
        void VisualizeModifiedScan(Vector4[] inScan)
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
                    SetVisual(true, this.mLineRenderers[i], inScan[i]);
            }
            // "Remove" some lines
            else if (inScan.Length < mLineRenderers.Count)
            {
                for (int i = 0; i < inScan.Length; i++)
                    SetVisual(true, this.mLineRenderers[i], inScan[i]);
                // Don't render excess lines
                for (int i = mLineRenderers.Count - 1; i >= inScan.Length; i--)
                    SetVisual(false, this.mLineRenderers[i], Vector3.zero);
            }
            else
            {
                // Update existing lines
                for (int i = 0; i < inScan.Length; i++)
                    SetVisual(true, this.mLineRenderers[i], inScan[i]);
            }
            this.mIsReading = false;
        }
        
        public void OnDestroy()
        {
            if (this.mLidarModifier != null)
                this.mLidarModifier.CleanUp();
        }
        
        public static int CountIntersections(Vector4[] laserScans)
        {
            if (laserScans == null)
                return 0;
            
            int count = 0;
            for (int i = 0; i < laserScans.Length; i++)
            {
                if (laserScans[i].w == 1)
                    count++;
            }
            return count;
        }

        public static void PrintScanData(Vector4[] laserScans)
        {
            for (int i = 0; i < laserScans.Length; i++)
            {
                print(laserScans[i]);
            }
            
        }
    }
}
