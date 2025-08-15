using UnityEngine;
using sensor_msgs.msg;
using ROS2;
using System.Collections.Generic;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    public class LidarVisualizer : MonoBehaviour
    {
        [SerializeField] protected string mLidarTopicName = "/scan"; // Topic name for Lidar scans
        [SerializeField] Material mLidarVisualMaterial; // Material for the line renderer
        [SerializeField] float mLineWidth = 0.02f; // Width of the lines
        [SerializeField] Gradient mDistanceGradient; // Gradient for color coding distances
        
        protected ROS2Node mNode;
        protected ISubscription<sensor_msgs.msg.LaserScan> mLidarSubscriber;
        
        // LineRenderer components for visualization
        protected List<LineRenderer> mLineRenderers = new List<LineRenderer>();
        protected List<(Vector3, float)> mCartesianData = new List<(Vector3, float)>();
        protected GameObject mLineParent;
        protected bool mIsReading = false;

        protected virtual void Awake()
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
                this.mLidarSubscriber = this.mNode.CreateSubscription<sensor_msgs.msg.LaserScan>(mLidarTopicName, UpdateCartesianData, qosProfile);
            }
            
            // Initialize visualization components
            InitializeVisualization();
        }

        protected virtual void Update()
        {
            VisualizeMostRecentScan();
        }

        protected void InitializeVisualization()
        {
            // Create parent object for organization
            mLineParent = new GameObject("LidarLines");
            mLineParent.transform.SetParent(transform);
            mLineParent.transform.localPosition = Vector3.zero;
            mLineParent.transform.localRotation = Quaternion.identity;
            
            // Initialize default gradient if not set
            if (mDistanceGradient == null)
            {
                mDistanceGradient = new Gradient();
                GradientColorKey[] colorKeys = new GradientColorKey[3];
                colorKeys[0] = new GradientColorKey(Color.red, 0.0f);    // Close range - red
                colorKeys[1] = new GradientColorKey(Color.yellow, 0.5f); // Medium range - yellow  
                colorKeys[2] = new GradientColorKey(Color.green, 1.0f);  // Far range - green
                GradientAlphaKey[] alphaKeys = new GradientAlphaKey[2];
                alphaKeys[0] = new GradientAlphaKey(1.0f, 0.0f);
                alphaKeys[1] = new GradientAlphaKey(1.0f, 1.0f);
                mDistanceGradient.SetKeys(colorKeys, alphaKeys);
            }
        }

        

        void VisualizeMostRecentScan()
        {
            List<(Vector3, float)> inScan = mCartesianData;
            if (inScan == null || inScan.Count == 0)
            {
                Debug.LogWarning("No valid points to visualize in Lidar scan.");
                return;
            }
            this.mIsReading = true;
            if (inScan.Count > mLineRenderers.Count)
            {
                // Add more lines
                int newScans = inScan.Count - mLineRenderers.Count;
                for (int i = 0; i < newScans; i++)
                {
                    LineRenderer lr = CreateScanVisual(i);
                    mLineRenderers.Add(lr);
                }
                // Render them
                for (int i = 0; i < inScan.Count; i++)
                    SetVisual(true, this.mLineRenderers[i], inScan[i]);
            }
            // "Remove" some lines
            else if (inScan.Count < mLineRenderers.Count)
            {
                for (int i = 0; i < inScan.Count; i++)
                    SetVisual(true, this.mLineRenderers[i], inScan[i]);
                // Don't render excess lines
                for (int i = mLineRenderers.Count - 1; i >= inScan.Count; i--)
                    SetVisual(false, this.mLineRenderers[i], (Vector3.zero, 0f));
            }
            else
            {
                // Update existing lines
                for (int i = 0; i < inScan.Count; i++)
                    SetVisual(true, this.mLineRenderers[i], inScan[i]);
            }
            this.mIsReading = false;
        }

        void SetVisual(bool inRender, LineRenderer inLineRenderer, (Vector3, float) inScan)
        {
            if (inRender)
            {
                inLineRenderer.gameObject.SetActive(true);
                inLineRenderer.SetPosition(0, Vector3.zero); // Set the start point at the origin
                inLineRenderer.SetPosition(1, inScan.Item1); // Set the end point at
            }
            else
            {
                inLineRenderer.gameObject.SetActive(false);
            }
        }

        protected void SetVisual(bool inRender, LineRenderer inLineRenderer, Vector4 inPoint)
        {
            Vector3 point = new Vector3(inPoint.x, inPoint.y, inPoint.z);
            if (inRender)
            {
                inLineRenderer.gameObject.SetActive(true);
                inLineRenderer.SetPosition(0, Vector3.zero); // Set the start point at the origin
                inLineRenderer.SetPosition(1, inPoint); // Set the end point 
                inLineRenderer.startColor = Color.white;
                if (inPoint.w > 0)
                    inLineRenderer.endColor = Color.red;
                else
                    inLineRenderer.endColor = Color.white;
                
            }
            else
            {
                inLineRenderer.gameObject.SetActive(false);
            }
        }

        protected void SetVisual(LineRenderer inLineRenderer, bool isVirtualHit, Vector3 inPosition)
        {
            if (isVirtualHit)
            {
                inLineRenderer.gameObject.SetActive(true);
                inLineRenderer.SetPosition(0, Vector3.zero); // Set the start point at the origin
                inLineRenderer.SetPosition(1, inPosition); // Set the end point 
                //inLineRenderer.endColor = Color.red;
            }
            else
            {
                //inLineRenderer.gameObject.SetActive(true);
                inLineRenderer.SetPosition(0, Vector3.zero); // Set the start point at the origin
                inLineRenderer.SetPosition(1, inPosition); // Set the end point 
            }
        }

        protected LineRenderer CreateScanVisual(int idx)
        {
            GameObject lineObj = new GameObject($"LidarLine_{mLineRenderers.Count + idx}");
            lineObj.transform.SetParent(mLineParent.transform);
            lineObj.transform.localPosition = Vector3.zero;
            LineRenderer lr = lineObj.AddComponent<LineRenderer>();
            lr.material = mLidarVisualMaterial;
            lr.startWidth = mLineWidth;
            lr.endWidth = mLineWidth;
            lr.positionCount = 2;
            lr.useWorldSpace = false; // Use local space relative to parent
            return lr;
        }
            
        void UpdateCartesianData(sensor_msgs.msg.LaserScan scan)
        {
            if (!this.mIsReading)
            {
                List<(Vector3, float)> cartesianScan = new List<(Vector3, float)>();
                float currentAngle = scan.Angle_min;
                for (int i = 0; i < scan.Ranges.Length; i++)
                {
                    float range = scan.Ranges[i];
                    if (IsValidMeasurement(range, scan.Range_min, scan.Range_max))
                    {
                        float x = range * Mathf.Cos(currentAngle);
                        float z = range * Mathf.Sin(currentAngle); // ROS Y becomes Unity Z
                        Vector3 point = new Vector3(x, 0, z);
                        cartesianScan.Add((point, range));
                    }
                    currentAngle += scan.Angle_increment;
                }
                this.mCartesianData = cartesianScan;
            }
        }
        
        bool IsValidMeasurement(float range, float rangeMin, float rangeMax)
        {
            return !float.IsNaN(range) && !float.IsInfinity(range) && range >= rangeMin && range <= rangeMax && range > 0;
        }
        
        void OnDestroy()
        {
            if (mLineParent != null)
            {
                DestroyImmediate(mLineParent);
            }
            
            // Clean up ROS2 subscription
            if (mLidarSubscriber != null)
            {
                mLidarSubscriber.Dispose();
            }
            
        }
    }
}
