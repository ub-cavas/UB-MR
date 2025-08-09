using UnityEngine;
using sensor_msgs.msg;
using ROS2;
using System.Collections.Generic;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    public class LidarVisualizer : MonoBehaviour
    {
        [SerializeField] string lidarTopicName = "/scan"; // Topic name for Lidar scans
        [SerializeField] Material lineMaterial; // Material for the line renderer
        [SerializeField] float lineWidth = 0.02f; // Width of the lines
        [SerializeField] Gradient distanceGradient; // Gradient for color coding distances
        
        ROS2Node mNode;
        ISubscription<LaserScan> mLidarSubscriber;
        
        // LineRenderer components for visualization
        List<LineRenderer> mLineRenderers = new List<LineRenderer>();
        private GameObject linesParent;

        void Awake()
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
                
                this.mLidarSubscriber = this.mNode.CreateSubscription<LaserScan>(lidarTopicName, VisualizeLidarScan, qosProfile);
            }
            
            // Initialize visualization components
            InitializeVisualization();
        }
        
        void InitializeVisualization()
        {
            // Create parent object for organization
            linesParent = new GameObject("LidarLines");
            linesParent.transform.SetParent(transform);
            linesParent.transform.localPosition = Vector3.zero;
            linesParent.transform.localRotation = Quaternion.identity;
            
            // Initialize default gradient if not set
            if (distanceGradient == null)
            {
                distanceGradient = new Gradient();
                GradientColorKey[] colorKeys = new GradientColorKey[3];
                colorKeys[0] = new GradientColorKey(Color.red, 0.0f);    // Close range - red
                colorKeys[1] = new GradientColorKey(Color.yellow, 0.5f); // Medium range - yellow  
                colorKeys[2] = new GradientColorKey(Color.green, 1.0f);  // Far range - green
                GradientAlphaKey[] alphaKeys = new GradientAlphaKey[2];
                alphaKeys[0] = new GradientAlphaKey(1.0f, 0.0f);
                alphaKeys[1] = new GradientAlphaKey(1.0f, 1.0f);
                distanceGradient.SetKeys(colorKeys, alphaKeys);
            }
        }

        void UpdateLines(List<(Vector3, float)> inScan)
        {
            if (inScan == null || inScan.Count == 0)
            {
                Debug.LogWarning("No valid points to visualize in Lidar scan.");
                return;
            }


            if (inScan.Count > mLineRenderers.Count)
            {
                // Add more lines
                int newScans = inScan.Count - mLineRenderers.Count;
                for (int i = 0; i < newScans; i++)
                {
                    LineRenderer lr = CreateScanVisual(i);
                    mLineRenderers.Add(lr);
                }
                for (int i = 0; i < inScan.Count; i++)
                {
                    SetRendererPosition(this.mLineRenderers[i], inScan[i]);
                }
                
            }
            // "Remove" some lines
            else if (inScan.Count < mLineRenderers.Count)
            {
                for (int i = 0; i < inScan.Count; i++)
                {
                    SetRendererPosition(this.mLineRenderers[i], inScan[i]);
                }
                // Don't render excess lines
                for (int i = mLineRenderers.Count - 1; i >= inScan.Count; i--)
                {

                }
            }
        }

        void SetRendererPosition(LineRenderer inLineRenderer, (Vector3, float) inScan)
        {
            inLineRenderer.SetPosition(0, Vector3.zero); // Set the start point at the origin
            inLineRenderer.SetPosition(1, inScan.Item1); // Set the end point at
            inLineRenderer.startColor = distanceGradient.Evaluate(Mathf.InverseLerp(inScan.Item2, 0, inScan.Item2));
            inLineRenderer.endColor = distanceGradient.Evaluate(Mathf.InverseLerp(inScan.Item2, 0, inScan.Item2));
        }

        LineRenderer CreateScanVisual(int idx)
        {
            GameObject lineObj = new GameObject($"LidarLine_{mLineRenderers.Count + idx}");
            lineObj.transform.SetParent(linesParent.transform);
            lineObj.transform.localPosition = Vector3.zero;
            LineRenderer lr = lineObj.AddComponent<LineRenderer>();
            lr.material = lineMaterial;
            lr.startWidth = lineWidth;
            lr.endWidth = lineWidth;
            lr.positionCount = 2;
            lr.useWorldSpace = false; // Use local space relative to parent
            return lr;
        }
            
      

        List<(Vector3, float)> GetCartesianData(LaserScan scan)
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
            return cartesianScan;
        }

        
        
        void VisualizeLidarScan(LaserScan scan)
        {
            Debug.Log($"Received Lidar Scan with {scan.Ranges.Length} points from topic {lidarTopicName}");
            if (scan.Ranges == null || scan.Ranges.Length == 0)
                return;
                

            // Convert scan data to Unity coordinates and create lines
            List<Vector3> validPoints = new List<Vector3>();
            List<float> validDistances = new List<float>();
            
            float currentAngle = scan.Angle_min;
            
            for (int i = 0; i < scan.Ranges.Length; i++)
            {
                float range = scan.Ranges[i];
                
                // Check if the measurement is valid
                if (IsValidMeasurement(range, scan.Range_min, scan.Range_max))
                {
                    float x = range * Mathf.Cos(currentAngle);
                    float z = range * Mathf.Sin(currentAngle); // ROS Y becomes Unity Z
                    Vector3 point = new Vector3(x, 0, z);
                    
                    validPoints.Add(point);
                    validDistances.Add(range);
                }
                
                currentAngle += scan.Angle_increment;
            }
            
            // Create line renderers for valid points
            CreateLinesFromPoints(validPoints, validDistances, scan.Range_min, scan.Range_max);
        }
        
        bool IsValidMeasurement(float range, float rangeMin, float rangeMax)
        {
            return !float.IsNaN(range) && !float.IsInfinity(range) && range >= rangeMin && range <= rangeMax && range > 0;
        }
        
        void CreateLinesFromPoints(List<Vector3> points, List<float> distances, float minRange, float maxRange)
        {
            Vector3 origin = Vector3.zero; // Relative to this GameObject
            
            for (int i = 0; i < points.Count; i++)
            {
                // Create a new line renderer for each point
                GameObject lineObj = new GameObject($"LidarLine_{i}");
                lineObj.transform.SetParent(linesParent.transform);
                lineObj.transform.localPosition = Vector3.zero;
                
                LineRenderer lr = lineObj.AddComponent<LineRenderer>();
                
                // Configure line renderer
                lr.material = lineMaterial;
                lr.startWidth = lineWidth;
                lr.endWidth = lineWidth;
                lr.positionCount = 2;
                lr.useWorldSpace = false; // Use local space relative to parent
                
                // Set positions
                lr.SetPosition(0, origin);
                lr.SetPosition(1, points[i]);
                
                // Set color based on distance
                float normalizedDistance = Mathf.InverseLerp(minRange, maxRange, distances[i]);
                Color lineColor = distanceGradient.Evaluate(normalizedDistance);
                lr.startColor = lineColor;
                lr.endColor = lineColor;
                
                // Store reference for cleanup
                this.mLineRenderers.Add(lr);
            }
        }
        
        
        
        void OnDestroy()
        {
            if (linesParent != null)
            {
                DestroyImmediate(linesParent);
            }
            
            // Clean up ROS2 subscription
            if (mLidarSubscriber != null)
            {
                mLidarSubscriber.Dispose();
            }
            
        }
    }
}
