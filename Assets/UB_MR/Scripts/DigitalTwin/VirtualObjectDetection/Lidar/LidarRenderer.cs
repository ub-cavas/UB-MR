using UnityEngine;
using System.Collections.Generic;
using Unity.Netcode;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    public class LidarRenderer : MonoBehaviour
    {
        [SerializeField] Material mLidarVisualMaterial;
        [SerializeField] float mLineWidth = 0.02f;
    
        List<LineRenderer> mLineRenderers = new List<LineRenderer>();
        GameObject mLineParent;
        bool isInitialized = false;
        
        LineRenderer CreateScanVisual(int idx)
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
        
        public void InitializeVisualization(Transform inTransform)
        {
            if (!isInitialized)
            {
                mLineParent = new GameObject("LidarLines");
                mLineParent.transform.SetParent(inTransform);
                mLineParent.transform.localPosition = Vector3.zero;
                mLineParent.transform.localRotation = Quaternion.identity;
                isInitialized = true;
            }
        }

        public void VisualizeScan(Vector4[] inScan, Transform inTransform)
        {
            if (!isInitialized)
                InitializeVisualization(inTransform);
            if (inScan == null || inScan.Length == 0)
            {
                Debug.LogWarning("No valid points to visualize in Lidar scan.");
                return;
            }
            
            // Add more lines to render
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
                    SetVisual(false, this.mLineRenderers[i], inScan[i]);
            }
            else
            {
                // Update existing lines
                for (int i = 0; i < inScan.Length; i++)
                    SetVisual(true, this.mLineRenderers[i], inScan[i]);
            }
        }
        
        void SetVisual(bool inRender, LineRenderer inLineRenderer, Vector4 inPoint)
        {
            Vector3 point = new Vector3(inPoint.x, inPoint.y, inPoint.z);
            if (inRender)
            {
                inLineRenderer.gameObject.SetActive(true);
                inLineRenderer.SetPosition(0, Vector3.zero); // Set the start point at the origin of the object
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
    }
}
