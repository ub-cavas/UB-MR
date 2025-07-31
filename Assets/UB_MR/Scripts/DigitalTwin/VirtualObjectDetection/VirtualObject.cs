using UnityEngine;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection
{
    [RequireComponent(typeof(Collider))]
    public class VirtualObject : MonoBehaviour
    {
        // Start is called once before the first execution of Update after the MonoBehaviour is created
        void Start()
        {
            VirtualBoundingBoxDetector.AddVirtualObjectToDatabase(this);
        }

        public Bounds GetBoundingBox()
        {
            return GetComponent<Collider>().bounds;
        }
    }
}
