using CAVAS.UB_MR.Config;
using UnityEngine;

namespace CAVAS.UB_MR.DT.Sensors
{
    // Values match autoware_perception_msgs/ObjectClassification.
    public enum VirtualObjectClassification : byte
    {
        Unknown = 0, Car = 1, Truck = 2, Bus = 3, Trailer = 4,
        Motorcycle = 5, Bicycle = 6, Pedestrian = 7
    }

    public class VirtualObject : MonoBehaviour
    {
        [SerializeField] Vector3 sdfTextureSize;
        [SerializeField] int sdfTextureResolution = 32;
        [SerializeField] MeshToSDF meshToSDF;
        [SerializeField] BoxCollider boundingBox;
        [SerializeField] VirtualObjectClassification classification = VirtualObjectClassification.Car;
        RenderTexture ownedSdf;
        bool usesLidar;
        bool warnedInvalidBox;

        public VirtualObjectClassification Classification => classification;

        void Awake()
        {
            var module = FindFirstObjectByType<Module>();
            usesLidar = module != null ? module.UsesLidarModification :
                (ConfigurationManager.GetConfiguration().Item1?.recognition?.mode ??
                 VirtualObjectRecognitionMode.LidarModification) == VirtualObjectRecognitionMode.LidarModification;
            if (!usesLidar && meshToSDF != null) meshToSDF.enabled = false;
        }

        void OnEnable() => VirtualBoundingBoxDetector.AddVirtualObjectToDatabase(this);
        void OnDisable() => VirtualBoundingBoxDetector.RemoveVirtualObjectFromDatabase(this);

        void Start()
        {
            if (!usesLidar) return;
            if (meshToSDF == null || meshToSDF.sdfTexture == null)
            {
                Debug.LogWarning($"{name}: missing MeshToSDF/SDFTexture; LiDAR cannot recognize this object.", this);
                return;
            }
            meshToSDF.sdfTexture.size = sdfTextureSize;
            meshToSDF.sdfTexture.resolution = sdfTextureResolution;
            Vector3Int resolution = meshToSDF.sdfTexture.voxelResolution;
            ownedSdf = new RenderTexture(resolution.x, resolution.y, 0, RenderTextureFormat.RHalf)
            {
                dimension = UnityEngine.Rendering.TextureDimension.Tex3D,
                volumeDepth = resolution.z,
                enableRandomWrite = true,
                name = "SDFTexture"
            };
            meshToSDF.sdfTexture.sdf = ownedSdf;
        }

        public bool TryGetBoundingBox(out Vector3 center, out Quaternion rotation, out Vector3 size)
        {
            if (TryGetColliderBox(boundingBox, out center, out rotation, out size)) return true;
            if (!warnedInvalidBox)
            {
                Debug.LogWarning($"{name}: missing, inactive or invalid bounding-box collider; skipping detection.", this);
                warnedInvalidBox = true;
            }
            return false;
        }

        public static bool TryGetColliderBox(BoxCollider collider, out Vector3 center,
            out Quaternion rotation, out Vector3 size)
        {
            center = Vector3.zero;
            rotation = Quaternion.identity;
            size = Vector3.zero;
            if (collider == null || !collider.enabled || !collider.gameObject.activeInHierarchy) return false;
            Transform box = collider.transform;
            center = box.TransformPoint(collider.center);
            rotation = box.rotation;
            Quaternion inverse = Quaternion.Inverse(rotation);
            // Project transformed local edges onto the collider orientation. This also encloses
            // sheared boxes produced by a rotated child of a non-uniformly scaled parent.
            Vector3 x = inverse * box.TransformVector(Vector3.right * collider.size.x);
            Vector3 y = inverse * box.TransformVector(Vector3.up * collider.size.y);
            Vector3 z = inverse * box.TransformVector(Vector3.forward * collider.size.z);
            size = Abs(x) + Abs(y) + Abs(z);
            return Finite(center) && Finite(size) && size.x > 0 && size.y > 0 && size.z > 0;
        }

        static Vector3 Abs(Vector3 v) => new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));
        static bool Finite(Vector3 v) => !(float.IsNaN(v.x) || float.IsInfinity(v.x) ||
            float.IsNaN(v.y) || float.IsInfinity(v.y) || float.IsNaN(v.z) || float.IsInfinity(v.z));

        void OnDestroy()
        {
            VirtualBoundingBoxDetector.RemoveVirtualObjectFromDatabase(this);
            if (ownedSdf != null)
            {
                if (meshToSDF != null && meshToSDF.sdfTexture != null && meshToSDF.sdfTexture.sdf == ownedSdf)
                    meshToSDF.sdfTexture.sdf = null;
                ownedSdf.Release();
                Destroy(ownedSdf);
            }
        }
    }
}
