using ROS2;
using sensor_msgs.msg;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection
{
    public class LidarModifier
    {
        ROS2Node mNode;
        Transform lidarTransform;
        int angularSectors = 360; // Pre-sort objects by angular sectors
        LayerMask virtualObjectLayers = -1; // Which layers to consider as virtual objects
        ISubscription<LaserScan> mLidarScanSubscriber;
        IPublisher<LaserScan> mModifiedLidarScanPublisher;
        
        // Virtual object representations
        private List<LidarVirtualObject>[] sectorizedObjects;
        private List<LidarVirtualObject> allVirtualObjects;


        public LidarModifier(Transform inLidarTransform, ROS2Node inNode, string inTopicName = "/lidar/scan")
        {
            this.mNode = inNode;
            this.lidarTransform = inLidarTransform;
            this.mLidarScanSubscriber = mNode.CreateSubscription<LaserScan>(inTopicName, ModifyLaserScan);
            this.mModifiedLidarScanPublisher = mNode.CreatePublisher<LaserScan>(inTopicName + "/modified");
            
            // Initialize sectorized objects
            sectorizedObjects = new List<LidarVirtualObject>[angularSectors];
            for (int i = 0; i < angularSectors; i++)
            {
                sectorizedObjects[i] = new List<LidarVirtualObject>();
            }
            
            allVirtualObjects = new List<LidarVirtualObject>();
            InitializeVirtualObjects();
        }

        public void CleanUp()
        {
            if (Ros2cs.Ok())
            {
                // Unsubscribe from the world transformation topic
                this.mNode.RemoveSubscription<LaserScan>(this.mLidarScanSubscriber);
            }
        }
        
        /// <summary>
        /// Modifies a ROS LaserScan message by integrating virtual objects
        /// </summary>
        public void ModifyLaserScan(LaserScan scan)
        {
            float angleMin = scan.Angle_min;
            float angleIncrement = scan.Angle_increment;

            for (int i = 0; i < scan.Ranges.Length; i++)
            {
                float currentAngle = angleMin + (i * angleIncrement);
                float originalDistance = scan.Ranges[i];

                // Skip invalid measurements
                if (float.IsInfinity(originalDistance) || float.IsNaN(originalDistance))
                    continue;

                // Get virtual object distance for this ray
                float virtualDistance = GetVirtualObjectDistance(currentAngle, originalDistance);

                // Replace with closer virtual object distance
                if (virtualDistance > 0 && virtualDistance < originalDistance)
                {
                    scan.Ranges[i] = virtualDistance;
                }
            }
        }
        
        /// <summary>
        /// Get the closest virtual object distance for a given ray
        /// </summary>
        private float GetVirtualObjectDistance(float angle, float maxDistance)
        {
            // Convert angle to world space ray
            Vector3 rayOrigin = lidarTransform.position;
            Vector3 rayDirection = AngleToWorldDirection(angle);
            
            // Get relevant objects for this angular sector
            int sectorIndex = AngleToSectorIndex(angle);
            var relevantObjects = sectorizedObjects[sectorIndex];
            
            float closestDistance = float.MaxValue;
            
            // Test intersection with relevant virtual objects
            foreach (var virtualObj in relevantObjects)
            {
                float distance = virtualObj.RayIntersect(rayOrigin, rayDirection, maxDistance);
                if (distance > 0 && distance < closestDistance)
                {
                    closestDistance = distance;
                }
            }
            
            return closestDistance == float.MaxValue ? -1 : closestDistance;
        }
        
        /// <summary>
        /// Initialize virtual objects from scene colliders
        /// </summary>
        private void InitializeVirtualObjects()
        {
            // Initialize sectoring
            sectorizedObjects = new List<LidarVirtualObject>[angularSectors];
            for (int i = 0; i < angularSectors; i++)
            {
                sectorizedObjects[i] = new List<LidarVirtualObject>();
            }
            
            allVirtualObjects = new List<LidarVirtualObject>();
            
            // Find all colliders in specified layers
            Collider[] colliders = GameObject.FindObjectsOfType<Collider>()
                .Where(c => ((1 << c.gameObject.layer) & virtualObjectLayers) != 0)
                .ToArray();
            
            foreach (var collider in colliders)
            {
                LidarVirtualObject virtualObj = CreateVirtualObject(collider);
                if (virtualObj != null)
                {
                    allVirtualObjects.Add(virtualObj);
                    AssignToSectors(virtualObj);
                }
            }
            
            Debug.Log($"Initialized {allVirtualObjects.Count} virtual objects for lidar modification");
        }
        
        /// <summary>
        /// Create appropriate virtual object based on collider type
        /// </summary>
        private LidarVirtualObject CreateVirtualObject(Collider collider)
        {
            switch (collider)
            {
                case BoxCollider box:
                    return new BoxLidarObject(box);
                case SphereCollider sphere:
                    return new SphereLidarObject(sphere);
                case CapsuleCollider capsule:
                    return new CapsuleLidarObject(capsule);
                case MeshCollider mesh:
                    // For complex meshes, create a bounding box approximation for performance
                    return new BoundsLidarObject(mesh);
                default:
                    Debug.LogWarning($"Unsupported collider type: {collider.GetType()}");
                    return null;
            }
        }
        
        /// <summary>
        /// Assign virtual object to relevant angular sectors
        /// </summary>
        private void AssignToSectors(LidarVirtualObject virtualObj)
        {
            // Get object bounds in world space
            Bounds bounds = virtualObj.GetWorldBounds();
            Vector3 lidarPos = lidarTransform.position;
            
            // Calculate angular range this object spans
            Vector3 toMin = bounds.min - lidarPos;
            Vector3 toMax = bounds.max - lidarPos;
            
            float minAngle = Mathf.Atan2(toMin.z, toMin.x);
            float maxAngle = Mathf.Atan2(toMax.z, toMax.x);
            
            // Handle angle wrapping
            if (maxAngle < minAngle) maxAngle += 2 * Mathf.PI;
            
            // Assign to all relevant sectors
            int startSector = AngleToSectorIndex(minAngle);
            int endSector = AngleToSectorIndex(maxAngle);
            
            for (int i = startSector; i <= endSector; i++)
            {
                int sectorIndex = i % angularSectors;
                if (!sectorizedObjects[sectorIndex].Contains(virtualObj))
                {
                    sectorizedObjects[sectorIndex].Add(virtualObj);
                }
            }
        }
        
        /// <summary>
        /// Update positions of moving virtual objects
        /// </summary>
        public void UpdateMovingObjects()
        {
            foreach (var virtualObj in allVirtualObjects)
            {
                if (virtualObj.HasMoved())
                {
                    // Remove from all sectors
                    for (int i = 0; i < angularSectors; i++)
                    {
                        sectorizedObjects[i].Remove(virtualObj);
                    }
                    
                    // Update position and reassign to sectors
                    virtualObj.UpdateTransform();
                    AssignToSectors(virtualObj);
                }
            }
        }
        
        /// <summary>
        /// Convert lidar angle to world direction vector
        /// </summary>
        private Vector3 AngleToWorldDirection(float angle)
        {
            // Convert lidar angle to local direction (assuming forward is X, right is Z)
            Vector3 localDirection = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
            return lidarTransform.TransformDirection(localDirection);
        }
        
        /// <summary>
        /// Convert angle to sector index for spatial partitioning
        /// </summary>
        private int AngleToSectorIndex(float angle)
        {
            // Normalize angle to [0, 2π]
            while (angle < 0) angle += 2 * Mathf.PI;
            while (angle >= 2 * Mathf.PI) angle -= 2 * Mathf.PI;
            
            return Mathf.FloorToInt((angle / (2 * Mathf.PI)) * angularSectors) % angularSectors;
        }
    }
    
    /// <summary>
    /// Base class for virtual objects with analytical intersection
    /// </summary>
    public abstract class LidarVirtualObject
    {
        protected Transform transform;
        protected Vector3 lastPosition;
        protected Quaternion lastRotation;
        protected Vector3 lastScale;
        
        public LidarVirtualObject(Collider collider)
        {
            transform = collider.transform;
            UpdateTransform();
        }
        
        public abstract float RayIntersect(Vector3 rayOrigin, Vector3 rayDirection, float maxDistance);
        public abstract Bounds GetWorldBounds();
        
        public bool HasMoved()
        {
            return transform.position != lastPosition || 
                   transform.rotation != lastRotation || 
                   transform.localScale != lastScale;
        }
        
        public void UpdateTransform()
        {
            lastPosition = transform.position;
            lastRotation = transform.rotation;
            lastScale = transform.localScale;
        }
    }
    
    /// <summary>
    /// Box collider with analytical ray intersection
    /// </summary>
    public class BoxLidarObject : LidarVirtualObject
    {
        private BoxCollider boxCollider;
        
        public BoxLidarObject(BoxCollider collider) : base(collider)
        {
            boxCollider = collider;
        }
        
        public override float RayIntersect(Vector3 rayOrigin, Vector3 rayDirection, float maxDistance)
        {
            // Transform ray to local space
            Matrix4x4 worldToLocal = transform.worldToLocalMatrix;
            Vector3 localOrigin = worldToLocal.MultiplyPoint(rayOrigin);
            Vector3 localDirection = worldToLocal.MultiplyVector(rayDirection).normalized;
            
            // Get box bounds in local space
            Vector3 center = boxCollider.center;
            Vector3 size = boxCollider.size * 0.5f;
            Vector3 min = center - size;
            Vector3 max = center + size;
            
            // Ray-box intersection using slab method
            Vector3 t1 = new Vector3(
                (min.x - localOrigin.x) / localDirection.x,
                (min.y - localOrigin.y) / localDirection.y,
                (min.z - localOrigin.z) / localDirection.z
            );
            
            Vector3 t2 = new Vector3(
                (max.x - localOrigin.x) / localDirection.x,
                (max.y - localOrigin.y) / localDirection.y,
                (max.z - localOrigin.z) / localDirection.z
            );
            
            Vector3 tMin = Vector3.Min(t1, t2);
            Vector3 tMax = Vector3.Max(t1, t2);
            
            float tNear = Mathf.Max(tMin.x, Mathf.Max(tMin.y, tMin.z));
            float tFar = Mathf.Min(tMax.x, Mathf.Min(tMax.y, tMax.z));
            
            if (tNear > tFar || tFar < 0) return -1; // No intersection
            
            float distance = tNear > 0 ? tNear : tFar;
            return distance <= maxDistance ? distance : -1;
        }
        
        public override Bounds GetWorldBounds()
        {
            return boxCollider.bounds;
        }
    }
    
    /// <summary>
    /// Sphere collider with analytical ray intersection
    /// </summary>
    public class SphereLidarObject : LidarVirtualObject
    {
        private SphereCollider sphereCollider;
        
        public SphereLidarObject(SphereCollider collider) : base(collider)
        {
            sphereCollider = collider;
        }
        
        public override float RayIntersect(Vector3 rayOrigin, Vector3 rayDirection, float maxDistance)
        {
            Vector3 center = transform.TransformPoint(sphereCollider.center);
            float radius = sphereCollider.radius * Mathf.Max(transform.localScale.x, transform.localScale.y, transform.localScale.z);
            
            Vector3 oc = rayOrigin - center;
            float a = Vector3.Dot(rayDirection, rayDirection);
            float b = 2.0f * Vector3.Dot(oc, rayDirection);
            float c = Vector3.Dot(oc, oc) - radius * radius;
            
            float discriminant = b * b - 4 * a * c;
            if (discriminant < 0) return -1; // No intersection
            
            float t1 = (-b - Mathf.Sqrt(discriminant)) / (2.0f * a);
            float t2 = (-b + Mathf.Sqrt(discriminant)) / (2.0f * a);
            
            float distance = t1 > 0 ? t1 : t2;
            return distance > 0 && distance <= maxDistance ? distance : -1;
        }
        
        public override Bounds GetWorldBounds()
        {
            return sphereCollider.bounds;
        }
    }
    
    /// <summary>
    /// Capsule collider with analytical ray intersection
    /// </summary>
    public class CapsuleLidarObject : LidarVirtualObject
    {
        private CapsuleCollider capsuleCollider;
        
        public CapsuleLidarObject(CapsuleCollider collider) : base(collider)
        {
            capsuleCollider = collider;
        }
        
        public override float RayIntersect(Vector3 rayOrigin, Vector3 rayDirection, float maxDistance)
        {
            // Simplified capsule intersection - treat as sphere for performance
            // For more accuracy, implement proper capsule-ray intersection
            Vector3 center = transform.TransformPoint(capsuleCollider.center);
            float radius = capsuleCollider.radius * Mathf.Max(transform.localScale.x, transform.localScale.z);
            
            Vector3 oc = rayOrigin - center;
            float a = Vector3.Dot(rayDirection, rayDirection);
            float b = 2.0f * Vector3.Dot(oc, rayDirection);
            float c = Vector3.Dot(oc, oc) - radius * radius;
            
            float discriminant = b * b - 4 * a * c;
            if (discriminant < 0) return -1;
            
            float t1 = (-b - Mathf.Sqrt(discriminant)) / (2.0f * a);
            float t2 = (-b + Mathf.Sqrt(discriminant)) / (2.0f * a);
            
            float distance = t1 > 0 ? t1 : t2;
            return distance > 0 && distance <= maxDistance ? distance : -1;
        }
        
        public override Bounds GetWorldBounds()
        {
            return capsuleCollider.bounds;
        }
    }
    
    /// <summary>
    /// Complex mesh approximated as bounding box for performance
    /// </summary>
    public class BoundsLidarObject : LidarVirtualObject
    {
        private Bounds bounds;
        
        public BoundsLidarObject(Collider collider) : base(collider)
        {
            bounds = collider.bounds;
        }
        
        public override float RayIntersect(Vector3 rayOrigin, Vector3 rayDirection, float maxDistance)
        {
            // Simple ray-bounds intersection
            Vector3 min = bounds.min;
            Vector3 max = bounds.max;
            
            Vector3 t1 = new Vector3(
                (min.x - rayOrigin.x) / rayDirection.x,
                (min.y - rayOrigin.y) / rayDirection.y,
                (min.z - rayOrigin.z) / rayDirection.z
            );
            
            Vector3 t2 = new Vector3(
                (max.x - rayOrigin.x) / rayDirection.x,
                (max.y - rayOrigin.y) / rayDirection.y,
                (max.z - rayOrigin.z) / rayDirection.z
            );
            
            Vector3 tMin = Vector3.Min(t1, t2);
            Vector3 tMax = Vector3.Max(t1, t2);
            
            float tNear = Mathf.Max(tMin.x, Mathf.Max(tMin.y, tMin.z));
            float tFar = Mathf.Min(tMax.x, Mathf.Min(tMax.y, tMax.z));
            
            if (tNear > tFar || tFar < 0) return -1;
            
            float distance = tNear > 0 ? tNear : tFar;
            return distance <= maxDistance ? distance : -1;
        }
        
        public override Bounds GetWorldBounds()
        {
            return bounds;
        }
    }
}
