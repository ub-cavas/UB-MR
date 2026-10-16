using CAVAS.UB_MR;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    /// <summary>
    /// Shared rigid transform between CARLA map coordinates (meters, yaw degrees)
    /// and the locally aligned Unity world. Mesh import scale is not part of this frame.
    /// </summary>
    [DisallowMultipleComponent]
    public class CarlaMapFrame : MonoBehaviour
    {
        [SerializeField] private Module module;
        [Tooltip("Imported map orientation before client alignment, never the aligned startup rotation.")]
        [SerializeField] private Vector3 uncorrectedMapRotationEuler = new(-90f, -90f, -180f);
        [Tooltip("CARLA origin in the uncorrected Unity basis, in meters. Shared by ego and traffic.")]
        [SerializeField] private Vector3 originOffset = Vector3.zero;

        private bool _reportedUnavailable;

        /// <summary>Runtime-only initialization; never adds components to an editor scene.</summary>
        public static CarlaMapFrame GetOrCreate(Module module)
        {
            if (!Application.isPlaying || module == null)
                return null;

            if (!module.TryGetComponent(out CarlaMapFrame frame))
                frame = module.gameObject.AddComponent<CarlaMapFrame>();

            // Bind an unconfigured component without overwriting its calibration.
            if (frame.module == null)
                frame.module = module;
            return frame;
        }

        public bool TryUnityWorldToCarlaPose(Vector3 worldPosition, Quaternion worldRotation,
            out Vector3 carlaLocation, out float carlaYaw)
        {
            carlaLocation = default;
            carlaYaw = 0f;
            if (!TryGetFrame(out Vector3 origin, out Quaternion rotation))
                return false;

            Quaternion inverse = Quaternion.Inverse(rotation);
            Vector3 forward = inverse * (worldRotation * Vector3.forward);
            if (forward.x * forward.x + forward.z * forward.z < 1e-8f)
                return false;

            Vector3 position = inverse * (worldPosition - origin);
            carlaLocation = new Vector3(position.z, position.x, position.y);
            carlaYaw = Mathf.Repeat(Mathf.Atan2(forward.x, forward.z) * Mathf.Rad2Deg + 180f, 360f) - 180f;
            return true;
        }

        public bool TryCarlaPoseToUnityWorld(Vector3 carlaLocation, float carlaYaw, out Pose worldPose)
        {
            worldPose = default;
            if (!TryGetFrame(out Vector3 origin, out Quaternion rotation))
                return false;

            Vector3 position = new(carlaLocation.y, carlaLocation.z, carlaLocation.x);
            worldPose = new Pose(origin + rotation * position, rotation * Quaternion.Euler(0f, carlaYaw, 0f));
            return true;
        }

        private bool TryGetFrame(out Vector3 origin, out Quaternion rotation)
        {
            origin = default;
            rotation = Quaternion.identity;
            Transform root = module != null ? module.MapRoot : null;
            if (root == null)
            {
                if (!_reportedUnavailable)
                    Debug.LogWarning("[CarlaMapFrame] Map reference unavailable; pose conversion paused.", this);
                _reportedUnavailable = true;
                return false;
            }

            if (_reportedUnavailable)
                Debug.Log("[CarlaMapFrame] Map reference recovered; pose conversion resumed.", this);
            _reportedUnavailable = false;

            rotation = root.rotation * Quaternion.Inverse(Quaternion.Euler(uncorrectedMapRotationEuler));
            origin = root.position + rotation * originOffset;
            return true;
        }
    }
}
