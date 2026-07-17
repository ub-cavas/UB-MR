using System.Net.Sockets;
using System.Text;
using CAVAS.UB_MR;
using CAVAS.UB_MR.DT.Vehicle;
using Newtonsoft.Json;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    /// <summary>
    /// Publishes the physical ego vehicle (LincolnMKZ / DynamicAgent) pose over
    /// UDP to ego_bridge.py, which relays it to Redis so render_ego.py can
    /// render it in CARLA.
    ///
    /// The LincolnMKZ GameObject is instantiated at runtime, so the DynamicAgent
    /// is discovered by periodic scene search rather than an inspector reference.
    /// While no pose source is available nothing is published, and the CARLA
    /// replica is destroyed downstream by render_ego.py's stale timeout.
    ///
    /// The pose is converted to the CARLA frame before sending, so the wire
    /// format matches the traffic messages (CARLA coordinates, yaw in degrees).
    /// The map-frame settings below MUST match the ones on TrafficRenderer,
    /// since this applies the exact inverse of TrafficRenderer.ApplyPose().
    /// </summary>
    public class EgoPublisher : MonoBehaviour
    {
        [Header("Ego")]
        [Tooltip("Optional manual pose source for editor testing. When assigned, it takes precedence over the DynamicAgent.")]
        [SerializeField] private Transform egoTransformOverride;
        [Tooltip("Seconds between scene searches for the runtime-spawned DynamicAgent.")]
        [SerializeField] private float agentSearchInterval = 1f;
        [Tooltip("Unique id for this client's ego. Left empty, the device unique identifier is used.")]
        [SerializeField] private string egoId = "";
        [Tooltip("CARLA blueprint to render this ego as (CARLA 0.9.16 id for the physical Lincoln MKZ 2017).")]
        [SerializeField] private string carlaBlueprint = "vehicle.lincoln.mkz_2017";
        [Tooltip("Vehicle color as R,G,B (0-255).")]
        [SerializeField] private string vehicleColor = "0,0,0";

        [Header("Network")]
        [SerializeField] private string bridgeHost = "100.83.98.37";
        [SerializeField] private int bridgePort = 12346;
        [SerializeField] private float publishRateHz = 20f;

        [Header("Map frame (must match TrafficRenderer)")]
        [SerializeField] private Module module;
        [SerializeField] private Transform mapRoot;
        [Tooltip("Apply only the runtime client-local Unity Y rotation delta from the map UI.")]
        [SerializeField] private bool applyClientMapYawCorrection = true;
        [Tooltip("Map UI Y rotation that corresponds to the uncorrected CARLA/RoadRunner traffic frame.")]
        [SerializeField] private float uncorrectedMapYawDegrees = 90f;
        [SerializeField] private Vector3 originOffset = new Vector3(1.347f, 0f, 5.916f);

        private DynamicAgent _agent;
        private float _nextAgentSearchTime;
        private bool _hasPoseSource;

        private UdpClient _udpClient;
        private float _sendInterval;
        private float _nextSendTime;

        void Awake()
        {
            if (string.IsNullOrEmpty(egoId))
                egoId = SystemInfo.deviceUniqueIdentifier;

            ResolveModule();
            ResolveMapRoot();
        }

        void Start()
        {
            _udpClient = new UdpClient();
            _sendInterval = 1f / Mathf.Max(publishRateHz, 1f);
            Debug.Log($"[EgoPublisher] Publishing ego '{egoId}' to {bridgeHost}:{bridgePort} at {publishRateHz} Hz " +
                      "(waiting for DynamicAgent to spawn)");
        }

        void Update()
        {
            if (_udpClient == null) return;
            if (Time.time < _nextSendTime) return;
            _nextSendTime = Time.time + _sendInterval;

            if (!TryGetEgoWorldPose(out Vector3 worldPosition, out float worldYaw))
                return;

            SendEgoPose(worldPosition, worldYaw);
        }

        void OnDestroy()
        {
            _udpClient?.Close();
            _udpClient = null;
        }

        private bool TryGetEgoWorldPose(out Vector3 worldPosition, out float worldYaw)
        {
            worldPosition = default;
            worldYaw = 0f;

            if (egoTransformOverride != null)
            {
                worldPosition = egoTransformOverride.position;
                worldYaw = egoTransformOverride.eulerAngles.y;
                NotePoseSource(true, "transform override");
                return true;
            }

            // The LincolnMKZ is instantiated at runtime; search on an interval,
            // not every frame. Unity's destroyed-object null semantics make this
            // re-resolve automatically if the agent despawns and respawns.
            if (_agent == null && Time.time >= _nextAgentSearchTime)
            {
                _nextAgentSearchTime = Time.time + Mathf.Max(agentSearchInterval, 0.1f);
                _agent = FindFirstObjectByType<DynamicAgent>();
            }

            if (_agent == null)
            {
                NotePoseSource(false, null);
                return false;
            }

            worldPosition = _agent.WorldPosition();
            Debug.Log($"[EgoPublisher] Ego position: {worldPosition}");
            worldYaw = _agent.WorldRotation().eulerAngles.y;
            Debug.Log($"[EgoPublisher] Ego rotation: {worldYaw}");
            NotePoseSource(true, _agent.gameObject.name);
            return true;
        }

        private void NotePoseSource(bool available, string sourceName)
        {
            if (available == _hasPoseSource) return;
            _hasPoseSource = available;

            if (available)
                Debug.Log($"[EgoPublisher] Ego pose source acquired: {sourceName}");
            else
                Debug.Log("[EgoPublisher] Ego pose source lost — publishing paused until the DynamicAgent reappears");
        }

        private void SendEgoPose(Vector3 worldPosition, float worldYaw)
        {
            // Inverse of TrafficRenderer.ApplyPose():
            //   world = yawCorrection * (unityFromCarla(pos) + originOffset)
            // therefore:
            //   unityFromCarla(pos) = inverse(yawCorrection) * world - originOffset
            float yawDelta = GetClientMapYawDeltaDegrees();
            Quaternion inverseCorrection = Quaternion.Euler(0f, -yawDelta, 0f);

            Vector3 mapLocal = inverseCorrection * worldPosition - originOffset;
            float mapLocalYaw = worldYaw - yawDelta;

            // Unity (x right, y up, z forward) -> CARLA (x forward, y right, z up).
            // Same basis swap as TrafficReceiver.VehicleData.Position(), inverted;
            // yaw keeps the same sign under this mapping.
            var payload = new
            {
                id = egoId,
                blueprint = carlaBlueprint,
                color = vehicleColor,
                location = new { x = mapLocal.z, y = mapLocal.x, z = mapLocal.y },
                yaw = mapLocalYaw
            };

            try
            {
                byte[] data = Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(payload));
                _udpClient.Send(data, data.Length, bridgeHost, bridgePort);
            }
            catch (SocketException e)
            {
                Debug.LogWarning($"[EgoPublisher] Send failed: {e.Message}");
            }
        }

        private float GetClientMapYawDeltaDegrees()
        {
            if (!applyClientMapYawCorrection)
                return 0f;

            Module mapModule = ResolveModule();
            if (mapModule != null && mapModule.HasMapRotationState)
            {
                return Mathf.DeltaAngle(uncorrectedMapYawDegrees, mapModule.CurrentMapRotationEuler.y);
            }

            Transform root = ResolveMapRoot();
            if (root == null)
                return 0f;

            return Mathf.DeltaAngle(uncorrectedMapYawDegrees, root.eulerAngles.y);
        }

        private Transform ResolveMapRoot()
        {
            if (mapRoot != null)
                return mapRoot;

            Module mapModule = ResolveModule();
            if (mapModule != null)
                mapRoot = mapModule.MapRoot;

            return mapRoot;
        }

        private Module ResolveModule()
        {
            if (module != null)
                return module;

            module = FindFirstObjectByType<Module>();
            return module;
        }
    }
}
