using System.Net.Sockets;
using System.Text;
using CAVAS.UB_MR;
using CAVAS.UB_MR.DT.Vehicle;
using Newtonsoft.Json;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    /// <summary>
    /// Publishes the physical ego vehicle (LincolnMKZ / DynamicAgent) through the
    /// Main Menu's Redis connection for the server-side CARLA ego renderer.
    /// Scene-only launches retain the UDP bridge path.
    ///
    /// The LincolnMKZ GameObject is instantiated at runtime, so the DynamicAgent
    /// is discovered by periodic scene search rather than an inspector reference.
    /// While no pose source is available nothing is published, and the CARLA
    /// replica is destroyed downstream by render_ego.py's stale timeout.
    ///
    /// The pose is converted to the CARLA frame before sending, so the wire
    /// format matches the traffic messages (CARLA coordinates, yaw in degrees).
    /// CarlaMapFrame is shared with TrafficRenderer so both directions use the
    /// same origin and undo/apply the complete runtime map alignment.
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

        [Header("UDP fallback (scene-only launches)")]
        [SerializeField] private string bridgeHost = "127.0.0.1";
        [SerializeField] private int bridgePort = 12346;
        [Header("Publication")]
        [SerializeField] private float publishRateHz = 20f;

        [Header("Map frame (shared with TrafficRenderer)")]
        [SerializeField] private Module module;
        private CarlaMapFrame _mapFrame;
        private bool _waitingForModule;

        private DynamicAgent _agent;
        private float _nextAgentSearchTime;
        private bool _hasPoseSource;

        private ServerConnection serverConnection;
        private UdpClient _udpClient;
        private float _sendInterval;
        private float _nextSendTime;

        void Awake()
        {
            if (string.IsNullOrEmpty(egoId))
                egoId = SystemInfo.deviceUniqueIdentifier;

            ResolveMapFrame();
        }

        void Start()
        {
            serverConnection = ServerConnection.Instance;
            if (serverConnection == null) _udpClient = new UdpClient();
            _sendInterval = 1f / Mathf.Max(publishRateHz, 1f);
            if (serverConnection != null)
            {
                Debug.Log("[EgoPublisher] Using the Main Menu server connection.");
                return;
            }
            Debug.Log($"[EgoPublisher] Publishing ego '{egoId}' to {bridgeHost}:{bridgePort} at {publishRateHz} Hz " +
                      "(waiting for DynamicAgent to spawn)");
        }

        void LateUpdate()
        {
            if (_udpClient == null && (serverConnection == null || !serverConnection.IsConnected)) return;
            if (Time.time < _nextSendTime) return;
            _nextSendTime = Time.time + _sendInterval;

            if (!TryGetEgoWorldPose(out Vector3 worldPosition, out Quaternion worldRotation))
                return;

            SendEgoPose(worldPosition, worldRotation);
        }

        void OnDestroy()
        {
            _udpClient?.Close();
            _udpClient = null;
        }

        private bool TryGetEgoWorldPose(out Vector3 worldPosition, out Quaternion worldRotation)
        {
            worldPosition = default;
            worldRotation = Quaternion.identity;

            if (egoTransformOverride != null)
            {
                worldPosition = egoTransformOverride.position;
                worldRotation = egoTransformOverride.rotation;
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
            worldRotation = _agent.WorldRotation();
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

        private void SendEgoPose(Vector3 worldPosition, Quaternion worldRotation)
        {
            CarlaMapFrame frame = ResolveMapFrame();
            if (frame == null || !frame.TryUnityWorldToCarlaPose(worldPosition, worldRotation,
                    out Vector3 location, out float yaw))
                return;

            var payload = new
            {
                id = egoId,
                blueprint = carlaBlueprint,
                color = vehicleColor,
                location = new { x = location.x, y = location.y, z = location.z },
                yaw
            };

            if (serverConnection != null)
            {
                serverConnection.PublishEgo(payload);
                return;
            }

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

        private CarlaMapFrame ResolveMapFrame()
        {
            if (!Application.isPlaying) return null;
            if (_mapFrame != null && module != null) return _mapFrame;

            if (module == null)
                module = FindFirstObjectByType<Module>();
            if (module == null)
            {
                if (!_waitingForModule)
                    Debug.LogWarning("[EgoPublisher] Waiting for a Module before publishing poses.", this);
                _waitingForModule = true;
                return null;
            }

            if (_waitingForModule)
                Debug.Log("[EgoPublisher] Module acquired; pose conversion available.", this);
            _waitingForModule = false;
            _mapFrame = CarlaMapFrame.GetOrCreate(module);
            return _mapFrame;
        }
    }
}
