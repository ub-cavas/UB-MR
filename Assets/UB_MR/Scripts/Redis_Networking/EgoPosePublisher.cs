using System;
using System.Net.Sockets;
using System.Text;
using CAVAS.UB_MR;
using CAVAS.UB_MR.DT;
using CAVAS.UB_MR.DT.Vehicle;
using Newtonsoft.Json;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    public class EgoPosePublisher : MonoBehaviour
    {
        [Serializable]
        private class LocationData
        {
            public float x;
            public float y;
            public float z;
        }

        [Serializable]
        private class EgoData
        {
            public string id;
            public string blueprint;
            public string color;
            public LocationData location;
            public float yaw;
        }

        [Header("Network")]
        [SerializeField] private string bridgeHost = "127.0.0.1";
        [SerializeField] private int bridgePort = 12346;
        [SerializeField, Min(1f)] private float publishRateHz = 20f;

        [Header("Ego")]
        [SerializeField] private string egoId = "ub-mr-ego";
        [SerializeField] private string blueprint = "vehicle.lincoln.mkz_2020";
        [SerializeField] private string color = "0,0,255";
        [SerializeField] private Transform egoTransform;

        [Header("Frame Conversion")]
        [SerializeField] private Module module;
        [SerializeField] private Transform mapRoot;
        [SerializeField] private bool applyClientMapYawCorrection = true;
        [SerializeField] private float uncorrectedMapYawDegrees = 90f;
        [SerializeField] private Vector3 originOffset = Vector3.zero;

        private UdpClient udpClient;
        private float nextPublishTime;

        private void Awake()
        {
            ResolveModule();
            ResolveMapRoot();
        }

        private void Start()
        {
            ApplyEnvironmentOverrides();
            udpClient = new UdpClient();
            Debug.Log($"[EgoPosePublisher] Publishing UB-MR ego to {bridgeHost}:{bridgePort}");
        }

        private void Update()
        {
            if (Time.time < nextPublishTime)
                return;

            nextPublishTime = Time.time + (1f / publishRateHz);

            Transform source = ResolveEgoTransform();
            if (source == null)
                return;

            EgoData ego = BuildEgoData(source);
            string json = JsonConvert.SerializeObject(ego);
            byte[] data = Encoding.UTF8.GetBytes(json);
            try
            {
                udpClient.Send(data, data.Length, bridgeHost, bridgePort);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[EgoPosePublisher] Failed to publish ego pose: {e.Message}");
            }
        }

        private void OnDestroy()
        {
            udpClient?.Close();
        }

        private EgoData BuildEgoData(Transform source)
        {
            Quaternion inverseYawCorrection = Quaternion.Inverse(GetTrafficYawCorrection());
            Vector3 mapLocalPosition = inverseYawCorrection * source.position - originOffset;
            Quaternion mapLocalRotation = inverseYawCorrection * source.rotation;

            return new EgoData
            {
                id = egoId,
                blueprint = blueprint,
                color = color,
                location = new LocationData
                {
                    x = mapLocalPosition.z,
                    y = mapLocalPosition.x,
                    z = mapLocalPosition.y
                },
                yaw = Mathf.DeltaAngle(0f, mapLocalRotation.eulerAngles.y)
            };
        }

        private Quaternion GetTrafficYawCorrection()
        {
            float clientMapYawDelta = GetClientMapYawDeltaDegrees();
            return Quaternion.Euler(0f, clientMapYawDelta, 0f);
        }

        private float GetClientMapYawDeltaDegrees()
        {
            if (!applyClientMapYawCorrection)
                return 0f;

            Module mapModule = ResolveModule();
            if (mapModule != null && mapModule.HasMapRotationState)
                return Mathf.DeltaAngle(uncorrectedMapYawDegrees, mapModule.CurrentMapRotationEuler.y);

            Transform root = ResolveMapRoot();
            if (root == null)
                return 0f;

            return Mathf.DeltaAngle(uncorrectedMapYawDegrees, root.eulerAngles.y);
        }

        private Transform ResolveEgoTransform()
        {
            if (egoTransform != null)
                return egoTransform;

            Module mapModule = ResolveModule();
            if (mapModule != null && mapModule.ActiveAgent != null)
            {
                egoTransform = mapModule.ActiveAgent.transform;
                return egoTransform;
            }

            DynamicAgent dynamicAgent = FindFirstObjectByType<DynamicAgent>();
            if (dynamicAgent != null)
                egoTransform = dynamicAgent.transform;

            return egoTransform;
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

        private void ApplyEnvironmentOverrides()
        {
            string envHost = Environment.GetEnvironmentVariable("UB_EGO_BRIDGE_HOST");
            if (!string.IsNullOrWhiteSpace(envHost))
                bridgeHost = envHost;

            string envPort = Environment.GetEnvironmentVariable("UB_EGO_BRIDGE_PORT");
            if (int.TryParse(envPort, out int parsedPort))
                bridgePort = parsedPort;
        }
    }
}
