using System;
using System.Globalization;
using System.IO;
using System.Text.RegularExpressions;
using UnityEngine;

namespace CAVAS.UB_MR.Telemetry
{
    // One recording per player/Editor Play session, including menu and map changes.
    public sealed class ExperimentRecording : MonoBehaviour
    {
        ExperimentLog log;

        [Serializable]
        sealed class ExperimentInfo
        {
            public int schema_version = 1;
            public string client_type = "ub-mr", controller_type, client_id, hostname, started_utc;
        }

#if UNITY_EDITOR
        // Local to this user and project; never included in player builds.
        public static bool EditorLoggingEnabled
        {
            get => UnityEditor.EditorPrefs.GetBool("UB-MR.ExperimentRecording." + Application.dataPath, false);
            set => UnityEditor.EditorPrefs.SetBool("UB-MR.ExperimentRecording." + Application.dataPath, value);
        }
#endif

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void Reset() { ExperimentLog.Active?.Dispose(); ExperimentLog.Active = null; }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        static void StartRecording()
        {
            if ((Environment.GetEnvironmentVariable("UB_EXPERIMENT_ENABLED")
                ?? Environment.GetEnvironmentVariable("UB_MR_METRICS_ENABLED")) == "0") return;
#if UNITY_EDITOR
            if (!EditorLoggingEnabled) return;
#endif
            var root = new GameObject("Experiment recording");
            DontDestroyOnLoad(root);
            root.AddComponent<ExperimentRecording>().Initialize();
        }
        void Initialize()
        {
            try
            {
                string directory = Environment.GetEnvironmentVariable("UB_EXPERIMENT_RUN_DIR")
                    ?? Environment.GetEnvironmentVariable("UB_MR_METRICS_RUN_DIR");
                if (string.IsNullOrWhiteSpace(directory))
                {
                    string parent = Environment.GetEnvironmentVariable("UB_EXPERIMENT_DIR")
                        ?? Environment.GetEnvironmentVariable("UB_MR_METRICS_DIR");
                    if (string.IsNullOrWhiteSpace(parent)) parent = Path.Combine(Application.persistentDataPath, "Experiments");
                    string hostName = Environment.GetEnvironmentVariable("UB_EXPERIMENT_HOST_NAME")
                        ?? Environment.GetEnvironmentVariable("UB_MR_HOST_NAME");
                    if (string.IsNullOrWhiteSpace(hostName)) hostName = Environment.MachineName;
                    hostName = Regex.Replace(hostName, "[^A-Za-z0-9._-]", "_");
                    if (hostName.Length > 63) hostName = hostName.Substring(0, 63);
                    directory = Path.Combine(parent, "run-" + DateTime.UtcNow.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture)
                        + "Z-" + hostName + "-" + Guid.NewGuid().ToString("N"));
                }
                log = new ExperimentLog(directory);
                File.WriteAllText(Path.Combine(directory, "experiment.json"), JsonUtility.ToJson(new ExperimentInfo
                {
                    controller_type = Environment.GetEnvironmentVariable("UB_EXPERIMENT_CONTROLLER") ?? "ros",
                    client_id = Environment.GetEnvironmentVariable("UB_EXPERIMENT_CLIENT_ID") ?? Guid.NewGuid().ToString("N"),
                    hostname = Environment.GetEnvironmentVariable("UB_EXPERIMENT_HOST_NAME")
                        ?? Environment.GetEnvironmentVariable("UB_MR_HOST_NAME") ?? Environment.MachineName,
                    started_utc = DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture)
                }, true));
                ExperimentLog.Active = log;
                Debug.Log("Experiment metrics: " + log.DirectoryPath);
            }
            catch (Exception failure)
            {
                log?.Dispose(); log = null;
                Debug.LogWarning("Experiment recording could not start: " + failure.Message);
            }
        }
        void Update()
        {
            if (log?.Error == null) return;
            Debug.LogError("Experiment recording failed: " + log.Error);
            StopRecording();
        }
        void StopRecording()
        {
            if (log == null) return;
            if (ExperimentLog.Active == log) ExperimentLog.Active = null;
            log.Dispose();
            if (log.Error != null) Debug.LogError("Experiment recording error: " + log.Error);
            if (log.DroppedRows > 0) Debug.LogWarning("Experiment recording dropped rows: " + log.DroppedRows);
            log = null;
        }
        void OnApplicationQuit() => StopRecording();
        void OnDestroy() => StopRecording();
    }
}
