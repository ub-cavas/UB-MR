using System;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace CAVAS.UB_MR.Tests
{
    public static class ServerConnectionMenuChecks
    {
        [MenuItem("UB-MR/Tests/Server connection checks")]
        public static async void Run()
        {
            try
            {
                await ServerConnectionChecks.Run();
                const string result = "Server connection checks PASSED: validation, password exclusion, Unicode traffic, ego publishing, invalid messages, heartbeat, reconnect, authentication failures, unreachable endpoint, shutdown.";
                Debug.Log(result);
                File.WriteAllText("/tmp/ubmr-server-checks.txt", result);
                if (Application.isBatchMode) EditorApplication.Exit(0);
            }
            catch (Exception error)
            {
                Debug.LogException(error);
                File.WriteAllText("/tmp/ubmr-server-checks.txt", error.ToString());
                if (Application.isBatchMode) EditorApplication.Exit(1);
            }
        }
    }
}
