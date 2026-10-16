using System;
using System.Net;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    [Serializable]
    public sealed class ServerSettings
    {
        public string host = "127.0.0.1";
        public int port = 6390;
        public string channel = "carla:telemetry";
        [NonSerialized] public string password = "";

        const string PreferenceKey = "UBMR.ServerConnection";

        public bool TryValidate(out string error)
        {
            error = null;
            if (string.IsNullOrWhiteSpace(host) || host != host.Trim() ||
                (!IPAddress.TryParse(host, out _) && Uri.CheckHostName(host) != UriHostNameType.Dns))
                error = "Enter an IP address or hostname, without a URL or port.";
            else if (port < 1 || port > 65535)
                error = "Port must be between 1 and 65535.";
            else if (string.IsNullOrWhiteSpace(channel) || channel.Length > 256 ||
                channel.IndexOfAny(new[] { '\r', '\n', '\0' }) >= 0)
                error = "Enter a channel name of 1–256 characters.";
            return error == null;
        }

        public ServerSettings Copy() => new() { host = host, port = port, channel = channel, password = password };

        public static ServerSettings Load()
        {
            try
            {
                var settings = JsonUtility.FromJson<ServerSettings>(PlayerPrefs.GetString(PreferenceKey, "{}"));
                if (settings != null && settings.TryValidate(out _)) return settings;
            }
            catch (ArgumentException) { }
            return new ServerSettings();
        }

        public void Save()
        {
            // Keep credentials in memory only; PlayerPrefs is not an encrypted credential store.
            PlayerPrefs.SetString(PreferenceKey, JsonUtility.ToJson(this));
            PlayerPrefs.Save();
        }
    }
}
