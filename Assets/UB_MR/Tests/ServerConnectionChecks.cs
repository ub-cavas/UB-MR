using System;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using UB_MR.Redis_Networking;
using UnityEngine;

namespace CAVAS.UB_MR.Tests
{
    // Requires an isolated Redis instance on 127.0.0.1:6392 with password ubmr-test.
    // Never run against the simulation Redis: the reconnect test closes its subscribers.
    public static class ServerConnectionChecks
    {
        public static async Task Run()
        {
            var settings = new ServerSettings { host = "127.0.0.1", port = 6392, password = "ubmr-test", channel = "ubmr:test:雪" };
            Check(settings.TryValidate(out _), "Valid endpoint rejected.");
            foreach (string host in new[] { "", "https://localhost", "localhost:6390", "bad host" })
            {
                var invalid = settings.Copy(); invalid.host = host;
                Check(!invalid.TryValidate(out _), "Invalid host accepted: " + host);
            }
            foreach (int port in new[] { 0, -1, 65536 })
            {
                var invalid = settings.Copy(); invalid.port = port;
                Check(!invalid.TryValidate(out _), "Invalid port accepted.");
            }
            Check(!JsonUtility.ToJson(settings).Contains(settings.password), "Password was serialized.");
            using var admin = new RedisWire();
            admin.Open(settings.host, settings.port, settings.password, CancellationToken.None);
            var session = new RedisSession(settings);
            try
            {
                await Until(() => session.Connected, "Connection did not complete.");
                string traffic = "{\"type\":2,\"vehicles\":[{\"id\":\"雪\",\"location\":{\"x\":1,\"y\":2,\"z\":3},\"yaw\":40}]}";
                admin.Write("PUBLISH", settings.channel, traffic); admin.Read();
                string received = null;
                await Until(() => (received = session.TakeTraffic()) != null, "No traffic received.");
                Check(received == traffic, "Unicode traffic changed on the wire.");
                // Other message types must not overwrite the traffic snapshot.
                admin.Write("PUBLISH", settings.channel, "{\"type\":3,\"ego\":{}}"); admin.Read();
                await Task.Delay(100);
                Check(session.TakeTraffic() == null, "Ego was treated as traffic.");
                admin.Write("PUBLISH", settings.channel, "not-json"); admin.Read();
                admin.Write("PUBLISH", settings.channel, traffic); admin.Read();
                await Until(() => session.TakeTraffic() != null, "Malformed message killed subscription.");

                using var observer = new RedisWire();
                observer.Open(settings.host, settings.port, settings.password, CancellationToken.None);
                observer.Write("SUBSCRIBE", settings.channel); observer.Read();
                string ego = "{\"type\":3,\"ego\":{\"id\":\"ego-check\",\"location\":{\"x\":1,\"y\":2,\"z\":3}}}";
                session.Publish(ego);
                var response = await Task.Run(() => observer.Read() as object[]);
                Check(response != null && Equals(response[2], ego), "Ego publication failed.");
                // Idle subscriptions need heartbeats, not repeated reconnects.
                await Task.Delay(3500);
                Check(session.Connected, "Idle connection was lost.");
                admin.Write("CLIENT", "KILL", "TYPE", "pubsub"); admin.Read();
                await Until(() => !session.Connected, "Dropped subscription was not detected.");
                await Until(() => session.Connected, "Automatic reconnect failed.");
                admin.Write("PUBLISH", settings.channel, traffic); admin.Read();
                await Until(() => session.TakeTraffic() != null, "Traffic did not resume after reconnect.");
            }
            finally
            {
                session.Dispose();
                await Until(() => session.Completion.IsCompleted, "Connection workers did not stop.");
                Check(!session.Completion.IsFaulted, "Worker failed during shutdown.");
            }
            var wrongPassword = settings.Copy(); wrongPassword.password = "incorrect";
            var denied = new RedisSession(wrongPassword);
            try
            {
                await Until(() => denied.Status.Contains("Authentication failed"), "Wrong password was not reported.");
                Check(!denied.Connected, "Wrong password connected.");
            }
            finally { denied.Dispose(); await denied.Completion; }
            var unreachable = settings.Copy(); unreachable.port = 1;
            var failed = new RedisSession(unreachable);
            try { await Until(() => failed.Status.Contains("Unable to reach"), "Unreachable endpoint was not reported."); }
            finally { failed.Dispose(); await failed.Completion; }
        }

        static async Task Until(Func<bool> condition, string message)
        {
            DateTime limit = DateTime.UtcNow.AddSeconds(8);
            while (!condition())
            {
                if (DateTime.UtcNow > limit) throw new Exception(message);
                await Task.Delay(25);
            }
        }
        static void Check(bool value, string message) { if (!value) throw new Exception(message); }
    }
}
