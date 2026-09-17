using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using CAVAS.UB_MR.Telemetry;
using UB_MR.Redis_Networking;

namespace CAVAS.UB_MR.Tests
{
    // Isolated RESP fixture on an ephemeral loopback port. Never touches simulation Redis.
    public static class ResourceNetworkChecks
    {
        static void Check(bool condition, string message) => ResourceTelemetryChecks.Check(condition, message);
        public static async Task Run()
        {
            using var server = new RespFixture();
            using var telemetry = new RedisTelemetry();
            using (var wire = new RedisWire(telemetry.Traffic))
            {
                wire.Open("127.0.0.1", server.Port, "", CancellationToken.None);
                wire.Write("PING"); Check(Equals(wire.Read(), "PONG"), "PING response mismatch");
                var count = telemetry.Traffic.Snapshot();
                Check(count.SentBytes == 14 && count.ReceivedBytes == 7, "RESP framing byte counts incorrect");
                wire.Write("ECHO", "雪"); Check(Equals(wire.Read(), "雪"), "UTF8 bulk response mismatch");
                count = telemetry.Traffic.Snapshot();
                Check(count.SentBytes == 14 + Encoding.UTF8.GetByteCount("*2\r\n$4\r\nECHO\r\n$3\r\n雪\r\n") &&
                    count.ReceivedBytes == 7 + Encoding.UTF8.GetByteCount("$3\r\n雪\r\n"), "UTF8 or fragmented-read accounting incorrect");
            }

            var settings = new ServerSettings { host = "127.0.0.1", port = server.Port, channel = "hud:test", password = "" };
            var session = new RedisSession(settings);
            try
            {
                await Until(() => session.Connected && session.Telemetry.Snapshot().RoundTrip.Available, "Connection/RTT missing");
                const string traffic = "{\"type\":2,\"vehicles\":[{\"id\":\"test\"}]}";
                using (var external = new RedisWire())
                {
                    external.Open(settings.host, settings.port, "", CancellationToken.None);
                    external.Write("PUBLISH", settings.channel, traffic); external.Read();
                }
                string received = null;
                await Until(() => (received = session.TakeTraffic()) != null, "Traffic subscription stopped working");
                Check(received == traffic, "Traffic payload changed");
                double initialSample = session.Telemetry.Snapshot().RoundTrip.SampleTime;
                int initialPings = server.Pings;
                // Continuous poses previously prevented the idle-only PING path from running.
                for (int i = 0; i < 120; i++)
                {
                    session.Publish("{\"type\":3,\"ego\":{\"id\":\"test\"}}");
                    await Task.Delay(20).ConfigureAwait(false);
                }
                var snapshot = session.Telemetry.Snapshot();
                Check(snapshot.RoundTrip.SampleTime > initialSample && server.Pings >= initialPings + 2,
                    "Continuous publishing starved RTT sampling");
                Check(snapshot.RoundTrip.LatestMs >= 20 && snapshot.RoundTrip.AverageMs >= 20, "RTT measured publication ACK instead of delayed PONG");
                Check(snapshot.Traffic.ReceivedBytes > 0 && snapshot.Traffic.SentBytes > 0, "Session did not aggregate wire counters");
                long previousBytes = snapshot.Traffic.ReceivedBytes;
                server.DropSubscriptions();
                await Until(() => !session.Connected, "Subscription drop not detected");
                Check(!session.Telemetry.Snapshot().RoundTrip.Available, "Disconnected session retained RTT");
                await Until(() => session.Connected && session.Telemetry.Snapshot().RoundTrip.Available, "Reconnect did not restore RTT");
                Check(session.Telemetry.Snapshot().Traffic.ReceivedBytes >= previousBytes, "Reconnect reset aggregate byte counters");
            }
            finally
            {
                session.Dispose();
                await Until(() => session.Completion.IsCompleted, "Redis workers did not stop");
                await session.Completion.ConfigureAwait(false);
                Check(!session.Telemetry.Snapshot().RoundTrip.Available, "Disposed session retained RTT");
            }
        }

        static async Task Until(Func<bool> condition, string message)
        {
            double deadline = MonotonicClock.Instance.Seconds + 8;
            while (!condition())
            {
                if (MonotonicClock.Instance.Seconds > deadline) throw new Exception(message);
                await Task.Delay(10).ConfigureAwait(false);
            }
        }

        sealed class RespFixture : IDisposable
        {
            sealed class Client
            {
                public TcpClient Socket;
                public NetworkStream Stream;
                public string Channel;
                public readonly object SendGate = new();
                public void Send(string text)
                {
                    byte[] data = Encoding.UTF8.GetBytes(text);
                    lock (SendGate)
                        for (int i = 0; i < data.Length; i += 2)
                            Stream.Write(data, i, Math.Min(2, data.Length - i));
                }
            }
            readonly TcpListener listener = new(IPAddress.Loopback, 0);
            readonly List<Client> clients = new();
            readonly List<Task> workers = new();
            readonly Task accept;
            readonly object gate = new();
            volatile bool disposed;
            int pings;
            public int Pings => Volatile.Read(ref pings);
            public int Port => ((IPEndPoint)listener.LocalEndpoint).Port;
            public RespFixture()
            {
                listener.Start();
                accept = Task.Run(async () =>
                {
                    try
                    {
                        while (!disposed)
                        {
                            var socket = await listener.AcceptTcpClientAsync().ConfigureAwait(false);
                            socket.NoDelay = true;
                            var client = new Client { Socket = socket, Stream = socket.GetStream() };
                            lock (gate)
                            {
                                if (disposed) { socket.Dispose(); return; }
                                clients.Add(client); workers.Add(Task.Run(() => Serve(client)));
                            }
                        }
                    }
                    catch (ObjectDisposedException) when (disposed) { }
                    catch (SocketException) when (disposed) { }
                });
            }
            void Serve(Client client)
            {
                try
                {
                    while (!disposed)
                    {
                        string[] command = ReadCommand(client.Stream);
                        switch (command[0])
                        {
                            case "PING":
                                Interlocked.Increment(ref pings); Thread.Sleep(30);
                                client.Send(client.Channel == null ? "+PONG\r\n" : "*2\r\n$4\r\npong\r\n$0\r\n\r\n"); break;
                            case "ECHO": client.Send(Bulk(command[1])); break;
                            case "SUBSCRIBE":
                                lock (gate) client.Channel = command[1];
                                client.Send("*3\r\n" + Bulk("subscribe") + Bulk(command[1]) + ":1\r\n"); break;
                            case "PUBLISH":
                                Client[] subscribers;
                                lock (gate) subscribers = clients.FindAll(c => c.Channel == command[1]).ToArray();
                                foreach (var subscriber in subscribers)
                                {
                                    try { subscriber.Send("*3\r\n" + Bulk("message") + Bulk(command[1]) + Bulk(command[2])); }
                                    catch (IOException) { }
                                    catch (ObjectDisposedException) { }
                                }
                                client.Send(":" + subscribers.Length + "\r\n"); break;
                            default: throw new IOException("Unexpected fixture command");
                        }
                    }
                }
                catch (IOException) { }
                catch (SocketException) { }
                catch (ObjectDisposedException) { }
                finally { lock (gate) clients.Remove(client); client.Socket.Dispose(); }
            }
            static string Bulk(string value) => "$" + Encoding.UTF8.GetByteCount(value) + "\r\n" + value + "\r\n";
            static string Line(Stream stream)
            {
                var result = new StringBuilder();
                for (int i = 0; i < 1000; i++)
                {
                    int value = stream.ReadByte(); if (value < 0) throw new EndOfStreamException();
                    if (value == '\r') { if (stream.ReadByte() != '\n') throw new IOException(); return result.ToString(); }
                    result.Append((char)value);
                }
                throw new IOException("Fixture line too long");
            }
            static string[] ReadCommand(Stream stream)
            {
                string header = Line(stream);
                if (!header.StartsWith("*")) throw new IOException();
                var result = new string[int.Parse(header.Substring(1))];
                for (int i = 0; i < result.Length; i++)
                {
                    int size = int.Parse(Line(stream).Substring(1)); var buffer = new byte[size];
                    for (int offset = 0; offset < size;)
                    { int count = stream.Read(buffer, offset, size - offset); if (count == 0) throw new EndOfStreamException(); offset += count; }
                    if (stream.ReadByte() != '\r' || stream.ReadByte() != '\n') throw new IOException();
                    result[i] = Encoding.UTF8.GetString(buffer);
                }
                return result;
            }
            public void DropSubscriptions()
            {
                lock (gate)
                    foreach (var client in clients) if (client.Channel != null) client.Socket.Dispose();
            }
            public void Dispose()
            {
                disposed = true; listener.Stop();
                lock (gate) foreach (var client in clients) client.Socket.Dispose();
                accept.GetAwaiter().GetResult();
                Task[] remaining; lock (gate) remaining = workers.ToArray();
                Task.WhenAll(remaining).GetAwaiter().GetResult();
            }
        }
    }
}
