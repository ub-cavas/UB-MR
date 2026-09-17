using System;
using System.Globalization;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace UB_MR.Redis_Networking
{
    // Small RESP2 transport for AUTH, SUBSCRIBE, PUBLISH and PING. No Unity APIs on worker threads.
    // Protocol: https://redis.io/docs/latest/develop/reference/protocol-spec/
    internal sealed class RedisWire : IDisposable
    {
        readonly TcpClient client = new();
        NetworkStream stream;
        const int MaxBulkBytes = 4 * 1024 * 1024;

        public void Open(string host, int port, string password, CancellationToken cancellation)
        {
            client.NoDelay = true;
            client.ReceiveTimeout = client.SendTimeout = 3000;
            if (!client.ConnectAsync(host, port).Wait(3000, cancellation))
                throw new TimeoutException("Connection timed out.");
            cancellation.ThrowIfCancellationRequested();
            stream = client.GetStream();
            if (!string.IsNullOrEmpty(password))
            {
                Write("AUTH", password);
                if (!Equals(Read(), "OK")) throw new IOException("Authentication failed.");
            }
        }

        public void Write(params string[] arguments)
        {
            using var buffer = new MemoryStream();
            void Put(string value)
            {
                byte[] bytes = Encoding.UTF8.GetBytes(value);
                buffer.Write(bytes, 0, bytes.Length);
            }
            Put("*" + arguments.Length.ToString(CultureInfo.InvariantCulture) + "\r\n");
            foreach (string argument in arguments)
            {
                byte[] bytes = Encoding.UTF8.GetBytes(argument);
                Put("$" + bytes.Length.ToString(CultureInfo.InvariantCulture) + "\r\n");
                buffer.Write(bytes, 0, bytes.Length);
                Put("\r\n");
            }
            byte[] command = buffer.ToArray();
            stream.Write(command, 0, command.Length);
        }

        public object Read() => ReadValue(0);

        public bool WaitForData() => client.Client.Poll(1000000, SelectMode.SelectRead);

        object ReadValue(int depth)
        {
            if (depth > 4) throw new IOException("Invalid Redis response nesting.");
            int marker = stream.ReadByte();
            if (marker < 0) throw new EndOfStreamException();
            string line = ReadLine();
            switch (marker)
            {
                case '+': return line;
                case '-':
                    // Do not echo server-controlled messages or credentials into logs/UI.
                    if (line.StartsWith("WRONGPASS") || line.StartsWith("NOAUTH"))
                        throw new UnauthorizedAccessException("Authentication failed. Check the password.");
                    if (line.StartsWith("NOPERM"))
                        throw new UnauthorizedAccessException("Server denied access to this channel.");
                    throw new IOException("Server rejected the Redis command.");
                case ':': return long.Parse(line, CultureInfo.InvariantCulture);
                case '$':
                    int length = int.Parse(line, CultureInfo.InvariantCulture);
                    if (length == -1) return null;
                    if (length < 0 || length > MaxBulkBytes) throw new IOException("Invalid Redis payload size.");
                    byte[] bytes = new byte[length];
                    for (int offset = 0; offset < length;)
                    {
                        int count = stream.Read(bytes, offset, length - offset);
                        if (count == 0) throw new EndOfStreamException();
                        offset += count;
                    }
                    if (stream.ReadByte() != '\r' || stream.ReadByte() != '\n')
                        throw new IOException("Invalid Redis payload terminator.");
                    return Encoding.UTF8.GetString(bytes);
                case '*':
                    int size = int.Parse(line, CultureInfo.InvariantCulture);
                    if (size == -1) return null;
                    if (size < 0 || size > 1024) throw new IOException("Invalid Redis array size.");
                    object[] values = new object[size];
                    for (int i = 0; i < size; i++) values[i] = ReadValue(depth + 1);
                    return values;
                default: throw new IOException("Unsupported Redis response.");
            }
        }

        string ReadLine()
        {
            using var bytes = new MemoryStream();
            for (int i = 0; i < 1024; i++)
            {
                int value = stream.ReadByte();
                if (value < 0) throw new EndOfStreamException();
                if (value == '\r')
                {
                    if (stream.ReadByte() != '\n') throw new IOException("Invalid Redis line ending.");
                    return Encoding.UTF8.GetString(bytes.ToArray());
                }
                bytes.WriteByte((byte)value);
            }
            throw new IOException("Redis response header is too long.");
        }

        public void Dispose() => client.Close();
    }
}
