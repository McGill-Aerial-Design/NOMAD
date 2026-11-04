// SPDX-License-Identifier: Apache-2.0
using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace NOMAD.Simulation
{
    internal sealed class RtpBridge : IDisposable
    {
        private readonly TcpClient _tcp = new TcpClient { NoDelay = true };
        private readonly UdpClient _udp = new UdpClient(AddressFamily.InterNetwork);
        private readonly Thread _reader;
        private volatile bool _stopping;
        private volatile Exception _error;
        internal Exception Error => _error;
        internal int Packets;

        private readonly int _udpPort;

        internal RtpBridge(IPAddress host, int tcpPort, int udpPort)
        {
            if (tcpPort < 1 || tcpPort > 65535 || udpPort < 1 || udpPort > 65535)
            { _tcp.Close(); _udp.Close(); throw new ArgumentOutOfRangeException("Review simulator video ports."); }
            _udpPort = udpPort;
            try
            {
                var connect = _tcp.ConnectAsync(host, tcpPort);
                if (!connect.Wait(5000)) { throw new TimeoutException("RTP relay connect deadline"); }
                connect.GetAwaiter().GetResult();
                _reader = new Thread(Read) { IsBackground = true, Name = "NOMAD encoded RTP bridge" };
                _reader.Start();
            }
            catch { _tcp.Close(); _udp.Close(); throw; }
        }

        private void Read()
        {
            try
            {
                var stream = _tcp.GetStream();
                var header = new byte[2];
                while (!_stopping)
                {
                    Exact(stream, header);
                    int length = (header[0] << 8) | header[1];
                    if (length < 12) { throw new InvalidDataException("Invalid RTP relay length"); }
                    var packet = new byte[length]; Exact(stream, packet);
                    if (packet[0] >> 6 != 2 || (packet[1] & 127) != 96)
                    { throw new InvalidDataException("Invalid RTP relay header"); }
                    try { _udp.Send(packet, packet.Length, new IPEndPoint(IPAddress.Loopback, _udpPort)); }
                    catch (SocketException error) when (error.SocketErrorCode == SocketError.ConnectionReset) { }
                    Interlocked.Increment(ref Packets);
                }
            }
            catch (Exception error) { if (!_stopping) { _error = error; } }
        }

        private static void Exact(Stream stream, byte[] data)
        {
            int offset = 0;
            while (offset < data.Length)
            {
                int read = stream.Read(data, offset, data.Length - offset);
                if (read == 0) { throw new EndOfStreamException("RTP relay closed"); }
                offset += read;
            }
        }

        public void Dispose()
        {
            _stopping = true; _tcp.Close(); _udp.Close();
            if (!_reader.Join(2000)) { throw new TimeoutException("RTP bridge reader did not stop"); }
        }
    }
}
