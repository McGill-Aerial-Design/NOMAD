// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    // Owned by the router worker. Reconnect never waits for a TCP connection.
    internal sealed class PhysicalLink : IDisposable
    {
        internal readonly LinkConfig Config;
        internal readonly LinkSourceStats Stats;
        internal MavlinkFrameParser Parser = new MavlinkFrameParser();
        internal readonly System.Collections.Generic.Dictionary<int, byte> Sequences =
            new System.Collections.Generic.Dictionary<int, byte>();
        internal long Seen, Lost, PreviousBytes;
        internal double HealthySince = RouterClock.Unset, LastAttempt = RouterClock.Unset;
        internal double LastPacket = RouterClock.Unset, LastHeartbeat = RouterClock.Unset;
        private UdpClient _udp;
        private TcpClient _tcp;
        private Task _connecting;
        private Task<IPAddress[]> _resolving;
        private readonly System.Collections.Generic.HashSet<int> _localPorts;
        private SerialPort _serial;
        private IPEndPoint _remote;
        private readonly byte[] _buffer = new byte[8192];

        internal PhysicalLink(LinkConfig config, System.Collections.Generic.HashSet<int> localPorts)
        {
            Config = config;
            _localPorts = localPorts;
            Stats = new LinkSourceStats { Type = config.Id, Name = config.Name ?? config.Id,
                Transport = config.Transport, Enabled = config.Enabled,
                Endpoint = config.Transport == "COM" ? config.Device : config.Transport + ":" + config.Port };
        }

        internal void Open(double now)
        {
            LastAttempt = now;
            Dispose();
            Parser = new MavlinkFrameParser();
            Sequences.Clear();
            if (Config.Transport == "UDP")
            {
                _udp = OpenUdp(IPAddress.Parse(Config.BindAddress), Config.Port);
                if (!string.IsNullOrEmpty(Config.RemoteHost))
                {
                    if (IPAddress.TryParse(Config.RemoteHost, out var address))
                    {
                        SetRemote(address);
                    }
                    else
                    {
                        _resolving = Dns.GetHostAddressesAsync(Config.RemoteHost);
                    }
                }
                Stats.IsOpen = _resolving == null;
            }
            else if (Config.Transport == "TCP")
            {
                _tcp = new TcpClient { NoDelay = true };
                _connecting = _tcp.ConnectAsync(Config.RemoteHost, Config.Port);
            }
            else
            {
                _serial = new SerialPort(Config.Device, Config.BaudRate) { ReadTimeout = 20, WriteTimeout = 100 };
                _serial.Open();
                Stats.IsOpen = true;
            }
        }

        internal static UdpClient OpenUdp(IPAddress address, int port)
        {
            var socket = new UdpClient();
            try
            {
                socket.ExclusiveAddressUse = true;
                socket.Client.Bind(new IPEndPoint(address, port));
                socket.Client.ReceiveBufferSize = 1024 * 1024;
                socket.Client.SendTimeout = 100;
                if (Environment.OSVersion.Platform == PlatformID.Win32NT)
                {
                    socket.Client.IOControl(-1744830452, new byte[4], null);
                }
                return socket;
            }
            catch
            {
                socket.Close(); throw;
            }
        }

        internal void Poll(Action<byte[], int> receive, double now)
        {
            if (!ResolveRemote(now))
            {
                return;
            }
            if (_connecting != null)
            {
                if (!_connecting.IsCompleted && (now - LastAttempt) < 2)
                {
                    return;
                }
                if (!_connecting.IsCompleted)
                {
                    throw new TimeoutException("TCP connect timeout");
                }
                _connecting.GetAwaiter().GetResult();
                _connecting = null;
                _tcp.SendTimeout = 100;
                Stats.IsOpen = true;
            }
            for (int i = 0; i < 64 && Stats.IsOpen; i++)
            {
                if (_udp != null)
                {
                    if (_udp.Available == 0)
                    {
                        return;
                    }
                    IPEndPoint sender = null;
                    var bytes = _udp.Receive(ref sender);
                    if (_remote != null && !_remote.Equals(sender))
                    {
                        continue;
                    }
                    Stats.LastRemote = sender;
                    receive(bytes, bytes.Length);
                }
                else
                {
                    int available = _tcp != null ? _tcp.Available : _serial.BytesToRead;
                    if (available == 0)
                    {
                        if (_tcp != null && _tcp.Client.Poll(0, SelectMode.SelectRead))
                        {
                            throw new System.IO.IOException("TCP peer closed");
                        }
                        return;
                    }
                    int count = Math.Min(available, _buffer.Length);
                    int read = _tcp != null ? _tcp.GetStream().Read(_buffer, 0, count) : _serial.Read(_buffer, 0, count);
                    receive(_buffer, read);
                }
            }
        }

        private bool ResolveRemote(double now)
        {
            if (_resolving == null)
            {
                return true;
            }
            if (!_resolving.IsCompleted)
            {
                if ((now - LastAttempt) >= 2)
                {
                    throw new TimeoutException("UDP peer lookup timeout");
                }
                return false;
            }
            var address = _resolving.GetAwaiter().GetResult()
                .FirstOrDefault(ip => ip.AddressFamily == AddressFamily.InterNetwork);
            if (address == null)
            {
                throw new ArgumentException("UDP peer must resolve to IPv4");
            }
            SetRemote(address);
            _resolving = null;
            Stats.IsOpen = true;
            return true;
        }

        private void SetRemote(IPAddress address)
        {
            LocalAddressGuard.ValidateRemote(address, Config.RemotePort, _localPorts);
            _remote = new IPEndPoint(address, Config.RemotePort);
        }

        internal void Send(byte[] bytes)
        {
            if (!Stats.IsOpen)
            {
                return;
            }
            if (_udp != null)
            {
                var remote = _remote ?? Stats.LastRemote;
                if (remote == null)
                {
                    return;
                }
                _udp.Send(bytes, bytes.Length, remote);
            }
            else if (_tcp != null)
            {
                _tcp.GetStream().Write(bytes, 0, bytes.Length);
            }
            else
            {
                _serial.Write(bytes, 0, bytes.Length);
            }
            Stats.BytesSentOutbound += bytes.Length;
        }

        internal bool CanAnnounce => Stats.IsOpen && LastPacket == RouterClock.Unset &&
            (Config.Transport != "UDP" || _remote != null);

        internal bool Opening => _connecting != null || _resolving != null;
        public void Dispose()
        {
            _udp?.Close(); _tcp?.Close(); _serial?.Dispose();
            _udp = null; _tcp = null; _serial = null; _connecting = null; _resolving = null; _remote = null;
            Stats.IsOpen = false; Stats.IsConnected = false; Stats.Health = LinkHealth.Disconnected;
            Stats.LastRemote = null; Stats.LastPacketTime = DateTime.MinValue;
            Stats.LastHeartbeatTime = DateTime.MinValue; HealthySince = RouterClock.Unset;
            LastPacket = LastHeartbeat = RouterClock.Unset;
        }
    }
}
