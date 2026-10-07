// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Net;
using System.Net.Sockets;

namespace NOMAD.MissionPlanner
{
    internal sealed class LocalConsumer : IDisposable
    {
        internal readonly ConsumerConfig Config;
        private UdpClient _socket;
        private IPEndPoint _client;
        private MavlinkFrameParser _parser;
        private double _lastClientPacket = RouterClock.Unset;
        private readonly RouterClock _clock;
        internal LocalConsumer(ConsumerConfig config, RouterClock clock) { Config = config; _clock = clock; }
        internal void Open()
        {
            _socket = PhysicalLink.OpenUdp(IPAddress.Loopback, Config.RouterPort);
            _parser = new MavlinkFrameParser();
            _client = Config.ClientPort == 0 ? null : new IPEndPoint(IPAddress.Loopback, Config.ClientPort);
        }
        internal void Poll(Action<MavlinkFrame> receive)
        {
            for (int i = 0; i < 64 && _socket.Available > 0; i++)
            {
                IPEndPoint sender = null;
                var bytes = _socket.Receive(ref sender);
                if (!IPAddress.IsLoopback(sender.Address) || sender.Port == Config.RouterPort)
                {
                    continue;
                }
                if (_client != null && !_client.Equals(sender))
                {
                    if (Config.ClientPort != 0 || (_clock.Seconds() - _lastClientPacket) < 3)
                    {
                        continue;
                    }
                    _parser = new MavlinkFrameParser();
                }
                _client = sender;
                _lastClientPacket = _clock.Seconds();
                _parser.Push(bytes, bytes.Length, receive);
            }
        }
        internal void Send(byte[] bytes)
        {
            if (_client == null)
            {
                return;
            }
            try { _socket.Send(bytes, bytes.Length, _client); }
            catch (SocketException)
            {
                /* A missing local consumer must not interrupt other deliveries. */
            }
        }
        public void Dispose() { _socket?.Close(); _socket = null; _client = null; }
    }
}
