// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    public sealed partial class RouterManagementServer
    {
        private sealed partial class ManagementSession
        {
            private readonly RouterManagementServer _server;
            private readonly TcpClient _client;
            private readonly NetworkStream _stream;
            private readonly int _maxQueuedMessages;
            private readonly object _gate = new object();
            private readonly Queue<OutboundMessage> _outgoing = new Queue<OutboundMessage>();
            private readonly HashSet<string> _pendingEventKinds = new HashSet<string>(StringComparer.Ordinal);
            private readonly AutoResetEvent _outgoingReady = new AutoResetEvent(false);
            private Thread _reader;
            private Thread _writer;
            private bool _subscribed;
            private bool _stopping;
            private bool _disposed;

            internal ManagementSession(RouterManagementServer server, TcpClient client, int maxQueuedMessages)
            {
                _server = server;
                _client = client;
                _stream = client.GetStream();
                _stream.ReadTimeout = 1000;
                _stream.WriteTimeout = 1000;
                _maxQueuedMessages = maxQueuedMessages;
            }

            internal bool IsSubscribed
            {
                get { lock (_gate) { return _subscribed && !_stopping; } }
            }

            internal void SetSubscribed(bool subscribed)
            {
                lock (_gate)
                {
                    _subscribed = subscribed;
                }
            }

            internal void Start()
            {
                lock (_gate)
                {
                    if (_disposed)
                    {
                        return;
                    }
                    _writer = new Thread(WriteLoop)
                    {
                        IsBackground = true,
                        Name = "NOMAD router management writer",
                    };
                    _reader = new Thread(ReadLoop)
                    {
                        IsBackground = true,
                        Name = "NOMAD router management client",
                    };
                    _writer.Start();
                    _reader.Start();
                }
            }

            internal void EnqueueResponse(byte[] message)
            {
                Enqueue(new OutboundMessage(message, null));
            }

            internal void EnqueueEvent(string eventName, byte[] message)
            {
                Enqueue(new OutboundMessage(message, eventName));
            }

            internal void Stop()
            {
                Thread reader;
                Thread writer;
                lock (_gate)
                {
                    if (_disposed)
                    {
                        return;
                    }
                    _disposed = true;
                    _stopping = true;
                    _subscribed = false;
                    _outgoing.Clear();
                    _pendingEventKinds.Clear();
                    reader = _reader;
                    writer = _writer;
                }
                _outgoingReady.Set();
                try { _client.Close(); } catch { }
                if (reader != null && reader != Thread.CurrentThread) reader.Join();
                if (writer != null && writer != Thread.CurrentThread) writer.Join();
                _outgoingReady.Dispose();
            }

            private void ReadLoop()
            {
                try
                {
                    while (!IsStopping())
                    {
                        string line;
                        try
                        {
                            line = RouterManagementProtocol.ReadLine(_stream);
                        }
                        catch (IOException)
                        {
                            if (!IsStopping()) continue;
                            return;
                        }
                        catch (FormatException ex)
                        {
                            DrainOversizedLine();
                            SendImmediate(RouterManagementProtocol.EncodeLine(
                                RouterManagementProtocol.Error(null, "malformed_request", ex.Message)));
                            return;
                        }

                        if (line == null)
                        {
                            return;
                        }

                        EnqueueResponse(_server.HandleRequest(this, line));
                    }
                }
                catch (Exception)
                {
                    // A management client may disappear at any time. Routing is independent.
                }
                finally
                {
                    Stop();
                    _server.RemoveSession(this);
                }
            }

            private void WriteLoop()
            {
                try
                {
                    while (!IsStopping())
                    {
                        OutboundMessage next = null;
                        lock (_gate)
                        {
                            if (_outgoing.Count > 0)
                            {
                                next = _outgoing.Dequeue();
                                if (next.EventKind != null)
                                {
                                    _pendingEventKinds.Remove(next.EventKind);
                                }
                            }
                        }
                        if (next == null)
                        {
                            _outgoingReady.WaitOne(100);
                            continue;
                        }

                        _stream.Write(next.Payload, 0, next.Payload.Length);
                    }
                }
                catch (Exception)
                {
                }
                finally
                {
                    Stop();
                    _server.RemoveSession(this);
                }
            }

            private void Enqueue(OutboundMessage message)
            {
                lock (_gate)
                {
                    if (_stopping)
                    {
                        return;
                    }
                    if (message.EventKind != null && !_pendingEventKinds.Add(message.EventKind))
                    {
                        return;
                    }
                    if (_outgoing.Count >= _maxQueuedMessages && !DropOldestEvent())
                    {
                        if (message.EventKind != null)
                        {
                            _pendingEventKinds.Remove(message.EventKind);
                        }
                        return;
                    }
                    _outgoing.Enqueue(message);
                    _outgoingReady.Set();
                }
            }

            private bool DropOldestEvent()
            {
                var items = _outgoing.ToArray();
                var index = Array.FindIndex(items, item => item.EventKind != null);
                if (index < 0)
                {
                    return false;
                }

                _outgoing.Clear();
                for (var i = 0; i < items.Length; i++)
                {
                    if (i == index)
                    {
                        _pendingEventKinds.Remove(items[i].EventKind);
                        continue;
                    }
                    _outgoing.Enqueue(items[i]);
                }
                return true;
            }

            private void SendImmediate(byte[] message)
            {
                try
                {
                    _stream.Write(message, 0, message.Length);
                }
                catch (Exception)
                {
                }
            }

            private void DrainOversizedLine()
            {
                for (var i = 0; i < RouterManagementProtocol.MaxMessageBytes * 2; i++)
                {
                    try
                    {
                        var value = _stream.ReadByte();
                        if (value < 0 || value == '\n')
                        {
                            return;
                        }
                    }
                    catch (IOException)
                    {
                        return;
                    }
                }
            }

            private bool IsStopping()
            {
                lock (_gate) { return _stopping; }
            }

            private sealed class OutboundMessage
            {
                internal readonly byte[] Payload;
                internal readonly string EventKind;

                internal OutboundMessage(byte[] payload, string eventKind)
                {
                    Payload = payload;
                    EventKind = eventKind;
                }
            }
        }
    }
}
