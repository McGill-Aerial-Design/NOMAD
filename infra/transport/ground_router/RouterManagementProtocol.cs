// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Net.Sockets;
using System.Text;
using System.Web.Script.Serialization;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Versioned, router-only messages exchanged over the local management socket.
    /// This contract deliberately contains no MAVLink bytes or flight operations.
    /// </summary>
    public static class RouterManagementProtocol
    {
        public const string ProtocolName = "nomad-link-router";
        public const int Version = 1;
        public const int MaxMessageBytes = 64 * 1024;

        public static readonly string[] SupportedOperations =
        {
            "hello", "get_status", "get_links", "select_link", "set_auto", "subscribe", "ping"
        };

        private static readonly Encoding Utf8 = new UTF8Encoding(false, true);

        public static string Serialize(IDictionary<string, object> message)
        {
            if (message == null)
            {
                throw new ArgumentNullException(nameof(message));
            }

            var json = new JavaScriptSerializer
            {
                MaxJsonLength = MaxMessageBytes,
                RecursionLimit = 32,
            }.Serialize(message);
            if (Utf8.GetByteCount(json) > MaxMessageBytes)
            {
                throw new InvalidOperationException("Management message exceeds the maximum size.");
            }

            return json;
        }

        public static byte[] EncodeLine(IDictionary<string, object> message)
        {
            var json = Serialize(message) + "\n";
            var bytes = Utf8.GetBytes(json);
            if (bytes.Length > MaxMessageBytes)
            {
                throw new InvalidOperationException("Management message exceeds the maximum size.");
            }

            return bytes;
        }

        public static string ReadLine(NetworkStream stream)
        {
            var buffer = new byte[MaxMessageBytes];
            var count = 0;
            while (true)
            {
                var value = stream.ReadByte();
                if (value < 0)
                {
                    return count == 0 ? null : throw new FormatException("Management message ended before newline.");
                }

                if (value == '\n')
                {
                    if (count > 0 && buffer[count - 1] == '\r')
                    {
                        count--;
                    }

                    return Utf8.GetString(buffer, 0, count);
                }

                if (count >= MaxMessageBytes - 1)
                {
                    throw new FormatException("Management message is too large.");
                }

                buffer[count++] = (byte)value;
            }
        }

        public static Dictionary<string, object> Parse(string json)
        {
            if (string.IsNullOrWhiteSpace(json))
            {
                throw new FormatException("Management message is empty.");
            }

            if (Utf8.GetByteCount(json) > MaxMessageBytes)
            {
                throw new FormatException("Management message is too large.");
            }

            var value = new JavaScriptSerializer
            {
                MaxJsonLength = MaxMessageBytes,
                RecursionLimit = 32,
            }.DeserializeObject(json);
            var message = value as Dictionary<string, object>;
            if (message == null)
            {
                throw new FormatException("Management message must be a JSON object.");
            }

            return message;
        }

        public static bool TryGetCompatibleVersion(
            IDictionary<string, object> message,
            out string errorCode,
            out string error)
        {
            errorCode = null;
            error = null;
            var protocol = GetString(message, "protocol");
            if (!string.Equals(protocol, ProtocolName, StringComparison.Ordinal))
            {
                errorCode = "unsupported_protocol";
                error = "The request must identify the nomad-link-router protocol.";
                return false;
            }

            var version = GetInt(message, "version");
            if (!version.HasValue || version.Value != Version)
            {
                errorCode = "incompatible_version";
                error = "The request protocol version is not supported.";
                return false;
            }

            return true;
        }

        public static object GetValue(IDictionary<string, object> message, string key)
        {
            if (message != null && message.TryGetValue(key, out var value))
            {
                return value;
            }

            return null;
        }

        public static string GetString(IDictionary<string, object> message, string key)
        {
            var value = GetValue(message, key);
            return value == null ? null : Convert.ToString(value, CultureInfo.InvariantCulture);
        }

        public static int? GetInt(IDictionary<string, object> message, string key)
        {
            var value = GetValue(message, key);
            if (value == null)
            {
                return null;
            }

            try
            {
                return Convert.ToInt32(value, CultureInfo.InvariantCulture);
            }
            catch (Exception) when (value is IConvertible)
            {
                return null;
            }
        }

        public static IDictionary<string, object> GetObject(IDictionary<string, object> message, string key)
        {
            return GetValue(message, key) as IDictionary<string, object>;
        }

        public static IList GetArray(IDictionary<string, object> message, string key)
        {
            return GetValue(message, key) as IList;
        }

        public static bool TryReadStatus(
            IDictionary<string, object> message,
            out RouterStatusSnapshot status)
        {
            status = null;
            var source = GetObject(message, "status");
            if (source == null)
            {
                return false;
            }

            var result = new RouterStatusSnapshot
            {
                Running = string.Equals(GetString(source, "state"), "running", StringComparison.Ordinal),
                ConfiguredLinkCount = GetInt(source, "configuredLinkCount") ?? 0,
                ConnectedLinkCount = GetInt(source, "connectedLinkCount") ?? 0,
                ActiveLinkId = GetString(source, "activeLinkId") ?? "",
                ManualOverrideId = GetString(source, "manualOverrideId") ?? "",
                AutomaticFailoverEnabled = GetBool(source, "automaticFailoverEnabled"),
                PreferredLinkId = GetString(source, "preferredLinkId") ?? "",
                TimestampUtc = GetString(source, "timestampUtc") ?? "",
            };

            var links = GetArray(source, "links");
            if (links == null)
            {
                return false;
            }

            foreach (var item in links)
            {
                var link = item as IDictionary<string, object>;
                if (link == null)
                {
                    return false;
                }

                result.Links.Add(new RouterLinkStatusSnapshot
                {
                    StableId = GetString(link, "stableId") ?? "",
                    DisplayName = GetString(link, "displayName") ?? "",
                    TransportType = GetString(link, "transportType") ?? "",
                    Endpoint = GetString(link, "endpoint") ?? "",
                    Enabled = GetBool(link, "enabled"),
                    IsOpen = GetBool(link, "isOpen"),
                    IsConnected = GetBool(link, "isConnected"),
                    Health = GetString(link, "health") ?? "Disconnected",
                    LastPacketAgeMs = GetDouble(link, "lastPacketAgeMs"),
                    LastHeartbeatAgeMs = GetDouble(link, "lastHeartbeatAgeMs"),
                    PacketLossEstimate = GetDouble(link, "packetLossEstimate") ?? 0,
                    DataRateBytesPerSecond = GetDouble(link, "dataRateBytesPerSecond") ?? 0,
                    HeartbeatJitterMs = GetDouble(link, "heartbeatJitterMs") ?? 0,
                    Rssi = GetNullableInt(link, "rssi"),
                    RemoteRssi = GetNullableInt(link, "remoteRssi"),
                    HeartbeatCount = GetLong(link, "heartbeatCount") ?? 0,
                    ReceivedFrameCount = GetLong(link, "receivedFrameCount") ?? 0,
                    ForwardedFrameCount = GetLong(link, "forwardedFrameCount") ?? 0,
                    DuplicateFrameCount = GetLong(link, "duplicateFrameCount") ?? 0,
                });
            }

            status = result;
            return true;
        }

        public static Dictionary<string, object> HelloResponse(object id, string implementationVersion)
        {
            var response = Response(id, "hello", true);
            response["protocol"] = ProtocolName;
            response["version"] = Version;
            response["implementationVersion"] = implementationVersion ?? "unknown";
            response["operations"] = new List<string>(SupportedOperations);
            response["maxMessageBytes"] = MaxMessageBytes;
            return response;
        }

        public static Dictionary<string, object> StatusResponse(object id, RouterStatusSnapshot status)
        {
            var response = Response(id, "status", true);
            response["status"] = BuildStatus(status);
            return response;
        }

        public static Dictionary<string, object> CommandResponse(
            object id,
            string type,
            RouterStatusSnapshot status)
        {
            var response = Response(id, type, true);
            response["status"] = BuildStatus(status);
            return response;
        }

        public static Dictionary<string, object> Event(
            string eventName,
            long sequence,
            RouterStatusSnapshot status,
            FailoverEventArgs failover = null)
        {
            var message = new Dictionary<string, object>
            {
                ["type"] = "event",
                ["protocol"] = ProtocolName,
                ["version"] = Version,
                ["event"] = eventName,
                ["sequence"] = sequence,
                ["timestampUtc"] = DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture),
                ["status"] = BuildStatus(status),
            };
            if (failover != null)
            {
                message["fromLinkId"] = failover.FromLink ?? "";
                message["toLinkId"] = failover.ToLink ?? "";
                message["reason"] = failover.Reason ?? "";
                message["timestampUtc"] = failover.Timestamp.ToUniversalTime()
                    .ToString("O", CultureInfo.InvariantCulture);
            }

            return message;
        }

        public static Dictionary<string, object> Response(object id, string type, bool ok)
        {
            var response = new Dictionary<string, object>
            {
                ["type"] = type,
                ["ok"] = ok,
            };
            if (id != null)
            {
                response["id"] = id;
            }

            return response;
        }

        public static Dictionary<string, object> Error(
            object id,
            string errorCode,
            string error)
        {
            var response = Response(id, "error", false);
            response["protocol"] = ProtocolName;
            response["version"] = Version;
            response["errorCode"] = errorCode;
            response["error"] = error;
            return response;
        }

        private static Dictionary<string, object> BuildStatus(RouterStatusSnapshot status)
        {
            var links = new List<Dictionary<string, object>>();
            foreach (var link in status?.Links ?? new List<RouterLinkStatusSnapshot>())
            {
                links.Add(new Dictionary<string, object>
                {
                    ["stableId"] = link.StableId,
                    ["displayName"] = link.DisplayName,
                    ["transportType"] = link.TransportType,
                    ["endpoint"] = link.Endpoint,
                    ["enabled"] = link.Enabled,
                    ["isOpen"] = link.IsOpen,
                    ["isConnected"] = link.IsConnected,
                    ["health"] = link.Health,
                    ["lastPacketAgeMs"] = link.LastPacketAgeMs,
                    ["lastHeartbeatAgeMs"] = link.LastHeartbeatAgeMs,
                    ["packetLossEstimate"] = link.PacketLossEstimate,
                    ["dataRateBytesPerSecond"] = link.DataRateBytesPerSecond,
                    ["heartbeatJitterMs"] = link.HeartbeatJitterMs,
                    ["rssi"] = link.Rssi,
                    ["remoteRssi"] = link.RemoteRssi,
                    ["heartbeatCount"] = link.HeartbeatCount,
                    ["receivedFrameCount"] = link.ReceivedFrameCount,
                    ["forwardedFrameCount"] = link.ForwardedFrameCount,
                    ["duplicateFrameCount"] = link.DuplicateFrameCount,
                });
            }

            return new Dictionary<string, object>
            {
                ["state"] = status?.Running == true ? "running" : "stopped",
                ["configuredLinkCount"] = status?.ConfiguredLinkCount ?? 0,
                ["connectedLinkCount"] = status?.ConnectedLinkCount ?? 0,
                ["activeLinkId"] = status?.ActiveLinkId ?? "",
                ["manualOverrideId"] = status?.ManualOverrideId ?? "",
                ["automaticFailoverEnabled"] = status?.AutomaticFailoverEnabled == true,
                ["preferredLinkId"] = status?.PreferredLinkId ?? "",
                ["timestampUtc"] = status?.TimestampUtc ?? "",
                ["links"] = links,
            };
        }

        private static bool GetBool(IDictionary<string, object> message, string key)
        {
            var value = GetValue(message, key);
            if (value is bool boolean)
            {
                return boolean;
            }

            return bool.TryParse(Convert.ToString(value, CultureInfo.InvariantCulture), out var parsed) && parsed;
        }

        private static int? GetNullableInt(IDictionary<string, object> message, string key)
        {
            var value = GetValue(message, key);
            if (value == null)
            {
                return null;
            }

            try
            {
                return Convert.ToInt32(value, CultureInfo.InvariantCulture);
            }
            catch (Exception) when (value is IConvertible)
            {
                return null;
            }
        }

        private static long? GetLong(IDictionary<string, object> message, string key)
        {
            var value = GetValue(message, key);
            if (value == null)
            {
                return null;
            }

            try
            {
                return Convert.ToInt64(value, CultureInfo.InvariantCulture);
            }
            catch (Exception) when (value is IConvertible)
            {
                return null;
            }
        }

        private static double? GetDouble(IDictionary<string, object> message, string key)
        {
            var value = GetValue(message, key);
            if (value == null)
            {
                return null;
            }

            try
            {
                return Convert.ToDouble(value, CultureInfo.InvariantCulture);
            }
            catch (Exception) when (value is IConvertible)
            {
                return null;
            }
        }
    }

    public sealed class RouterStatusSnapshot
    {
        public bool Running;
        public int ConfiguredLinkCount;
        public int ConnectedLinkCount;
        public string ActiveLinkId = "";
        public string ManualOverrideId = "";
        public bool AutomaticFailoverEnabled;
        public string PreferredLinkId = "";
        public string TimestampUtc = "";
        public readonly List<RouterLinkStatusSnapshot> Links = new List<RouterLinkStatusSnapshot>();
    }

    public sealed class RouterLinkStatusSnapshot
    {
        public string StableId = "";
        public string DisplayName = "";
        public string TransportType = "";
        public string Endpoint = "";
        public bool Enabled;
        public bool IsOpen;
        public bool IsConnected;
        public string Health = "Disconnected";
        public double? LastPacketAgeMs;
        public double? LastHeartbeatAgeMs;
        public double PacketLossEstimate;
        public double DataRateBytesPerSecond;
        public double HeartbeatJitterMs;
        public int? Rssi;
        public int? RemoteRssi;
        public long HeartbeatCount;
        public long ReceivedFrameCount;
        public long ForwardedFrameCount;
        public long DuplicateFrameCount;
    }
}
