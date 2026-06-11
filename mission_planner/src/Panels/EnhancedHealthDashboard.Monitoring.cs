// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Enhanced Health Dashboard — Monitoring
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Net.Http;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace NOMAD.MissionPlanner
{
    public partial class EnhancedHealthDashboard
    {
        // ============================================================
        // Data Polling
        // ============================================================

        private void StartPolling()
        {
            _pollTimer = new System.Threading.Timer(
                _ => PollHealth(),
                null,
                TimeSpan.FromMilliseconds(500),
                TimeSpan.FromMilliseconds(_config.HealthPollInterval)
            );
        }

        private async void PollHealth()
        {
            // Prevent overlapping polls — if the previous one is still in-flight, skip.
            if (System.Threading.Interlocked.Exchange(ref _isPollInFlight, 1) == 1)
                return;

            try
            {
                if (IsDisposed || !IsHandleCreated) return;

                var healthResponse = await JetsonApiService.GetAsync("/health/detailed");

                if (IsDisposed || !IsHandleCreated) return;

                if (!healthResponse.IsSuccessStatusCode)
                {
                    UiAsync.RunSync(this, () => UpdateStatusError($"HTTP {healthResponse.StatusCode}"), "UpdateStatusError");
                    return;
                }

                var healthJson = await healthResponse.Content.ReadAsStringAsync();
                var healthData = JObject.Parse(healthJson);

                var networkData = await TryGetOptionalJsonAsync(
                    "/network/status", OptionalEndpointBudget, useLongRunClient: false);

                UiAsync.RunSync(this, () => UpdateUI(healthData, networkData, null), "UpdateUI");

                if (DateTime.UtcNow - _lastIsaacPollUtc >= IsaacPollInterval)
                {
                    _lastIsaacPollUtc = DateTime.UtcNow;
                    var isaacData = await TryGetOptionalJsonAsync(
                        "/api/isaac/status", TimeSpan.FromSeconds(5), useLongRunClient: true);
                    if (isaacData != null)
                        UiAsync.RunSync(this, () => UpdateDriftStats(isaacData), "UpdateDriftStats");
                }
            }
            catch (ObjectDisposedException) { }
            catch (InvalidOperationException) { }
            catch (Exception ex)
            {
                UiAsync.RunSync(this, () => UpdateStatusError(ex.Message), "UpdateStatusError");
            }
            finally
            {
                System.Threading.Interlocked.Exchange(ref _isPollInFlight, 0);
            }
        }

        private async Task<JObject> TryGetOptionalJsonAsync(string path, TimeSpan budget, bool useLongRunClient)
        {
            try
            {
                var requestTask = useLongRunClient
                    ? JetsonApiService.GetLongRunAsync(path)
                    : JetsonApiService.GetAsync(path);
                var completed = await Task.WhenAny(requestTask, Task.Delay(budget));
                if (completed != requestTask)
                {
                    _ = requestTask.ContinueWith(t =>
                    {
                        if (t.IsFaulted)
                            System.Diagnostics.Debug.WriteLine($"Optional health endpoint {path} failed late: {t.Exception?.GetBaseException().Message}");
                        else if (t.Status == TaskStatus.RanToCompletion)
                            t.Result.Dispose();
                    });
                    return null;
                }

                using (var response = await requestTask)
                {
                    if (!response.IsSuccessStatusCode) return null;
                    var json = await response.Content.ReadAsStringAsync();
                    return JObject.Parse(json);
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Optional health endpoint {path} error: {ex.Message}");
                return null;
            }
        }

        public void RefreshHealth()
        {
            PollHealth();
        }

        // ============================================================
        // UI Updates
        // ============================================================

        private void UpdateUI(JObject data, JObject networkData = null, JObject isaacData = null)
        {
            try
            {
                _lastHealthSuccessUtc = DateTime.UtcNow;

                var status = data["status"]?.ToString() ?? "unknown";
                UpdateOverallStatus(status);

                var cpuTemp = data["cpu_temp"]?.Value<float>() ?? 0;
                var cpuLoad = data["cpu_load"]?.Value<float>() ?? 0;
                UpdateMetric(_lblCpuTemp, _prgCpuTemp, cpuTemp, "C", 85, 95);
                UpdateMetric(_lblCpuLoad, _prgCpuLoad, cpuLoad, "%", 80, 95);

                var gpuTemp = data["gpu_temp"]?.Value<float>() ?? 0;
                var gpuLoad = data["gpu_load"]?.Value<float>() ?? 0;
                UpdateMetric(_lblGpuTemp, _prgGpuTemp, gpuTemp, "C", 85, 95);
                UpdateMetric(_lblGpuLoad, _prgGpuLoad, gpuLoad, "%", 80, 95);

                var memUsed = data["memory_used_pct"]?.Value<float>() ?? 0;
                UpdateMetric(_lblMemory, _prgMemory, memUsed, "%", 80, 95);

                var diskUsed = data["disk_used_pct"]?.Value<float>() ?? 0;
                UpdateMetric(_lblDisk, _prgDisk, diskUsed, "%", 80, 95);

                var power = data["power_draw_w"]?.Value<float>() ?? 0;
                var fan = data["fan_speed_pct"]?.Value<float>() ?? 0;
                _lblPower.Text = $"Power: {power:F1}W";
                _lblFan.Text = $"Fan: {fan:F0}%";

                if (networkData != null)
                {
                    UpdateNetworkStatus(networkData);
                }
                else
                {
                    var tsConnected = data["tailscale_connected"]?.Value<bool>() ?? false;
                    var tsIp = data["tailscale_ip"]?.ToString() ?? "--";
                    _lblTailscaleIP.Text = tsConnected ? tsIp : "(daemon down)";
                    _lblTailscaleIP.ForeColor = tsConnected ? Color.White : Color.Orange;
                    _lblPeerCount.Text = "--";
                    _lblInternetStatus.Text = "--";
                    _lblInternetStatus.ForeColor = Color.Gray;
                    _lblGcsReachable.Text = "--";
                    _lblGcsReachable.ForeColor = Color.Gray;
                    _lblModemConnection.Text = "--";
                    _lblModemStatus.Text = "--";
                    _lblModemSignal.Text = "--";
                    _lblModemInterface.Text = "--";
                }

                var vio = data["vio"];
                if (vio != null && vio.Type != JTokenType.Null)
                {
                    var vioHealth = vio["health"]?.ToString() ?? "unknown";
                    var vioConf = vio["tracking_confidence"]?.Value<float>() ?? 0;
                    var vioRate = vio["message_rate_hz"]?.Value<float>() ?? 0;
                    _lblVioStatus.Text = $"Status: {vioHealth}\nConfidence: {vioConf:F1}\nRate: {vioRate:F0} Hz";
                    _lblVioStatus.ForeColor = vioHealth == "healthy" ? Color.LimeGreen :
                        (vioHealth == "degraded" ? Color.Orange : Color.Red);
                }
                else
                {
                    _lblVioStatus.Text = "Status: No Data\nConfidence: --\nRate: -- Hz";
                    _lblVioStatus.ForeColor = Color.Gray;
                }

                UpdateHistory(cpuTemp, gpuTemp, cpuLoad, gpuLoad, memUsed);

                _lblLastUpdate.Text = $"Last update: {DateTime.Now:HH:mm:ss}";

                DrawGraph();

                if (isaacData != null)
                    UpdateDriftStats(isaacData);

                CheckAlerts(cpuTemp, gpuTemp, memUsed, diskUsed);
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Health UI error: {ex.Message}");
            }
        }

        private void UpdateNetworkStatus(JObject networkData)
        {
            try
            {
                var tailscale = networkData["tailscale"];
                if (tailscale != null && tailscale.Type != JTokenType.Null)
                {
                    var status = tailscale["status"]?.ToString() ?? "unknown";
                    var ip = tailscale["ip"]?.ToString() ?? "--";
                    var peerCount = tailscale["peer_count"]?.Type == JTokenType.Null
                        ? 0 : (tailscale["peer_count"]?.Value<int>() ?? 0);
                    var latency = tailscale["latency_ms"];

                    bool isConnected = status.Equals("connected", StringComparison.OrdinalIgnoreCase);
                    _lblTailscaleIP.Text = string.IsNullOrEmpty(ip) || ip == "--"
                        ? (isConnected ? "(no IP)" : status.Replace("_", " "))
                        : ip;
                    _lblTailscaleIP.ForeColor = isConnected
                        ? Color.White
                        : (status == "connecting" ? Color.Yellow : Color.Orange);
                    _lblPeerCount.Text = peerCount > 0 ? peerCount.ToString() : "0";
                    _lblPeerCount.ForeColor = peerCount > 0 ? Color.LightGray : Color.Gray;
                }
                else
                {
                    _lblTailscaleIP.Text = "(daemon down)";
                    _lblTailscaleIP.ForeColor = Color.Orange;
                    _lblPeerCount.Text = "--";
                }

                var internetReachable = networkData["internet_reachable"]?.Value<bool>() ?? false;
                _lblInternetStatus.Text = internetReachable ? "Reachable" : "Unreachable";
                _lblInternetStatus.ForeColor = internetReachable ? Color.LimeGreen : Color.Red;

                var gcsReachable = networkData["gcs_reachable"]?.Value<bool>() ?? false;
                _lblGcsReachable.Text = gcsReachable ? "Reachable" : "Unreachable";
                _lblGcsReachable.ForeColor = gcsReachable ? Color.LimeGreen : Color.Red;

                var modem = networkData["modem"];
                if (modem != null && modem.Type != JTokenType.Null)
                {
                    UpdateModemFields(modem);
                }
                else
                {
                    _lblModemConnection.Text = "Not detected";
                    _lblModemConnection.ForeColor = Color.Gray;
                    _lblModemStatus.Text = "--";
                    _lblModemStatus.ForeColor = Color.Gray;
                    _lblModemSignal.Text = "--";
                    _lblModemSignal.ForeColor = Color.Gray;
                    _lblModemInterface.Text = "--";
                    _lblModemInterface.ForeColor = Color.Gray;
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Network status error: {ex.Message}");
            }
        }

        private void UpdateModemFields(JToken modem)
        {
            var connected = modem["connected"]?.Value<bool>() ?? false;
            var carrier = modem["carrier"]?.ToString() ?? "";
            var technology = modem["technology"]?.ToString() ?? "";
            var signalQuality = modem["signal_quality"]?.ToString() ?? "";
            var signalDbm = modem["signal_strength_dbm"];
            var signalPercent = modem["signal_percent"];
            var nmName = modem["nm_connection_name"]?.ToString() ?? "";
            var nmState = modem["nm_connection_state"]?.ToString() ?? "";
            var iface = modem["interface"]?.ToString() ?? "";
            var ip4 = modem["ip_address"]?.ToString() ?? "";

            if (!string.IsNullOrEmpty(nmName))
            {
                _lblModemConnection.Text = string.IsNullOrEmpty(nmState)
                    ? nmName
                    : $"{nmName} ({nmState})";
                _lblModemConnection.ForeColor = nmState == "activated"
                    ? Color.LimeGreen
                    : nmState == "activating" ? Color.Yellow : Color.Orange;
            }
            else
            {
                _lblModemConnection.Text = "(no NM profile)";
                _lblModemConnection.ForeColor = Color.Gray;
            }

            if (!string.IsNullOrEmpty(carrier))
            {
                _lblModemStatus.Text = string.IsNullOrEmpty(technology)
                    ? carrier
                    : $"{carrier} \u00b7 {technology}";
                _lblModemStatus.ForeColor = connected ? Color.LimeGreen : Color.Orange;
            }
            else
            {
                _lblModemStatus.Text = connected ? "Connected (no carrier name)" : "Disconnected";
                _lblModemStatus.ForeColor = connected ? Color.LightGreen : Color.Red;
            }

            if (signalDbm != null && signalDbm.Type != JTokenType.Null)
            {
                var parts = new List<string>
                {
                    $"{signalDbm} dBm"
                };
                if (!string.IsNullOrEmpty(signalQuality)) parts.Add(Capitalize(signalQuality));
                if (signalPercent != null && signalPercent.Type != JTokenType.Null)
                    parts.Add($"{signalPercent}%");
                _lblModemSignal.Text = string.Join("  \u00b7  ", parts);
                _lblModemSignal.ForeColor = GetSignalColor(signalQuality);
            }
            else if (!string.IsNullOrEmpty(signalQuality))
            {
                _lblModemSignal.Text = Capitalize(signalQuality);
                _lblModemSignal.ForeColor = GetSignalColor(signalQuality);
            }
            else
            {
                _lblModemSignal.Text = "--";
                _lblModemSignal.ForeColor = Color.Gray;
            }

            string ifPart = string.IsNullOrEmpty(iface) ? "--" : iface;
            string ipPart = string.IsNullOrEmpty(ip4) ? "(no IP)" : ip4;
            _lblModemInterface.Text = $"{ifPart}  {ipPart}";
            _lblModemInterface.ForeColor = !string.IsNullOrEmpty(ip4) ? Color.White : Color.Orange;
        }

        private static string Capitalize(string s) =>
            string.IsNullOrEmpty(s) ? s : char.ToUpper(s[0]) + s.Substring(1).Replace('_', ' ');

        private Color GetSignalColor(string quality)
        {
            return quality?.ToLower() switch
            {
                "excellent" => Color.LimeGreen,
                "good" => Color.LightGreen,
                "fair" => Color.Yellow,
                "poor" => Color.Orange,
                _ => Color.Gray
            };
        }

        private string TruncateString(string value, int maxLength)
        {
            if (string.IsNullOrEmpty(value)) return value;
            return value.Length <= maxLength ? value : value.Substring(0, maxLength - 3) + "...";
        }

        private void UpdateDriftStats(JObject isaacData)
        {
            try
            {
                if (isaacData == null)
                {
                    _lblDriftCycles.Text = "Cycles: --";
                    _lblDriftAvg.Text = "Avg: --";
                    _lblDriftMax.Text = "Max: --";
                    _lblDriftWarning.Visible = false;
                    return;
                }

                var bridgeStats = isaacData["bridge_stats"] ?? isaacData["stats"];
                var tiltDrift = bridgeStats?["tilt_drift"];

                if (tiltDrift == null || tiltDrift.Type == JTokenType.Null)
                {
                    _lblDriftCycles.Text = "Cycles: 0";
                    _lblDriftAvg.Text = "Avg: --";
                    _lblDriftMax.Text = "Max: --";
                    _lblDriftWarning.Visible = false;
                    return;
                }

                var cycles = tiltDrift["cycles"]?.Value<int>() ?? 0;
                var avgDrift = tiltDrift["avg_drift_m"]?.Value<float>() ?? 0f;
                var maxDrift = tiltDrift["max_drift_m"]?.Value<float>() ?? 0f;

                _lblDriftCycles.Text = $"Cycles: {cycles}";
                _lblDriftAvg.Text = $"Avg: {avgDrift * 100f:F1}cm";
                _lblDriftMax.Text = $"Max: {maxDrift * 100f:F1}cm";

                if (maxDrift > 0.05f)
                {
                    _lblDriftWarning.Text = "DRIFT HIGH";
                    _lblDriftWarning.ForeColor = Color.Red;
                    _lblDriftWarning.Visible = true;
                }
                else if (cycles > 0)
                {
                    _lblDriftWarning.Text = "OK";
                    _lblDriftWarning.ForeColor = Color.LimeGreen;
                    _lblDriftWarning.Visible = true;
                }
                else
                {
                    _lblDriftWarning.Visible = false;
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Drift stats error: {ex.Message}");
            }
        }

        private void UpdateOverallStatus(string status)
        {
            switch (status.ToLower())
            {
                case "ok":
                    _lblOverallStatus.Text = "\u25cf HEALTHY";
                    _lblOverallStatus.ForeColor = Color.LimeGreen;
                    break;
                case "warning":
                    _lblOverallStatus.Text = "\u25cf WARNING";
                    _lblOverallStatus.ForeColor = Color.Yellow;
                    break;
                case "critical":
                    _lblOverallStatus.Text = "\u25cf CRITICAL";
                    _lblOverallStatus.ForeColor = Color.Red;
                    break;
                default:
                    _lblOverallStatus.Text = "\u25cf UNKNOWN";
                    _lblOverallStatus.ForeColor = Color.Gray;
                    break;
            }
        }

        private void UpdateMetric(Label label, ProgressBar progress, float value, string unit, float warnThreshold, float critThreshold)
        {
            label.Text = $"{value:F1}{unit}";
            progress.Value = Math.Min(100, Math.Max(0, (int)value));

            if (value >= critThreshold)
            {
                label.ForeColor = Color.Red;
            }
            else if (value >= warnThreshold)
            {
                label.ForeColor = Color.Yellow;
            }
            else
            {
                label.ForeColor = Color.LimeGreen;
            }
        }

        private void UpdateStatusError(string error)
        {
            if (_lastHealthSuccessUtc != DateTime.MinValue &&
                DateTime.UtcNow - _lastHealthSuccessUtc < OfflineGrace)
            {
                _lblLastUpdate.Text = $"Last update: {DateTime.Now:HH:mm:ss} (missed poll: {error})";
                return;
            }

            _lblOverallStatus.Text = "\u25cf OFFLINE";
            _lblOverallStatus.ForeColor = Color.Red;
            _lblLastUpdate.Text = $"Error: {error}";
        }

        private void UpdateHistory(float cpuTemp, float gpuTemp, float cpuLoad, float gpuLoad, float memory)
        {
            AddToHistory(_cpuTempHistory, cpuTemp);
            AddToHistory(_gpuTempHistory, gpuTemp);
            AddToHistory(_cpuLoadHistory, cpuLoad);
            AddToHistory(_gpuLoadHistory, gpuLoad);
            AddToHistory(_memoryHistory, memory);
        }

        private void AddToHistory(Queue<float> queue, float value)
        {
            queue.Enqueue(value);
            while (queue.Count > HISTORY_LENGTH)
            {
                queue.Dequeue();
            }
        }
    }
}
