// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using NOMAD.MissionPlanner.Connectivity;

namespace NOMAD.MissionPlanner
{
    /// <summary>Explicit read-only snapshots; every field remains separate from command permission.</summary>
    public sealed class RuntimeStatusPanel : UserControl
    {
        private const double DisplayWindowMilliseconds = 3000;
        private readonly Func<NomadCoreClient> _getClient;
        private readonly Button _refresh = new Button
        {
            Name = "RefreshRuntimeStatus", Text = "Refresh runtime status", Dock = DockStyle.Top, Height = 32,
        };
        private readonly Label _summary = new Label
        {
            Name = "RuntimeStatusSummary", Text = "Refresh to observe the authenticated runtime.",
            Dock = DockStyle.Top, Height = 52, ForeColor = Color.Silver,
        };
        private readonly ListView _details = new ListView
        {
            Name = "RuntimeStatusValues", Dock = DockStyle.Fill, View = View.Details,
            FullRowSelect = true, ShowItemToolTips = true, HeaderStyle = ColumnHeaderStyle.Nonclickable,
        };
        private readonly System.Windows.Forms.Timer _clock = new System.Windows.Forms.Timer { Interval = 250 };
        private CancellationTokenSource _pending;
        private NomadCoreClient _queryClient;
        private NomadCoreClient _snapshotClient;
        private NomadRuntimeStatus _snapshot;
        private long _ticket;
        private bool _disposed;
        internal Task ActiveRequest { get; private set; } = Task.CompletedTask;
        internal Task PendingRead { get; private set; } = Task.CompletedTask;
        internal string OperatorStatus => _summary.Text;

        public RuntimeStatusPanel() : this(OutputController.CreateCoreClient) { }

        internal RuntimeStatusPanel(Func<NomadCoreClient> getClient)
        {
            _getClient = getClient ?? throw new ArgumentNullException(nameof(getClient));
            Size = new Size(740, 600);
            BackColor = Color.Transparent;
            _details.Columns.Add("Observation", 240);
            _details.Columns.Add("Reported value", 480);
            _refresh.Click += RefreshClicked;
            _clock.Tick += (sender, args) => UpdateProjection();
            Controls.Add(_details);
            Controls.Add(_summary);
            Controls.Add(_refresh);
            Controls.Add(new Label
            {
                Dock = DockStyle.Bottom, Height = 36, ForeColor = Color.Silver,
                Text = "Runtime observations are separate from native Mission Planner telemetry. "
                    + "Altitude datums are MSL and above home; landed state is reported autopilot telemetry.",
            });
            RenderRows(null);
        }

        private async void RefreshClicked(object sender, EventArgs args)
        {
            ActiveRequest = RefreshStatusAsync();
            try
            {
                await ActiveRequest;
            }
            catch (Exception)
            {
                if (!_disposed)
                {
                    InvalidateObservation("Runtime status view is unavailable.");
                }
            }
        }

        internal async Task RefreshStatusAsync()
        {
            if (_disposed || !IsHandleCreated || !Visible)
            {
                return;
            }
            ReconcileClient();
            if (_pending != null)
            {
                return;
            }
            var client = _getClient();
            if (client == null)
            {
                InvalidateObservation("Save core settings before reading runtime status.");
                return;
            }
            using var cancel = new CancellationTokenSource();
            _pending = cancel;
            _queryClient = client;
            var ticket = ++_ticket;
            _snapshot = null;
            _snapshotClient = null;
            _refresh.Enabled = false;
            _summary.Text = "Reading authenticated runtime status...";
            RenderRows(null);
            try
            {
                var read = client.GetStatusAsync(cancel.Token);
                PendingRead = read;
                var result = await read;
                PublishResult(ticket, client, result);
            }
            finally
            {
                if (ReferenceEquals(_pending, cancel))
                {
                    _pending = null;
                    _queryClient = null;
                    if (!_disposed)
                    {
                        _refresh.Enabled = true;
                    }
                }
            }
        }

        private void PublishResult(long ticket, NomadCoreClient client, NomadStatusResult result)
        {
            if (_disposed || ticket != _ticket || !IsHandleCreated || !Visible)
            {
                return;
            }
            if (!ReferenceEquals(client, _getClient()))
            {
                InvalidateObservation("Core settings changed; refresh to observe the current runtime.");
                return;
            }
            if (!result.Succeeded)
            {
                InvalidateObservation(result.Message);
                return;
            }
            _snapshot = result.Snapshot;
            _snapshotClient = client;
            UpdateProjection();
        }

        private void ReconcileClient()
        {
            var current = _getClient();
            if ((_queryClient != null && !ReferenceEquals(current, _queryClient))
                || (_snapshotClient != null && !ReferenceEquals(current, _snapshotClient)))
            {
                InvalidateObservation("Core settings changed; refresh to observe the current runtime.");
            }
        }

        private void InvalidateObservation(string reason, bool render = true)
        {
            ++_ticket;
            _pending?.Cancel();
            _pending = null;
            _queryClient = null;
            _snapshot = null;
            _snapshotClient = null;
            if (render && !_disposed)
            {
                _summary.Text = reason;
                _refresh.Enabled = true;
                RenderRows(null);
            }
        }

        private void UpdateProjection()
        {
            if (_disposed)
            {
                return;
            }
            ReconcileClient();
            if (_snapshot == null)
            {
                return;
            }
            var age = _snapshot.AgeUpperBoundMilliseconds;
            var expired = age > DisplayWindowMilliseconds;
            var captured = _snapshot.ReceivedAtUtc.ToString("yyyy-MM-dd HH:mm:ss.fff 'UTC'",
                CultureInfo.InvariantCulture);
            _summary.Text = "Snapshot received " + captured + "; request "
                + _snapshot.RequestDurationMilliseconds.ToString("F0", CultureInfo.InvariantCulture) + " ms.\n"
                + (expired ? "Display window expired; refresh for a new observation."
                    : "Read-only observations; command permission is evaluated by the runtime.");
            RenderRows(expired ? null : _snapshot, age);
        }

        private void RenderRows(NomadRuntimeStatus status, double age = 0)
        {
            _details.BeginUpdate();
            try
            {
                _details.Items.Clear();
                AddRuntimeRows(status);
                AddVehicleRows(status, age);
                AddTelemetryRows(status, age);
            }
            finally
            {
                _details.EndUpdate();
            }
        }

        private void AddRuntimeRows(NomadRuntimeStatus status)
        {
            Add("IPC listener ready", Boolean(status?.RuntimeReady));
            Add("Lifecycle (reported)", status?.Lifecycle);
            Add("Audit healthy", Boolean(status?.AuditHealthy));
            Add("Actuation enabled", Boolean(status?.ActuationEnabled));
            Add("Authority owner (reported)", status?.AuthorityOwnerKnown == true
                ? status.AuthorityOwner ?? "None" : null);
            Add("Authority generation", status?.AuthorityGeneration?.ToString(CultureInfo.InvariantCulture));
            Add("Runtime incarnation", status?.RuntimeIncarnation);
            Add("Runtime version", status?.RuntimeVersion);
            Add("Protocol advertised verbs", Strings(status?.ProtocolCapabilities, "None advertised"));
        }

        private void AddVehicleRows(NomadRuntimeStatus status, double age)
        {
            var fresh = CurrentHeartbeat(status, age);
            var knownClass = fresh && status.IdentityResolved == true
                && status.AircraftClass != null && status.AircraftClass != "Unknown";
            Add("MAVSDK connection open", Boolean(status?.ConnectionOpen));
            Add("Vehicle transport connected", Boolean(status?.VehicleTransportConnected));
            Add("Vehicle session established", Boolean(status?.VehicleSessionEstablished));
            Add("Vehicle session (reported)", status?.VehicleSession?.ToString(CultureInfo.InvariantCulture));
            Add("Vehicle heartbeat connected", Boolean(status?.VehicleConnected));
            Add("Heartbeat", Sample("Observed", status?.HeartbeatFresh, status?.HeartbeatAgeMs, age, fresh));
            Add("Aircraft class", knownClass ? status.AircraftClass : null);
            Add("Aircraft-supported operations", knownClass
                ? Strings(status.SupportedFlightOperations, "None supported") : null);
            Add("Armed", fresh ? Boolean(status?.Armed) : null);
            Add("Custom mode (numeric)", fresh ? status.CustomMode?.ToString(CultureInfo.InvariantCulture) : null);
        }

        private void AddTelemetryRows(NomadRuntimeStatus status, double age)
        {
            var fresh = CurrentHeartbeat(status, age);
            Add("VTOL state", Sample(status?.VtolState, status?.VtolStateValid, status?.VtolStateAgeMs, age, fresh));
            Add("Landed state (autopilot)", Sample(status?.LandedState, status?.LandedStateValid,
                status?.LandedStateAgeMs, age, fresh));
            var position = fresh && status.PositionValid == true && status.LatitudeDeg.HasValue
                && status.LongitudeDeg.HasValue && Recent(status.PositionAgeMs, age);
            var coordinates = position ? Number(status.LatitudeDeg) + ", " + Number(status.LongitudeDeg) : null;
            Add("Position (degrees)", Sample(coordinates,
                status?.PositionValid, status?.PositionAgeMs, age, fresh));
            Add("Altitude MSL (metres)", position ? Number(status.AbsoluteAltitudeM) : null);
            Add("Altitude above home (metres)", position ? Number(status.RelativeAltitudeM) : null);
            Add("GPS sample", Sample("Observed", status?.GpsValid, status?.GpsAgeMs, age, fresh));
            Add("Attitude sample", Sample("Observed", status?.AttitudeValid, status?.AttitudeAgeMs, age, fresh));
            Add("Battery sample", Sample("Observed", status?.BatteryValid, status?.BatteryAgeMs, age, fresh));
        }

        private static bool CurrentHeartbeat(NomadRuntimeStatus status, double age) => status != null
            && status.ConnectionOpen == true && status.VehicleTransportConnected == true
            && status.VehicleSessionEstablished == true && status.VehicleSession > 0
            && status.VehicleConnected == true && status.HeartbeatFresh == true && Recent(status.HeartbeatAgeMs, age);

        private static bool Recent(long? sampleAge, double age) => sampleAge.HasValue
            && sampleAge.Value + age <= DisplayWindowMilliseconds;

        private static string Sample(string value, bool? valid, long? sampleAge, double age, bool heartbeat)
        {
            var displayed = heartbeat && valid == true && Recent(sampleAge, age) ? value ?? "Unknown" : "Unknown";
            var ageText = sampleAge.HasValue
                ? (sampleAge.Value + age).ToString("F0", CultureInfo.InvariantCulture) + " ms"
                : "Unknown";
            return displayed + "; validity " + Boolean(valid) + "; age " + ageText;
        }

        private void Add(string name, string value)
        {
            var item = new ListViewItem(name) { Name = name, ToolTipText = value ?? "Unknown" };
            item.SubItems.Add(value ?? "Unknown");
            _details.Items.Add(item);
        }

        private static string Boolean(bool? value) => value.HasValue ? value.Value ? "Yes" : "No" : "Unknown";
        private static string Number(double? value) => value?.ToString("0.#######", CultureInfo.InvariantCulture);
        private static string Strings(IReadOnlyList<string> values, string empty) =>
            values == null ? null : values.Count == 0 ? empty : string.Join(", ", values);

        protected override void OnHandleCreated(EventArgs args)
        {
            base.OnHandleCreated(args);
            if (!_disposed)
            {
                _refresh.Enabled = true;
                _summary.Text = "Refresh to observe the authenticated runtime.";
                RenderRows(null);
            }
            if (Visible && !_disposed)
            {
                _clock.Start();
            }
        }

        protected override void OnHandleDestroyed(EventArgs args)
        {
            if (!_disposed)
            {
                _clock.Stop();
            }
            InvalidateObservation("View changed; refresh runtime status.", false);
            base.OnHandleDestroyed(args);
        }

        protected override void OnVisibleChanged(EventArgs args)
        {
            base.OnVisibleChanged(args);
            if (_disposed)
            {
                return;
            }
            if (Visible && IsHandleCreated && !_disposed)
            {
                _clock.Start();
            }
            else
            {
                _clock.Stop();
                InvalidateObservation("View changed; refresh runtime status.");
            }
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing && !_disposed)
            {
                _disposed = true;
                _clock.Stop();
                _clock.Dispose();
                InvalidateObservation("", false);
            }
            base.Dispose(disposing);
        }
    }
}
