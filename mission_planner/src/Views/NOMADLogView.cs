// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADLogView : NOMADViewBase, IUpdatableView
    {
        private readonly NOMADConfig _config;
        private readonly DualLinkSender _sender;
        private readonly int _uiThreadId;
        private readonly TelemetryInjector _telemetryInjector = new TelemetryInjector();
        private readonly Dictionary<string, TimeSeriesData> _liveBuffers =
            new Dictionary<string, TimeSeriesData>(StringComparer.OrdinalIgnoreCase);
        private readonly HashSet<string> _activeLiveIssues =
            new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        private TabControl _tabs;
        private TabPage _postFlightTab;
        private TabPage _liveTab;
        private IFlightLogData _logData;
        private LogSummary _summary;
        private CancellationTokenSource _loadCancellation;
        private int _loadGeneration;
        private DateTime _liveStartedUtc;
        private StreamWriter _recordWriter;

        public NOMADLogView(NOMADConfig config, DualLinkSender sender)
        {
            _uiThreadId = Thread.CurrentThread.ManagedThreadId;
            _config = config ?? new NOMADConfig();
            _sender = sender;
            AutoScaleMode = AutoScaleMode.Dpi;
            BackColor = NOMADTheme.BG_DARK;
            Padding = new Padding(0);
            InitializeUI();
            Resize += (s, e) => ApplyResponsiveLayout();
        }

        private void InitializeUI()
        {
            _tabs = new TabControl
            {
                Dock = DockStyle.Fill,
                DrawMode = TabDrawMode.OwnerDrawFixed,
                Font = NOMADTheme.Font(10),
                ItemSize = new Size(132, 32),
                SizeMode = TabSizeMode.Fixed,
                BackColor = NOMADTheme.BG_DARK,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
            };
            _tabs.DrawItem += DrawLogTab;
            _tabs.SelectedIndexChanged += (s, e) => ApplyResponsiveLayout();
            _postFlightTab = new TabPage("Post-Flight")
            {
                BackColor = NOMADTheme.BG_DARK,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
            };
            _liveTab = new TabPage("Live Tuning")
            {
                BackColor = NOMADTheme.BG_DARK,
                ForeColor = NOMADTheme.TEXT_PRIMARY,
            };
            _tabs.TabPages.Add(_postFlightTab);
            _tabs.TabPages.Add(_liveTab);
            Controls.Add(_tabs);

            InitializePostFlightUI();
            InitializeRealtimeUI();
            ApplyResponsiveLayout();
        }

        public void UpdateData()
        {
            if (_tabs?.SelectedTab == _liveTab)
                UpdateRealtimeData();
        }

        private Task RunOnUiThreadAsync(Action action)
        {
            if (action == null || IsDisposed)
                return Task.CompletedTask;

            if (Thread.CurrentThread.ManagedThreadId == _uiThreadId)
            {
                action();
                return Task.CompletedTask;
            }
            if (!IsHandleCreated)
                return Task.CompletedTask;

            var completion = new TaskCompletionSource<object>();
            try
            {
                BeginInvoke((MethodInvoker)(() =>
                {
                    if (IsDisposed)
                    {
                        completion.TrySetResult(null);
                        return;
                    }

                    try
                    {
                        action();
                        completion.TrySetResult(null);
                    }
                    catch (Exception ex)
                    {
                        completion.TrySetException(ex);
                    }
                }));
            }
            catch (ObjectDisposedException)
            {
                completion.TrySetResult(null);
            }
            catch (InvalidOperationException) when (IsDisposed || !IsHandleCreated)
            {
                completion.TrySetResult(null);
            }
            return completion.Task;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                var cancellation = Interlocked.Exchange(ref _loadCancellation, null);
                if (cancellation != null)
                {
                    try { cancellation.Cancel(); } catch (ObjectDisposedException) { }
                    cancellation.Dispose();
                }
                var logData = _logData;
                _logData = null;
                logData?.Dispose();
                StopRecording();
            }
            base.Dispose(disposing);
        }
    }
}
