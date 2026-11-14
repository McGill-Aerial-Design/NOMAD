// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Notification Toast Overlay
// ============================================================
// Small auto-dismissing popup shown bottom-right of the Mission
// Planner main window for Warning/Critical notifications, so the
// operator sees boundary/battery alerts even when a non-NOMAD page
// (FlightData, FlightPlan, ...) is active. Wired to
// NotificationService.NotificationAdded by the plugin at Init.
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    internal static class NotificationToast
    {
        private const int TOAST_WIDTH = 340;
        private const int TOAST_HEIGHT = 78;
        private const int TOAST_MARGIN = 10;
        private const int MAX_VISIBLE = 4;

        private static readonly List<ToastForm> _open = new List<ToastForm>();
        private static NotificationService _service;
        private static Form _mainForm;
        private static long _attachment;

        /// <summary>
        /// Subscribe to a notification service; Warning/Critical notifications
        /// pop a toast anchored to the main form's screen.
        /// </summary>
        public static void Attach(NotificationService service, Form mainForm)
        {
            Detach();
            _service = service;
            _mainForm = mainForm;
            if (_service != null)
            {
                _service.NotificationAdded += OnNotificationAdded;
            }
        }

        public static void Detach()
        {
            System.Threading.Interlocked.Increment(ref _attachment);
            if (_service != null)
            {
                _service.NotificationAdded -= OnNotificationAdded;
                _service = null;
            }
            foreach (var t in _open.ToArray())
            {
                try { t.Close(); } catch { }
            }
            _open.Clear();
            _mainForm = null;
        }

        private static void OnNotificationAdded(object sender, NotificationEventArgs e)
        {
            long attachment = System.Threading.Interlocked.Read(ref _attachment);
            var n = e?.Notification;
            if (n == null || n.Severity == NotificationSeverity.Info) return;

            var form = _mainForm;
            if (form == null || form.IsDisposed || !ReferenceEquals(sender, _service)) return;
            Action show = () =>
            {
                if (attachment == System.Threading.Interlocked.Read(ref _attachment) &&
                    ReferenceEquals(sender, _service) && ReferenceEquals(form, _mainForm)) { ShowToast(n); }
            };

            try
            {
                if (form.InvokeRequired)
                {
                    form.BeginInvoke(show);
                }
                else
                {
                    show();
                }
            }
            catch (Exception ex)
            {
                Log.Debug($"Toast show failed — {ex.Message}");
            }
        }

        private static void ShowToast(Notification n)
        {
            if (_mainForm == null || _mainForm.IsDisposed) return;

            // Cap how many stack up; drop the oldest first.
            while (_open.Count >= MAX_VISIBLE)
            {
                _open[0].Close();
            }

            var toast = new ToastForm(n);
            toast.FormClosed += (s, e) =>
            {
                _open.Remove(toast);
                Reposition();
            };
            _open.Add(toast);
            Reposition();
            toast.Show(_mainForm);
            Reposition(); // size known only after handle creation
        }

        /// <summary>Stack open toasts upward from the bottom-right corner.</summary>
        private static void Reposition()
        {
            if (_mainForm == null || _mainForm.IsDisposed) return;
            var area = Screen.FromControl(_mainForm).WorkingArea;
            int y = area.Bottom - TOAST_MARGIN;
            for (int i = _open.Count - 1; i >= 0; i--)
            {
                var t = _open[i];
                if (t.IsDisposed) continue;
                y -= t.Height + TOAST_MARGIN;
                t.Location = new Point(area.Right - t.Width - TOAST_MARGIN, y);
            }
        }

        /// <summary>
        /// Borderless click-to-dismiss popup; never steals focus from the GCS.
        /// </summary>
        [global::MissionPlanner.Utilities.PreventTheming]
        private sealed class ToastForm : Form
        {
            private readonly Timer _dismissTimer;

            protected override bool ShowWithoutActivation => true;

            public ToastForm(Notification n)
            {
                bool critical = n.Severity == NotificationSeverity.Critical;

                FormBorderStyle = FormBorderStyle.None;
                StartPosition = FormStartPosition.Manual;
                ShowInTaskbar = false;
                TopMost = true;
                Size = new Size(TOAST_WIDTH, TOAST_HEIGHT);
                var root = new System.Windows.Controls.Border
                {
                    Background = critical ? System.Windows.Media.Brushes.DarkRed : System.Windows.Media.Brushes.DarkGoldenrod,
                    BorderBrush = critical ? System.Windows.Media.Brushes.Red : System.Windows.Media.Brushes.Orange,
                    BorderThickness = new System.Windows.Thickness(5, 0, 0, 0),
                    Padding = new System.Windows.Thickness(9, 6, 8, 4),
                };
                var content = new System.Windows.Controls.StackPanel();
                content.Children.Add(new System.Windows.Controls.TextBlock
                {
                    Text = $"{(critical ? "CRITICAL" : "WARNING")} — {n.Title}",
                    FontWeight = System.Windows.FontWeights.Bold,
                    Foreground = System.Windows.Media.Brushes.White,
                    TextTrimming = System.Windows.TextTrimming.CharacterEllipsis,
                });
                content.Children.Add(new System.Windows.Controls.TextBlock
                {
                    Text = n.Message ?? "", Foreground = System.Windows.Media.Brushes.Gainsboro,
                    TextWrapping = System.Windows.TextWrapping.Wrap,
                    TextTrimming = System.Windows.TextTrimming.CharacterEllipsis,
                    MaxHeight = 42,
                });
                root.Child = content;
                root.MouseLeftButtonUp += (s, e) => Close();
                Controls.Add(WpfUi.Host(root));

                _dismissTimer = new Timer { Interval = critical ? 12000 : 7000 };
                _dismissTimer.Tick += (s, e) => Close();
                _dismissTimer.Start();
            }

            protected override void Dispose(bool disposing)
            {
                if (disposing)
                {
                    _dismissTimer?.Stop();
                    _dismissTimer?.Dispose();
                }
                base.Dispose(disposing);
            }
        }
    }
}
