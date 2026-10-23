// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Diagnostics;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    internal sealed class TerminationWidget : UserControl
    {
        private readonly TerminationController _controller;
        private readonly CancellationToken _shutdown;
        private readonly TerminationGesture _gesture = new TerminationGesture();
        private readonly Stopwatch _clock = Stopwatch.StartNew();
        private readonly System.Windows.Forms.Timer _timer = new System.Windows.Forms.Timer { Interval = 50 };
        private readonly ToolTip _toolTip = new ToolTip { AutoPopDelay = 15000 };
        private readonly Label _status;
        private readonly Button _check;
        private readonly Func<bool> _canInteract;
        private readonly Func<IWin32Window, string, DialogResult> _confirm;
        internal Task PendingReview { get; private set; } = Task.CompletedTask;
        private bool _busy, _waitingForRelease, _reviewCancelled, _confirming;
        private long _generation;
        private uint _mode;
        private string _target, _error;

        [DllImport("user32.dll")]
        private static extern short GetAsyncKeyState(int key);
        [DllImport("user32.dll")]
        private static extern IntPtr GetForegroundWindow();

        internal TerminationWidget(TerminationController controller, CancellationToken shutdown,
            Func<bool> canInteract = null, Func<IWin32Window, string, DialogResult> confirm = null)
        {
            _canInteract = canInteract ?? CanInteract;
            _confirm = confirm ?? Confirm;
            _controller = controller;
            _shutdown = shutdown;
            BackColor = NOMADTheme.CARD_BG;
            Padding = new Padding(10, 6, 10, 6);
            var header = new Panel { Dock = DockStyle.Top, Height = 24 };
            header.Controls.Add(new Label
            {
                Text = "TERMINATE", Dock = DockStyle.Fill, ForeColor = NOMADTheme.ERROR,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Bold), TextAlign = ContentAlignment.MiddleLeft,
            });
            _check = ControlFactory.Button("Check", width: 55, height: 24);
            _check.Dock = DockStyle.Right;
            _check.Click += async (sender, args) => await CheckSupportAsync();
            header.Controls.Add(_check);
            _status = new Label
            {
                Dock = DockStyle.Fill, ForeColor = NOMADTheme.TEXT_SECONDARY,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL), TextAlign = ContentAlignment.MiddleLeft,
            };
            Controls.Add(_status);
            Controls.Add(header);
            _timer.Tick += Sample;
            RefreshStatus();
        }

        protected override void OnHandleCreated(EventArgs args)
        {
            base.OnHandleCreated(args);
            _timer.Start();
        }

        internal bool IsForeground()
        {
            var form = FindForm();
            return !_shutdown.IsCancellationRequested && !IsDisposed && Visible && form?.Visible == true &&
                form.ContainsFocus && GetForegroundWindow() == form.Handle;
        }

        private bool CanInteract() => IsForeground() && ClientRectangle.Contains(PointToClient(Cursor.Position));

        private void Sample(object sender, EventArgs args)
        {
            if (_confirming) { return; }
            bool eligible = !VideoFlightInput.Captured && CanInteract();
            bool left = eligible && (GetAsyncKeyState(0xA0) & 0x8000) != 0;
            bool right = eligible && (GetAsyncKeyState(0xA1) & 0x8000) != 0;
            SampleState(eligible, left, right, _clock.ElapsedMilliseconds);
        }

        internal void SampleState(bool eligible, bool left, bool right, long milliseconds)
        {
            eligible &= !VideoFlightInput.Captured;
            if (_confirming) { return; }
            if (!eligible)
            {
                _gesture.Reset();
                _reviewCancelled = true;
                if (_waitingForRelease) { _waitingForRelease = _busy = false; }
                RefreshStatus();
                return;
            }
            if (_waitingForRelease && !left && !right)
            {
                _waitingForRelease = false;
                PendingReview = ConfirmAndRequestAsync();
            }
            else if (!_busy && _gesture.Update(_controller?.CanRequest == true, left, right, milliseconds))
            { PendingReview = PrepareReviewAsync(); }
            RefreshStatus();
        }

        private async Task CheckSupportAsync()
        {
            if (_busy || _controller == null || _shutdown.IsCancellationRequested) { return; }
            _busy = true;
            _gesture.Reset();
            RefreshStatus();
            _error = null;
            try { await _controller.CheckAsync(); }
            catch (Exception) { _error = "Support could not be verified. No termination sent."; }
            finally { _busy = false; RefreshStatus(); }
        }

        private async Task PrepareReviewAsync()
        {
            if (_busy || _controller == null || !_canInteract()) { return; }
            _busy = true;
            _reviewCancelled = false;
            _error = null;
            try
            {
                await _controller.CheckAsync();
                if (_reviewCancelled || !_canInteract() ||
                    !_controller.TryCaptureRequest(out _generation, out _mode)) { return; }
                _target = _controller.TargetDescription;
                _waitingForRelease = true;
            }
            catch (Exception) { _error = "Support could not be verified. No termination sent."; }
            finally
            {
                if (!_waitingForRelease) { _busy = false; }
                RefreshStatus();
            }
        }

        private async Task ConfirmAndRequestAsync()
        {
            _confirming = true;
            _gesture.Reset();
            try
            {
                if (_reviewCancelled || !_canInteract() || !_controller.CanRequest) { return; }
                var result = _confirm(FindForm(), _target);
                if (result != DialogResult.OK || IsDisposed || _shutdown.IsCancellationRequested) { return; }
                _status.Text = "Request pending";
                string outcome = await _controller.RequestAsync(_generation, _mode);
                if (!IsDisposed && !_shutdown.IsCancellationRequested && FindForm() != null)
                {
                    MessageBox.Show(FindForm(), outcome, "Termination", MessageBoxButtons.OK,
                        MessageBoxIcon.Information);
                }
            }
            catch (Exception)
            {
                if (!IsDisposed && !_shutdown.IsCancellationRequested)
                { MessageBox.Show(FindForm(), "Termination outcome is uncertain. Inspect aircraft; do not replay.",
                    "Termination", MessageBoxButtons.OK, MessageBoxIcon.Warning); }
            }
            finally { _confirming = _busy = false; RefreshStatus(); }
        }

        private static DialogResult Confirm(IWin32Window owner, string target) => MessageBox.Show(owner,
            $"Activate TERMINATE on {target}?\n" +
            "Fixed wing: motor off and full termination surfaces. VTOL: rapid descent through touchdown.\n" +
            "The aircraft chooses its response. This latches until controller reboot.\n" +
            "This is not a LAND command.", "TERMINATE aircraft",
            MessageBoxButtons.OKCancel, MessageBoxIcon.Warning, MessageBoxDefaultButton.Button2);

        internal void RefreshStatus()
        {
            if (IsDisposed) { return; }
            _check.Enabled = !_busy && _controller != null && !_shutdown.IsCancellationRequested;
            string detail = _error ?? _controller?.Status ?? "Termination unavailable.";
            string summary = "Unavailable · check support";
            if (_waitingForRelease) { summary = "Release both Shift keys to review."; }
            else if (_busy) { summary = "Checking support…"; }
            else if (_controller?.Observed == true) { summary = "Observed · reboot required"; }
            else if (_controller?.Supported == true && !_controller.Armed) { summary = "Supported · disarmed"; }
            else if (_controller?.CanRequest == true) { summary = "Hover here, then press both Shift keys."; }
            _status.Text = summary;
            _toolTip.SetToolTip(this, detail);
            _toolTip.SetToolTip(_status, detail);
            _toolTip.SetToolTip(_check, detail);
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _reviewCancelled = true;
                _gesture.Reset();
                _timer.Dispose();
                _toolTip.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
