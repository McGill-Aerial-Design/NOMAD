// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using NOMAD.MissionPlanner.Connectivity;

namespace NOMAD.MissionPlanner
{
    /// <summary>Requests LAND engagement through the runtime; owns only operator interaction.</summary>
    public sealed class LandControlPanel : UserControl
    {
        internal const string ConfirmationMessage = "Request Copter LAND mode engagement?\n\n" +
            "Success means LAND mode was observed after acknowledgement; touchdown is not verified. " +
            "LAND is not termination. Runtime authority must already be explicitly admitted.";
        private readonly Func<bool> _confirmLand;
        private readonly Func<bool> _inputStillCurrent;
        private readonly NomadCoreClient _client;
        private readonly CancellationTokenSource _shutdown = new CancellationTokenSource();
        private readonly Button _request = new Button
        {
            Name = "RequestLand", Text = "Engage Copter LAND", Location = new Point(0, 0), Size = new Size(180, 30)
        };
        private readonly Label _status = new Label
        {
            Name = "LandStatus", Location = new Point(0, 40), AutoSize = true, MaximumSize = new Size(560, 0),
            ForeColor = Color.Silver,
            Text = "LAND engagement only; touchdown is not verified. LAND is not termination. " +
                "Save core settings and explicitly admit authority before requesting."
        };
        private bool _requestInProgress;
        private bool _disposed;
        internal string OperatorStatus => _status.Text;
        internal Task ActiveRequest { get; private set; } = Task.CompletedTask;

        public LandControlPanel() : this(ConfirmLand, null) { }

        internal LandControlPanel(Func<bool> confirmLand, Func<bool> inputStillCurrent = null)
        {
            _confirmLand = confirmLand ?? ConfirmLand;
            _inputStillCurrent = inputStillCurrent;
            _client = OutputController.CreateCoreClient();
            Size = new Size(560, 155);
            _request.Click += (sender, args) => ActiveRequest = RequestLandAsync();
            Controls.Add(_request);
            Controls.Add(_status);
        }

        internal async Task RequestLandAsync()
        {
            if (_requestInProgress || _disposed)
            {
                return;
            }
            _requestInProgress = true;
            _request.Enabled = false;
            try
            {
                if (!InputIsCurrent())
                {
                    _status.Text = "Aircraft profile changed. Reopen its controls before requesting LAND.";
                    return;
                }
                if (!_confirmLand() || !InputIsCurrent())
                {
                    return;
                }
                if (_client == null)
                {
                    _status.Text = "Save core settings before requesting LAND. No mutation request was sent.";
                    return;
                }
                _status.Text = "Awaiting runtime LAND engagement result; touchdown is not verified.";
                var result = await _client.LandAsync(_shutdown.Token, InputIsCurrent);
                if (!_disposed && InputIsCurrent())
                {
                    _status.Text = result.Succeeded ? "LAND mode observed; touchdown not verified." :
                        OutputController.DescribeFailure("LAND engagement", result);
                }
            }
            finally
            {
                _requestInProgress = false;
                if (!_disposed)
                {
                    _request.Enabled = true;
                }
            }
        }

        private bool InputIsCurrent() => !_disposed && _client == OutputController.CreateCoreClient() &&
            (_inputStillCurrent == null || _inputStillCurrent());

        private static bool ConfirmLand() => MessageBox.Show(ConfirmationMessage, "NOMAD LAND engagement",
            MessageBoxButtons.OKCancel, MessageBoxIcon.Warning, MessageBoxDefaultButton.Button2) == DialogResult.OK;

        protected override void Dispose(bool disposing)
        {
            if (disposing && !_disposed)
            {
                _disposed = true;
                _shutdown.Cancel();
                _shutdown.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
