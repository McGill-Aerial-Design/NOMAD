// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// Settings → Payloads: dynamic editor for NOMADConfig.Payloads
// ============================================================
// A grid of up to NOMADConfig.MaxPayloads rows. Each row is a drop servo, a
// slider servo, or a relay/GPIO output. Edits a working copy that is committed
// to Config on Save.
// ============================================================

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private BindingList<PayloadControl> _payloadsBinding;
        private DataGridView _gridPayloads;
        private Label _lblPayloadsStatus;

        private TabPage CreatePayloadsTab()
        {
            var tab = CreateTabPage("Payloads");

            AddLabel(tab,
                $"Up to {NOMADConfig.MaxPayloads} payloads. Kind: Drop (servo, 3-click), Slider (live PWM), Relay (GPIO: pulse if Pulse>0, else toggle).",
                10, 8, Color.Gray);
            AddLabel(tab, "Ch/Relay = servo channel for Drop/Slider, relay number for Relay. RC = optional TX pass-through channel (5-16).",
                10, 26, Color.Gray);

            _gridPayloads = new DataGridView
            {
                Location = new Point(10, 48),
                Size = new Size(540, 250),
                AllowUserToAddRows = false,
                AllowUserToDeleteRows = false,
                AutoGenerateColumns = false,
                BackgroundColor = Color.FromArgb(40, 40, 43),
                BorderStyle = BorderStyle.None,
                RowHeadersVisible = false,
                EnableHeadersVisualStyles = false,
                SelectionMode = DataGridViewSelectionMode.FullRowSelect,
            };
            _gridPayloads.DefaultCellStyle.BackColor = Color.FromArgb(40, 40, 43);
            _gridPayloads.DefaultCellStyle.ForeColor = Color.White;
            _gridPayloads.DefaultCellStyle.SelectionBackColor = NOMADTheme.ACCENT;
            _gridPayloads.DefaultCellStyle.SelectionForeColor = Color.White;
            _gridPayloads.ColumnHeadersDefaultCellStyle.BackColor = Color.FromArgb(50, 50, 53);
            _gridPayloads.ColumnHeadersDefaultCellStyle.ForeColor = Color.White;

            _gridPayloads.Columns.Add(new DataGridViewTextBoxColumn   { DataPropertyName = "Name",       HeaderText = "Name",     Width = 96 });
            _gridPayloads.Columns.Add(new DataGridViewCheckBoxColumn  { DataPropertyName = "Enabled",    HeaderText = "On",       Width = 34 });
            var kindCol = new DataGridViewComboBoxColumn
            {
                DataPropertyName = "Kind",
                HeaderText = "Kind",
                Width = 66,
                FlatStyle = FlatStyle.Flat,
                DataSource = Enum.GetValues(typeof(PayloadKind)),
                ValueType = typeof(PayloadKind),
            };
            _gridPayloads.Columns.Add(kindCol);
            _gridPayloads.Columns.Add(new DataGridViewTextBoxColumn { DataPropertyName = "Channel",    HeaderText = "Ch/Relay", Width = 58 });
            _gridPayloads.Columns.Add(new DataGridViewTextBoxColumn { DataPropertyName = "PwmMin",     HeaderText = "Min",      Width = 50 });
            _gridPayloads.Columns.Add(new DataGridViewTextBoxColumn { DataPropertyName = "PwmMax",     HeaderText = "Max",      Width = 50 });
            _gridPayloads.Columns.Add(new DataGridViewTextBoxColumn { DataPropertyName = "PwmNeutral", HeaderText = "Neutral",  Width = 54 });
            _gridPayloads.Columns.Add(new DataGridViewCheckBoxColumn { DataPropertyName = "Reversed",  HeaderText = "Rev",      Width = 36 });
            _gridPayloads.Columns.Add(new DataGridViewTextBoxColumn { DataPropertyName = "PulseMs",    HeaderText = "Pulse",    Width = 50 });
            _gridPayloads.Columns.Add(new DataGridViewTextBoxColumn { DataPropertyName = "RcChannel",  HeaderText = "RC",       Width = 36 });
            tab.Controls.Add(_gridPayloads);

            Button MkBtn(string text, int x, int w)
            {
                var b = new Button
                {
                    Text = text,
                    Location = new Point(x, 306),
                    Size = new Size(w, 28),
                    FlatStyle = FlatStyle.Flat,
                    BackColor = NOMADTheme.ACCENT,
                    ForeColor = Color.White,
                    Font = new Font("Segoe UI", 8, FontStyle.Bold),
                };
                b.FlatAppearance.BorderSize = 0;
                tab.Controls.Add(b);
                return b;
            }

            MkBtn("Add", 10, 70).Click += (s, e) =>
            {
                if (_payloadsBinding.Count >= NOMADConfig.MaxPayloads)
                {
                    SetPayloadsStatus($"Maximum is {NOMADConfig.MaxPayloads} payloads.", Color.OrangeRed);
                    return;
                }
                _payloadsBinding.Add(new PayloadControl { Name = $"Payload {_payloadsBinding.Count + 1}" });
            };

            MkBtn("Remove", 86, 70).Click += (s, e) =>
            {
                if (_gridPayloads.CurrentRow?.DataBoundItem is PayloadControl p)
                    _payloadsBinding.Remove(p);
            };

            MkBtn("Apply RC Mappings", 170, 150).Click += (s, e) => ApplyPayloadRcMappings();

            _lblPayloadsStatus = new Label
            {
                Text = "",
                Location = new Point(10, 342),
                AutoSize = true,
                ForeColor = Color.Gray,
                Font = new Font("Segoe UI", 8),
            };
            tab.Controls.Add(_lblPayloadsStatus);

            return tab;
        }

        // Load a working copy into the grid (clones so Cancel discards edits).
        private void LoadPayloads()
        {
            var source = Config.Payloads ?? new List<PayloadControl>();
            _payloadsBinding = new BindingList<PayloadControl>(source.Select(p => p.Clone()).ToList());
            if (_gridPayloads != null) _gridPayloads.DataSource = _payloadsBinding;
        }

        // Commit the working copy back to Config.
        private void SavePayloads()
        {
            if (_payloadsBinding == null) return;
            _gridPayloads?.EndEdit();
            _gridPayloads?.BindingContext[_payloadsBinding]?.EndCurrentEdit();
            Config.Payloads = _payloadsBinding
                .Take(NOMADConfig.MaxPayloads)
                .Select(payload => payload.Clone())
                .ToList();
        }

        // Write RC{n}_OPTION on the Cube for each relay payload with an RC channel set.
        private void ApplyPayloadRcMappings()
        {
            if (_payloadsBinding == null) return;

            object comPort = null;
            try { comPort = global::MissionPlanner.MainV2.comPort; } catch { }
            if (comPort == null)
            {
                SetPayloadsStatus("Not connected to vehicle.", Color.OrangeRed);
                return;
            }

            int applied = 0;
            foreach (var p in _payloadsBinding.Where(p => p.Kind == PayloadKind.Relay && p.RcChannel >= 5 && p.RcChannel <= 16))
            {
                int option = RelayRcOptionCode(p.Channel);
                if (option == 0) continue; // relay number has no RC option code (use 0-3)
                if (TrySetParamReflect(comPort, $"RC{p.RcChannel}_OPTION", option)) applied++;
            }

            SetPayloadsStatus(
                applied > 0 ? $"Applied {applied} RC mapping(s)." : "No relay payloads with RC channel 5-16 (relay number must be 0-3).",
                applied > 0 ? Color.LightGreen : Color.Gray);
        }

        private void SetPayloadsStatus(string text, Color color)
        {
            if (_lblPayloadsStatus == null) return;
            _lblPayloadsStatus.Text = text;
            _lblPayloadsStatus.ForeColor = color;
        }
    }
}
