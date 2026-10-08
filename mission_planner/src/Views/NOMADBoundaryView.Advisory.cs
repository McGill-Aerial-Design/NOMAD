// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// NOMAD Boundary View - Advisory Display
// ============================================================

using System;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;
using MissionPlanner;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADBoundaryView
    {
        private Label _lblMigrationNotice;
        private CheckBox _chkEnableMonitoring;
        private NumericUpDown _nudMaxAlt;
        private CheckBox _chkAdvisoryAudio;

        private TableLayoutPanel BuildMonitoringCard()
        {
            var card = Card("ADVISORY DISPLAY", out var body);
            AddAdvisoryPreviewDescription(body);
            AddAdvisoryPreviewToggle(body);
            AddAdvisoryAltitudeThreshold(body);
            AddAdvisoryAudioToggle(body);
            AddMigrationNotice(body);
            return card;
        }

        private static void AddAdvisoryPreviewDescription(TableLayoutPanel body)
        {
            AddRow(body, new Label
            {
                Text = "Local position preview only. NOMAD runtime enforcement status is not available here.",
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                MaximumSize = new Size(320, 0),
            });
        }

        private void AddAdvisoryPreviewToggle(TableLayoutPanel body)
        {
            _chkEnableMonitoring = new CheckBox
            {
                Text = "Show local position against saved outlines",
                ForeColor = Color.White,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                AutoSize = true,
                Checked = _monitor?.IsMonitoring ?? _missionConfig.AdvisoryPreviewEnabled,
            };
            _chkEnableMonitoring.CheckedChanged += (s, e) =>
            {
                _missionConfig.AdvisoryPreviewEnabled = _chkEnableMonitoring.Checked;
                _missionConfig.Save();
                if (_chkEnableMonitoring.Checked)
                {
                    _monitor?.StartMonitoring();
                }
                else
                {
                    _monitor?.StopMonitoring();
                }
            };
            AddRow(body, _chkEnableMonitoring);
        }

        private void AddAdvisoryAltitudeThreshold(TableLayoutPanel body)
        {
            _nudMaxAlt = ControlFactory.Numeric(0, 10000,
                (decimal)Math.Max(0,
                    Math.Min(10000, _missionConfig.AdvisoryAltitudeDisplayThresholdMeters)), width: 70);
            _nudMaxAlt.ValueChanged += (s, e) =>
            {
                _missionConfig.AdvisoryAltitudeDisplayThresholdMeters = (double)_nudMaxAlt.Value;
                _missionConfig.Save();
                UpdateAdvisoryAltitude(ReadCurrentAltitude());
            };
            AddRow(body, Row(
                Lbl("MP altitude color threshold:", TEXT_PRIMARY),
                _nudMaxAlt,
                Lbl("m telemetry · advisory only", TEXT_SECONDARY)));
            AddRow(body, new Label
            {
                Text = "Uses Mission Planner-reported altitude; it does not set or verify an AGL limit.",
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                ForeColor = TEXT_SECONDARY,
                AutoSize = true,
                MaximumSize = new Size(320, 0),
            });
        }

        private void AddAdvisoryAudioToggle(TableLayoutPanel body)
        {
            _chkAdvisoryAudio = new CheckBox
            {
                Text = "Speak when position leaves a saved outline",
                ForeColor = Color.White,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL),
                AutoSize = true,
                Checked = _missionConfig.AdvisoryAudioAlertsEnabled,
            };
            _chkAdvisoryAudio.CheckedChanged += (s, e) =>
            {
                _missionConfig.AdvisoryAudioAlertsEnabled = _chkAdvisoryAudio.Checked;
                _missionConfig.Save();
            };
            AddRow(body, _chkAdvisoryAudio);
        }

        private void AddMigrationNotice(TableLayoutPanel body)
        {
            var migrationNotice = string.Join(" ", new[]
            {
                _missionConfig.MigrationNotice,
                _presetMigrationNotice,
            }.Where(value => !string.IsNullOrWhiteSpace(value)));
            if (string.IsNullOrWhiteSpace(migrationNotice))
            {
                return;
            }

            _lblMigrationNotice = new Label
            {
                Text = migrationNotice,
                Font = NOMADTheme.Font(NOMADTheme.SIZE_SMALL, FontStyle.Bold),
                ForeColor = Color.Gold,
                AutoSize = true,
                MaximumSize = new Size(320, 0),
            };
            AddRow(body, _lblMigrationNotice);
        }

        private string FormatAltitudeUnavailable() =>
            $"MP alt: -- / {_missionConfig.AdvisoryAltitudeDisplayThresholdMeters:F0}m reference";

        private double? ReadCurrentAltitude()
        {
            var state = MainV2.comPort?.MAV?.cs;
            if (state == null)
            {
                return null;
            }

            double altitude = state.alt;
            return double.IsNaN(altitude) || double.IsInfinity(altitude)
                ? (double?)null
                : altitude;
        }

        private void UpdateAdvisoryAltitude(double? altitudeMeters)
        {
            if (!altitudeMeters.HasValue)
            {
                _lblAltitude.Text = FormatAltitudeUnavailable();
                _lblAltitude.ForeColor = Color.White;
                return;
            }

            double threshold = _missionConfig.AdvisoryAltitudeDisplayThresholdMeters;
            _lblAltitude.Text = $"MP alt: {altitudeMeters.Value:F1}m / {threshold:F0}m reference";
            _lblAltitude.ForeColor = AdvisoryAltitudeStatus.IsAboveThreshold(altitudeMeters.Value, threshold)
                ? Color.Red
                : Color.White;
        }
    }
}
