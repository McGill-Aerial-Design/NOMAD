// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Captures unmodified arrow keys before the focused Mission Planner control
    /// consumes them. The filter is plugin-owned, so it keeps working when the
    /// floating gimbal window is closed or covered by another view.
    /// </summary>
    internal sealed class GimbalArrowKeyFilter : IMessageFilter, IDisposable
    {
        private const int WM_KEYDOWN = 0x0100;
        private const int WM_SYSKEYDOWN = 0x0104;
        private const float KEY_NUDGE_DEG = 2.0f;

        private static GimbalArrowKeyFilter s_current;

        private readonly NOMADConfig _config;
        private bool _enabled;
        private bool _disposed;

        public static bool Enabled => s_current?._enabled ?? false;
        public static event Action<bool> EnabledChanged;

        public GimbalArrowKeyFilter(NOMADConfig config)
        {
            _config = config ?? throw new ArgumentNullException(nameof(config));
            _enabled = config.GimbalArrowKeysEnabled;
            s_current = this;
            Application.AddMessageFilter(this);
        }

        public static void SetEnabled(bool enabled)
        {
            if (s_current == null || s_current._enabled == enabled) return;

            s_current._enabled = enabled;
            s_current._config.GimbalArrowKeysEnabled = enabled;
            s_current._config.Save();
            EnabledChanged?.Invoke(enabled);
        }

        public bool PreFilterMessage(ref Message m)
        {
            if (!_enabled || (m.Msg != WM_KEYDOWN && m.Msg != WM_SYSKEYDOWN))
                return false;
            if (Control.ModifierKeys != Keys.None)
                return false;

            return TryHandleKey((Keys)m.WParam.ToInt32());
        }

        internal static bool TryHandleKey(Keys key)
        {
            float pitchDelta = 0f;
            float rollDelta = 0f;
            switch (key & Keys.KeyCode)
            {
                case Keys.Up:
                    pitchDelta = KEY_NUDGE_DEG;
                    break;
                case Keys.Down:
                    pitchDelta = -KEY_NUDGE_DEG;
                    break;
                case Keys.Left:
                    rollDelta = KEY_NUDGE_DEG;
                    break;
                case Keys.Right:
                    rollDelta = -KEY_NUDGE_DEG;
                    break;
                default:
                    return false;
            }

            GimbalController.NudgeTarget(pitchDelta, rollDelta);
            return true;
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            Application.RemoveMessageFilter(this);
            if (ReferenceEquals(s_current, this))
                s_current = null;
        }
    }
}
