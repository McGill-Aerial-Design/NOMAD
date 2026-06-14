// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// BuildingViewer3D.UI.cs - Input handling and navigation
// ============================================================

using System;
using System.Drawing;
using System.Windows.Forms;
using OpenTK;

namespace NOMAD.MissionPlanner
{
    public partial class BuildingViewer3D
    {
        // ==================== Mouse + picking ====================

        private void GlControl_MouseDown(object sender, MouseEventArgs e)
        {
            _lastMouse = e.Location;
            _mouseDownPoint = e.Location;
            _dragButton = e.Button;
            _glControl.Focus();
        }

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            var key = keyData & Keys.KeyCode;
            if (IsNavigationKey(key))
            {
                _glControl.Focus();
                SetNavigationKey(key, true);
                return true;
            }
            return base.ProcessCmdKey(ref msg, keyData);
        }

        private void GlControl_PreviewKeyDown(object sender, PreviewKeyDownEventArgs e)
        {
            if (IsNavigationKey(e.KeyCode))
                e.IsInputKey = true;
        }

        private static bool IsNavigationKey(Keys key)
        {
            return key == Keys.W || key == Keys.A || key == Keys.S || key == Keys.D
                || key == Keys.Up || key == Keys.Down || key == Keys.Home;
        }

        private void SetNavigationKey(Keys key, bool down)
        {
            if (_dronePovEnabled)
                return;

            if (key == Keys.Home && down)
            {
                _panTarget = Vector3.Zero;
                _heldKeys.Clear();
                _navTimer.Stop();
                _glControl.Invalidate();
                return;
            }

            if (!IsNavigationKey(key) || key == Keys.Home)
                return;

            if (down) _heldKeys.Add(key);
            else _heldKeys.Remove(key);

            if (_heldKeys.Count > 0 && !_navTimer.Enabled)
                _navTimer.Start();
            else if (_heldKeys.Count == 0 && _navTimer.Enabled)
                _navTimer.Stop();
        }

        private void GlControl_KeyDown(object sender, KeyEventArgs e)
        {
            if (!IsNavigationKey(e.KeyCode)) return;
            SetNavigationKey(e.KeyCode, true);
            e.Handled = true;
            e.SuppressKeyPress = true;
        }

        private void GlControl_KeyUp(object sender, KeyEventArgs e)
        {
            if (!IsNavigationKey(e.KeyCode)) return;
            SetNavigationKey(e.KeyCode, false);
            e.Handled = true;
            e.SuppressKeyPress = true;
        }

        private void NavTimer_Tick(object sender, EventArgs e)
        {
            if (_dronePovEnabled)
            {
                _heldKeys.Clear();
                _navTimer.Stop();
                return;
            }

            if (_heldKeys.Count == 0)
            {
                _navTimer.Stop();
                return;
            }

            float yaw = _yawDeg * (float)Math.PI / 180f;
            Vector3 forward = new Vector3(-(float)Math.Sin(yaw), 0f, (float)Math.Cos(yaw));
            Vector3 right = new Vector3((float)Math.Cos(yaw), 0f, (float)Math.Sin(yaw));
            Vector3 delta = Vector3.Zero;

            if (_heldKeys.Contains(Keys.W)) delta += forward;
            if (_heldKeys.Contains(Keys.S)) delta -= forward;
            if (_heldKeys.Contains(Keys.D)) delta -= right;
            if (_heldKeys.Contains(Keys.A)) delta += right;
            if (_heldKeys.Contains(Keys.Up)) delta += Vector3.UnitY;
            if (_heldKeys.Contains(Keys.Down)) delta -= Vector3.UnitY;

            if (delta.LengthSquared <= 0.0001f)
                return;

            delta.Normalize();
            float speed = Math.Max(1.5f, _distance * 0.55f);
            var modifiers = Control.ModifierKeys;
            if ((modifiers & Keys.Shift) == Keys.Shift) speed *= 3.0f;
            if ((modifiers & Keys.Control) == Keys.Control) speed *= 0.25f;

            _panTarget += delta * speed * (_navTimer.Interval / 1000f);
            _glControl.Invalidate();
        }

        private void GlControl_MouseUp(object sender, MouseEventArgs e)
        {
            if (_dragButton == MouseButtons.Left)
            {
                int dx = e.X - _mouseDownPoint.X;
                int dy = e.Y - _mouseDownPoint.Y;
                bool isClick = dx * dx + dy * dy <= 25;
                if (isClick)
                {
                    if (PlacementMode)
                    {
                        var placement = PickPlacement(e.Location);
                        if (placement != null)
                            try { PlacementClicked?.Invoke(placement); } catch { }
                    }
                    else
                    {
                        string id = PickTarget(e.Location);
                        if (!string.IsNullOrEmpty(id))
                            try { TargetClicked?.Invoke(id); } catch { }
                    }
                }
            }
            _dragButton = MouseButtons.None;
        }

        private void GlControl_MouseMove(object sender, MouseEventArgs e)
        {
            int dx = e.X - _lastMouse.X;
            int dy = e.Y - _lastMouse.Y;
            _lastMouse = e.Location;

            if (_dronePovEnabled)
            {
                if (_dragButton == MouseButtons.None)
                    SetHover(PickTarget(e.Location));
                return;
            }

            if (_dragButton == MouseButtons.Left)
            {
                _yawDeg = (_yawDeg + dx * 0.4f) % 360f;
                _pitchDeg = MathHelperClamp(_pitchDeg + dy * 0.3f, -85f, 85f);
                _glControl.Invalidate();
            }
            else if (_dragButton == MouseButtons.Right || _dragButton == MouseButtons.Middle)
            {
                float scale = _distance * 0.0025f;
                float yaw = _yawDeg * (float)Math.PI / 180f;
                Vector3 right = new Vector3((float)Math.Cos(yaw), 0, (float)Math.Sin(yaw));
                _panTarget -= right * (dx * scale);
                _panTarget += new Vector3(0, dy * scale, 0);
                _glControl.Invalidate();
            }
            else
            {
                var hit = PickTarget(e.Location);
                SetHover(hit);
            }
        }

        private void GlControl_MouseWheel(object sender, MouseEventArgs e)
        {
            if (_dronePovEnabled) return;

            float factor = e.Delta > 0 ? 0.85f : 1.18f;
            _distance = MathHelperClamp(_distance * factor, 4f, 120f);
            _glControl.Invalidate();
        }

        private void SetHover(string id)
        {
            if (_hoverId == id) return;
            _hoverId = id;
            _glControl.Invalidate();
            try { TargetHovered?.Invoke(id); } catch { }
        }
    }
}
