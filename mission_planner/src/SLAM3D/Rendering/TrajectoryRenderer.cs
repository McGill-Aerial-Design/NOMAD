// ==========================================================
// TrajectoryRenderer.cs - Flight Path Visualization
// ==========================================================

using System;
using System.Collections.Generic;
using OpenTK.Graphics.OpenGL;

namespace NOMAD.MissionPlanner.SLAM3D.Rendering
{
    /// <summary>
    /// Renders the drone's flight trajectory as a colored line strip.
    /// Supports color gradients based on age or altitude.
    /// </summary>
    public class TrajectoryRenderer
    {
        // Trajectory data (OpenGL coords: X-right, Y-up, Z-toward)
        private readonly List<float[]> _points = new List<float[]>();
        private readonly object _lock = new object();

        // Configuration
        private int _maxPoints = 500;
        private float _lineWidth = 2.0f;
        private bool _showTrajectory = true;
        private bool _useAgeGradient = true;

        // Colors
        private float _startR = 0.2f, _startG = 0.5f, _startB = 1.0f;  // Blue (oldest)
        private float _endR = 0.2f, _endG = 1.0f, _endB = 0.5f;        // Green (newest)

        /// <summary>Maximum number of trajectory points to keep.</summary>
        public int MaxPoints
        {
            get => _maxPoints;
            set => _maxPoints = Math.Max(10, Math.Min(5000, value));
        }

        /// <summary>Line width in pixels.</summary>
        public float LineWidth
        {
            get => _lineWidth;
            set => _lineWidth = MathHelper.Clamp(value, 0.5f, 10f);
        }

        /// <summary>Whether to render the trajectory.</summary>
        public bool ShowTrajectory
        {
            get => _showTrajectory;
            set => _showTrajectory = value;
        }

        /// <summary>Use color gradient from old to new points.</summary>
        public bool UseAgeGradient
        {
            get => _useAgeGradient;
            set => _useAgeGradient = value;
        }

        /// <summary>Current point count.</summary>
        public int PointCount
        {
            get { lock (_lock) return _points.Count; }
        }

        /// <summary>
        /// Add a new trajectory point.
        /// </summary>
        /// <param name="x">X position (OpenGL coords)</param>
        /// <param name="y">Y position (OpenGL coords)</param>
        /// <param name="z">Z position (OpenGL coords)</param>
        /// <param name="minDistanceM">Minimum distance from last point to add</param>
        public void AddPoint(float x, float y, float z, float minDistanceM = 0.05f)
        {
            lock (_lock)
            {
                // Skip if too close to last point
                if (_points.Count > 0)
                {
                    var last = _points[_points.Count - 1];
                    float dx = x - last[0];
                    float dy = y - last[1];
                    float dz = z - last[2];
                    float dist = MathHelper.Sqrt(dx * dx + dy * dy + dz * dz);
                    if (dist < minDistanceM) return;
                }

                _points.Add(new[] { x, y, z });

                // Trim if over capacity
                while (_points.Count > _maxPoints)
                {
                    _points.RemoveAt(0);
                }
            }
        }

        /// <summary>
        /// Add a trajectory point from ROS map/odom frame coordinates.
        /// Converts to OpenGL: gx = -y, gy = z, gz = -x
        /// </summary>
        public void AddPointFromRos(float rosX, float rosY, float rosZ, float minDistanceM = 0.05f)
        {
            float gx = -rosY;
            float gy = rosZ;
            float gz = -rosX;
            AddPoint(gx, gy, gz, minDistanceM);
        }

        /// <summary>
        /// Clear all trajectory points.
        /// </summary>
        public void Clear()
        {
            lock (_lock)
            {
                _points.Clear();
            }
        }

        /// <summary>
        /// Render the trajectory line strip.
        /// </summary>
        public void Render()
        {
            if (!_showTrajectory) return;

            float[][] snapshot;
            lock (_lock)
            {
                if (_points.Count < 2) return;
                snapshot = _points.ToArray();
            }

            GL.Disable(EnableCap.Lighting);
            GL.LineWidth(_lineWidth);
            GL.Begin(PrimitiveType.LineStrip);

            int count = snapshot.Length;
            for (int i = 0; i < count; i++)
            {
                if (_useAgeGradient)
                {
                    float t = (float)i / Math.Max(1, count - 1);
                    float r = _startR + (_endR - _startR) * t;
                    float g = _startG + (_endG - _startG) * t;
                    float b = _startB + (_endB - _startB) * t;
                    GL.Color3(r, g, b);
                }
                else
                {
                    GL.Color3(_endR, _endG, _endB);
                }

                var pt = snapshot[i];
                GL.Vertex3(pt[0], pt[1], pt[2]);
            }

            GL.End();
            GL.LineWidth(1.0f);
        }

        /// <summary>
        /// Render vertical drop lines from trajectory to floor.
        /// </summary>
        /// <param name="floorY">Y coordinate of floor plane</param>
        public void RenderDropLines(float floorY = 0f)
        {
            if (!_showTrajectory) return;

            float[][] snapshot;
            lock (_lock)
            {
                if (_points.Count < 2) return;
                snapshot = _points.ToArray();
            }

            GL.Disable(EnableCap.Lighting);
            GL.LineWidth(1.0f);
            GL.Begin(PrimitiveType.Lines);
            GL.Color4(0.3f, 0.3f, 0.4f, 0.3f);

            // Draw drop lines every N points
            int step = Math.Max(1, snapshot.Length / 20);
            for (int i = 0; i < snapshot.Length; i += step)
            {
                var pt = snapshot[i];
                GL.Vertex3(pt[0], pt[1], pt[2]);
                GL.Vertex3(pt[0], floorY, pt[2]);
            }

            GL.End();
        }

        /// <summary>
        /// Set trajectory color gradient.
        /// </summary>
        /// <param name="startR">Start color R (oldest)</param>
        /// <param name="startG">Start color G</param>
        /// <param name="startB">Start color B</param>
        /// <param name="endR">End color R (newest)</param>
        /// <param name="endG">End color G</param>
        /// <param name="endB">End color B</param>
        public void SetGradientColors(float startR, float startG, float startB,
                                       float endR, float endG, float endB)
        {
            _startR = startR; _startG = startG; _startB = startB;
            _endR = endR; _endG = endG; _endB = endB;
        }

        /// <summary>
        /// Get trajectory bounds (min/max for each axis).
        /// </summary>
        public bool GetBounds(out float minX, out float maxX,
                               out float minY, out float maxY,
                               out float minZ, out float maxZ)
        {
            minX = maxX = minY = maxY = minZ = maxZ = 0;

            lock (_lock)
            {
                if (_points.Count == 0) return false;

                minX = maxX = _points[0][0];
                minY = maxY = _points[0][1];
                minZ = maxZ = _points[0][2];

                foreach (var pt in _points)
                {
                    if (pt[0] < minX) minX = pt[0];
                    if (pt[0] > maxX) maxX = pt[0];
                    if (pt[1] < minY) minY = pt[1];
                    if (pt[1] > maxY) maxY = pt[1];
                    if (pt[2] < minZ) minZ = pt[2];
                    if (pt[2] > maxZ) maxZ = pt[2];
                }
            }

            return true;
        }
    }
}
