// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ==========================================================
// GridRenderer.cs - 3D Grid Rendering for SLAM3DView
// ==========================================================

using System;
using OpenTK.Graphics.OpenGL;

namespace NOMAD.MissionPlanner.SLAM3D.Rendering
{
    /// <summary>
    /// Renders a 3D reference grid for spatial orientation.
    /// </summary>
    public class GridRenderer
    {
        // Grid configuration
        private int _gridSize = 20;
        private float _cellSize = 1.0f;
        private bool _showGrid = true;

        // Colors (RGB float)
        private float _majorLineR = 0.35f, _majorLineG = 0.35f, _majorLineB = 0.35f;
        private float _minorLineR = 0.22f, _minorLineG = 0.22f, _minorLineB = 0.22f;
        private float _axisXR = 0.8f, _axisXG = 0.2f, _axisXB = 0.2f;  // X-axis red
        private float _axisYR = 0.2f, _axisYG = 0.8f, _axisYB = 0.2f;  // Y-axis green
        private float _axisZR = 0.2f, _axisZG = 0.2f, _axisZB = 0.8f;  // Z-axis blue

        /// <summary>Number of grid cells in each direction from center.</summary>
        public int GridSize
        {
            get => _gridSize;
            set => _gridSize = Math.Max(1, Math.Min(100, value));
        }

        /// <summary>Size of each grid cell in meters.</summary>
        public float CellSize
        {
            get => _cellSize;
            set => _cellSize = Math.Max(0.1f, Math.Min(10f, value));
        }

        /// <summary>Whether to render the grid.</summary>
        public bool ShowGrid
        {
            get => _showGrid;
            set => _showGrid = value;
        }

        /// <summary>
        /// Render the grid at the given center position.
        /// </summary>
        /// <param name="centerX">Grid center X (OpenGL coords)</param>
        /// <param name="centerY">Grid center Y (OpenGL coords) - typically 0</param>
        /// <param name="centerZ">Grid center Z (OpenGL coords)</param>
        public void Render(float centerX, float centerY, float centerZ)
        {
            if (!_showGrid) return;

            GL.Disable(EnableCap.Lighting);
            GL.Begin(PrimitiveType.Lines);

            float halfSize = _gridSize * _cellSize;

            // Floor grid (XZ plane at Y=centerY)
            for (int i = -_gridSize; i <= _gridSize; i++)
            {
                float pos = i * _cellSize;
                bool isMajor = (i % 5 == 0);

                if (isMajor)
                    GL.Color3(_majorLineR, _majorLineG, _majorLineB);
                else
                    GL.Color3(_minorLineR, _minorLineG, _minorLineB);

                // Lines parallel to X-axis
                GL.Vertex3(centerX - halfSize, centerY, centerZ + pos);
                GL.Vertex3(centerX + halfSize, centerY, centerZ + pos);

                // Lines parallel to Z-axis
                GL.Vertex3(centerX + pos, centerY, centerZ - halfSize);
                GL.Vertex3(centerX + pos, centerY, centerZ + halfSize);
            }

            GL.End();

            // Origin axes (thicker lines)
            GL.LineWidth(2.5f);
            GL.Begin(PrimitiveType.Lines);

            // X-axis (red)
            GL.Color3(_axisXR, _axisXG, _axisXB);
            GL.Vertex3(centerX, centerY + 0.01f, centerZ);
            GL.Vertex3(centerX + halfSize * 0.5f, centerY + 0.01f, centerZ);

            // Y-axis (green) - vertical
            GL.Color3(_axisYR, _axisYG, _axisYB);
            GL.Vertex3(centerX, centerY, centerZ);
            GL.Vertex3(centerX, centerY + halfSize * 0.25f, centerZ);

            // Z-axis (blue)
            GL.Color3(_axisZR, _axisZG, _axisZB);
            GL.Vertex3(centerX, centerY + 0.01f, centerZ);
            GL.Vertex3(centerX, centerY + 0.01f, centerZ + halfSize * 0.5f);

            GL.End();
            GL.LineWidth(1.0f);
        }

        /// <summary>
        /// Render centered at origin with optional vertical offset.
        /// </summary>
        public void Render(float floorY = 0f)
        {
            Render(0f, floorY, 0f);
        }

        /// <summary>
        /// Set grid line colors.
        /// </summary>
        public void SetGridColors(float majorR, float majorG, float majorB,
                                   float minorR, float minorG, float minorB)
        {
            _majorLineR = majorR; _majorLineG = majorG; _majorLineB = majorB;
            _minorLineR = minorR; _minorLineG = minorG; _minorLineB = minorB;
        }

        /// <summary>
        /// Set axis colors.
        /// </summary>
        public void SetAxisColors(float xR, float xG, float xB,
                                   float yR, float yG, float yB,
                                   float zR, float zG, float zB)
        {
            _axisXR = xR; _axisXG = xG; _axisXB = xB;
            _axisYR = yR; _axisYG = yG; _axisYB = yB;
            _axisZR = zR; _axisZG = zG; _axisZB = zB;
        }
    }
}
