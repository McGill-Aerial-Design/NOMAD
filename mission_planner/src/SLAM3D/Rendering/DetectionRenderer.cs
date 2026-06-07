// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ==========================================================
// DetectionRenderer.cs - Object Detection Marker Rendering
// ==========================================================

using System;
using System.Collections.Generic;
using OpenTK.Graphics.OpenGL;

namespace NOMAD.MissionPlanner.SLAM3D.Rendering
{
    /// <summary>
    /// Marker data for 3D detection visualization.
    /// </summary>
    public class DetectionMarkerData
    {
        public string Label { get; set; }
        public float X { get; set; }
        public float Y { get; set; }
        public float Z { get; set; }
        public float Confidence { get; set; }
        public int SeenCount { get; set; }
        public bool ColorMatch { get; set; } = true;
        public bool NeedsReview { get; set; }
        public float R { get; set; } = 1.0f;
        public float G { get; set; } = 0.3f;
        public float B { get; set; } = 0.3f;
    }

    /// <summary>
    /// Renders 3D detection markers (bounding boxes, labels, confidence indicators).
    /// </summary>
    public class DetectionRenderer
    {
        // Marker storage
        private readonly List<DetectionMarkerData> _markers = new List<DetectionMarkerData>();
        private readonly object _lock = new object();

        // Configuration
        private float _markerSize = 0.15f;
        private bool _showLabels = true;
        private bool _showConfidence = true;
        private bool _showBoundingBox = true;

        // Colors
        private float _confirmedR = 0.2f, _confirmedG = 1.0f, _confirmedB = 0.3f;  // Green
        private float _reviewR = 1.0f, _reviewG = 0.8f, _reviewB = 0.2f;           // Yellow
        private float _mismatchR = 1.0f, _mismatchG = 0.3f, _mismatchB = 0.3f;     // Red

        /// <summary>Size of marker spheres.</summary>
        public float MarkerSize
        {
            get => _markerSize;
            set => _markerSize = MathHelper.Clamp(value, 0.05f, 1.0f);
        }

        /// <summary>Whether to show labels above markers.</summary>
        public bool ShowLabels
        {
            get => _showLabels;
            set => _showLabels = value;
        }

        /// <summary>Whether to show confidence values.</summary>
        public bool ShowConfidence
        {
            get => _showConfidence;
            set => _showConfidence = value;
        }

        /// <summary>Number of markers.</summary>
        public int MarkerCount
        {
            get { lock (_lock) return _markers.Count; }
        }

        /// <summary>
        /// Update markers from detection list.
        /// </summary>
        public void UpdateMarkers(IEnumerable<DetectionMarkerData> markers)
        {
            lock (_lock)
            {
                _markers.Clear();
                if (markers != null)
                {
                    _markers.AddRange(markers);
                }
            }
        }

        /// <summary>
        /// Add a single marker.
        /// </summary>
        public void AddMarker(DetectionMarkerData marker)
        {
            if (marker == null) return;
            lock (_lock)
            {
                _markers.Add(marker);
            }
        }

        /// <summary>
        /// Clear all markers.
        /// </summary>
        public void Clear()
        {
            lock (_lock)
            {
                _markers.Clear();
            }
        }

        /// <summary>
        /// Render all detection markers.
        /// </summary>
        public void Render()
        {
            DetectionMarkerData[] snapshot;
            lock (_lock)
            {
                if (_markers.Count == 0) return;
                snapshot = _markers.ToArray();
            }

            GL.Disable(EnableCap.Lighting);

            foreach (var m in snapshot)
            {
                // Choose color based on status
                float r, g, b;
                if (!m.ColorMatch)
                {
                    r = _mismatchR; g = _mismatchG; b = _mismatchB;
                }
                else if (m.NeedsReview)
                {
                    r = _reviewR; g = _reviewG; b = _reviewB;
                }
                else
                {
                    r = _confirmedR; g = _confirmedG; b = _confirmedB;
                }

                // Render marker sphere
                RenderSphere(m.X, m.Y, m.Z, _markerSize, r, g, b);

                // Render bounding box (wireframe cube)
                if (_showBoundingBox)
                {
                    RenderWireframeCube(m.X, m.Y, m.Z, _markerSize * 2f, r, g, b, 0.6f);
                }

                // Render vertical line to ground
                GL.Begin(PrimitiveType.Lines);
                GL.Color4(r, g, b, 0.4f);
                GL.Vertex3(m.X, m.Y, m.Z);
                GL.Vertex3(m.X, 0, m.Z);
                GL.End();
            }
        }

        private void RenderSphere(float x, float y, float z, float radius, float r, float g, float b)
        {
            // Simple sphere approximation using stacked circles
            GL.Color3(r, g, b);
            int segments = 12;
            int stacks = 8;

            for (int i = 0; i < stacks; i++)
            {
                float phi1 = MathHelper.PI * i / stacks;
                float phi2 = MathHelper.PI * (i + 1) / stacks;

                GL.Begin(PrimitiveType.TriangleStrip);
                for (int j = 0; j <= segments; j++)
                {
                    float theta = 2 * MathHelper.PI * j / segments;

                    // First vertex
                    float x1 = x + radius * MathHelper.Sin(phi1) * MathHelper.Cos(theta);
                    float y1 = y + radius * MathHelper.Cos(phi1);
                    float z1 = z + radius * MathHelper.Sin(phi1) * MathHelper.Sin(theta);

                    // Second vertex
                    float x2 = x + radius * MathHelper.Sin(phi2) * MathHelper.Cos(theta);
                    float y2 = y + radius * MathHelper.Cos(phi2);
                    float z2 = z + radius * MathHelper.Sin(phi2) * MathHelper.Sin(theta);

                    GL.Vertex3(x1, y1, z1);
                    GL.Vertex3(x2, y2, z2);
                }
                GL.End();
            }
        }

        private void RenderWireframeCube(float x, float y, float z, float size,
                                          float r, float g, float b, float alpha)
        {
            float h = size * 0.5f;
            GL.Color4(r, g, b, alpha);
            GL.LineWidth(1.5f);
            GL.Begin(PrimitiveType.Lines);

            // Bottom face
            GL.Vertex3(x - h, y - h, z - h); GL.Vertex3(x + h, y - h, z - h);
            GL.Vertex3(x + h, y - h, z - h); GL.Vertex3(x + h, y - h, z + h);
            GL.Vertex3(x + h, y - h, z + h); GL.Vertex3(x - h, y - h, z + h);
            GL.Vertex3(x - h, y - h, z + h); GL.Vertex3(x - h, y - h, z - h);

            // Top face
            GL.Vertex3(x - h, y + h, z - h); GL.Vertex3(x + h, y + h, z - h);
            GL.Vertex3(x + h, y + h, z - h); GL.Vertex3(x + h, y + h, z + h);
            GL.Vertex3(x + h, y + h, z + h); GL.Vertex3(x - h, y + h, z + h);
            GL.Vertex3(x - h, y + h, z + h); GL.Vertex3(x - h, y + h, z - h);

            // Vertical edges
            GL.Vertex3(x - h, y - h, z - h); GL.Vertex3(x - h, y + h, z - h);
            GL.Vertex3(x + h, y - h, z - h); GL.Vertex3(x + h, y + h, z - h);
            GL.Vertex3(x + h, y - h, z + h); GL.Vertex3(x + h, y + h, z + h);
            GL.Vertex3(x - h, y - h, z + h); GL.Vertex3(x - h, y + h, z + h);

            GL.End();
            GL.LineWidth(1.0f);
        }

        /// <summary>
        /// Set marker colors by status.
        /// </summary>
        public void SetStatusColors(float confirmedR, float confirmedG, float confirmedB,
                                     float reviewR, float reviewG, float reviewB,
                                     float mismatchR, float mismatchG, float mismatchB)
        {
            _confirmedR = confirmedR; _confirmedG = confirmedG; _confirmedB = confirmedB;
            _reviewR = reviewR; _reviewG = reviewG; _reviewB = reviewB;
            _mismatchR = mismatchR; _mismatchG = mismatchG; _mismatchB = mismatchB;
        }
    }
}
