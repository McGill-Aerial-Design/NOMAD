// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ==========================================================
// VoxelRenderer.cs - Voxel/Block Mesh Rendering
// ==========================================================

using System;
using System.Collections.Generic;
using OpenTK.Graphics.OpenGL;

namespace NOMAD.MissionPlanner.SLAM3D.Rendering
{
    /// <summary>
    /// Delegate for unpacking voxel keys into coordinates.
    /// </summary>
    public delegate void UnpackVoxelKeyDelegate(long key, out int ix, out int iy, out int iz);

    /// <summary>
    /// Renders voxel data as colored cubes with optional lighting.
    /// Optimized for batch rendering of large voxel counts.
    /// </summary>
    public class VoxelRenderer
    {
        // Vertex data (interleaved: pos(3) + color(3) + normal(3) = 9 floats/vertex)
        private float[] _vertexData;
        private int[] _indexData;
        private int _indexCount;

        // OpenGL buffers
        private int _vbo = -1;
        private int _ibo = -1;
        private bool _buffersInitialized;
        private bool _dataUploaded;

        // Rendering options
        private bool _enableLighting = true;
        private float _ambientStrength = 0.35f;
        private float _voxelScale = 1.0f;

        // Statistics
        private int _lastVoxelCount;
        private int _lastVertexCount;
        private int _lastIndexCount;

        /// <summary>Whether to apply lighting to voxels.</summary>
        public bool EnableLighting
        {
            get => _enableLighting;
            set => _enableLighting = value;
        }

        /// <summary>Ambient light strength (0-1).</summary>
        public float AmbientStrength
        {
            get => _ambientStrength;
            set => _ambientStrength = MathHelper.Clamp(value, 0f, 1f);
        }

        /// <summary>Voxel size scale factor.</summary>
        public float VoxelScale
        {
            get => _voxelScale;
            set => _voxelScale = Math.Max(0.1f, Math.Min(5f, value));
        }

        /// <summary>Last rendered voxel count.</summary>
        public int LastVoxelCount => _lastVoxelCount;

        /// <summary>Last vertex count.</summary>
        public int LastVertexCount => _lastVertexCount;

        /// <summary>
        /// Build vertex data from voxel dictionary.
        /// </summary>
        /// <param name="voxels">Dictionary of voxel key -> packed color (0xRRGGBB)</param>
        /// <param name="voxelSize">Size of each voxel in world units</param>
        /// <param name="unpackKey">Function to unpack key into (ix, iy, iz)</param>
        public void BuildMesh(Dictionary<long, uint> voxels, double voxelSize,
                               UnpackVoxelKeyDelegate unpackKey)
        {
            if (voxels == null || voxels.Count == 0)
            {
                // Keep backing buffers allocated for reuse; just zero counts.
                _indexCount = 0;
                _lastVoxelCount = 0;
                _lastVertexCount = 0;
                _lastIndexCount = 0;
                _dataUploaded = false;
                return;
            }

            int count = voxels.Count;
            float size = (float)(voxelSize * _voxelScale);
            float half = size * 0.5f;

            // 24 vertices per cube (4 per face, 6 faces), 9 floats per vertex
            // 36 indices per cube (6 per face, 6 faces)
            int vertFloats = count * 24 * 9;
            int idxInts = count * 36;

            // Reuse backing buffers across rebuilds to keep multi-MB arrays
            // off the LOH. Grow with headroom (1.5x) so steady-state map
            // changes do not reallocate every frame.
            if (_vertexData == null || _vertexData.Length < vertFloats)
            {
                _vertexData = new float[(int)(vertFloats * 1.5)];
            }
            if (_indexData == null || _indexData.Length < idxInts)
            {
                _indexData = new int[(int)(idxInts * 1.5)];
            }

            int vIdx = 0;
            int iIdx = 0;
            int baseVertex = 0;

            foreach (var kvp in voxels)
            {
                int ix, iy, iz;
                unpackKey(kvp.Key, out ix, out iy, out iz);

                // Center position
                float cx = ix * size;
                float cy = iy * size;
                float cz = iz * size;

                // Color extraction
                uint c = kvp.Value;
                float r, g, b;
                if (c == uint.MaxValue)
                {
                    r = g = b = 0.6f; // Default gray
                }
                else
                {
                    r = ((c >> 16) & 0xFF) / 255f;
                    g = ((c >> 8) & 0xFF) / 255f;
                    b = (c & 0xFF) / 255f;
                }

                // Generate cube vertices with face normals
                // Each face has 4 vertices, each vertex has: pos(3) + color(3) + normal(3)

                // Front face (+Z)
                AddFaceVertices(ref vIdx, cx, cy, cz, half, r, g, b, 0, 0, 1);
                AddFaceIndices(ref iIdx, baseVertex, 0, 1, 2, 2, 3, 0);
                baseVertex += 4;

                // Back face (-Z)
                AddFaceVertices(ref vIdx, cx, cy, cz, half, r, g, b, 0, 0, -1);
                AddFaceIndices(ref iIdx, baseVertex, 0, 1, 2, 2, 3, 0);
                baseVertex += 4;

                // Top face (+Y)
                AddFaceVertices(ref vIdx, cx, cy, cz, half, r, g, b, 0, 1, 0);
                AddFaceIndices(ref iIdx, baseVertex, 0, 1, 2, 2, 3, 0);
                baseVertex += 4;

                // Bottom face (-Y)
                AddFaceVertices(ref vIdx, cx, cy, cz, half, r, g, b, 0, -1, 0);
                AddFaceIndices(ref iIdx, baseVertex, 0, 1, 2, 2, 3, 0);
                baseVertex += 4;

                // Right face (+X)
                AddFaceVertices(ref vIdx, cx, cy, cz, half, r, g, b, 1, 0, 0);
                AddFaceIndices(ref iIdx, baseVertex, 0, 1, 2, 2, 3, 0);
                baseVertex += 4;

                // Left face (-X)
                AddFaceVertices(ref vIdx, cx, cy, cz, half, r, g, b, -1, 0, 0);
                AddFaceIndices(ref iIdx, baseVertex, 0, 1, 2, 2, 3, 0);
                baseVertex += 4;
            }

            _indexCount = iIdx;
            _lastVoxelCount = count;
            _lastVertexCount = baseVertex;
            _lastIndexCount = iIdx;
            _dataUploaded = false;
        }

        private void AddFaceVertices(ref int idx, float cx, float cy, float cz, float h,
                                      float r, float g, float b, int nx, int ny, int nz)
        {
            // Calculate 4 corners based on normal direction
            float[] corners = new float[12]; // 4 vertices x 3 coords

            if (nz == 1) // Front (+Z)
            {
                corners = new[] { cx-h, cy-h, cz+h,  cx+h, cy-h, cz+h,  cx+h, cy+h, cz+h,  cx-h, cy+h, cz+h };
            }
            else if (nz == -1) // Back (-Z)
            {
                corners = new[] { cx+h, cy-h, cz-h,  cx-h, cy-h, cz-h,  cx-h, cy+h, cz-h,  cx+h, cy+h, cz-h };
            }
            else if (ny == 1) // Top (+Y)
            {
                corners = new[] { cx-h, cy+h, cz+h,  cx+h, cy+h, cz+h,  cx+h, cy+h, cz-h,  cx-h, cy+h, cz-h };
            }
            else if (ny == -1) // Bottom (-Y)
            {
                corners = new[] { cx-h, cy-h, cz-h,  cx+h, cy-h, cz-h,  cx+h, cy-h, cz+h,  cx-h, cy-h, cz+h };
            }
            else if (nx == 1) // Right (+X)
            {
                corners = new[] { cx+h, cy-h, cz+h,  cx+h, cy-h, cz-h,  cx+h, cy+h, cz-h,  cx+h, cy+h, cz+h };
            }
            else if (nx == -1) // Left (-X)
            {
                corners = new[] { cx-h, cy-h, cz-h,  cx-h, cy-h, cz+h,  cx-h, cy+h, cz+h,  cx-h, cy+h, cz-h };
            }

            // Add 4 vertices for this face
            for (int i = 0; i < 4; i++)
            {
                // Position
                _vertexData[idx++] = corners[i * 3];
                _vertexData[idx++] = corners[i * 3 + 1];
                _vertexData[idx++] = corners[i * 3 + 2];
                // Color
                _vertexData[idx++] = r;
                _vertexData[idx++] = g;
                _vertexData[idx++] = b;
                // Normal
                _vertexData[idx++] = nx;
                _vertexData[idx++] = ny;
                _vertexData[idx++] = nz;
            }
        }

        private void AddFaceIndices(ref int idx, int baseV, int i0, int i1, int i2, int i3, int i4, int i5)
        {
            _indexData[idx++] = baseV + i0;
            _indexData[idx++] = baseV + i1;
            _indexData[idx++] = baseV + i2;
            _indexData[idx++] = baseV + i3;
            _indexData[idx++] = baseV + i4;
            _indexData[idx++] = baseV + i5;
        }

        /// <summary>
        /// Initialize OpenGL buffers. Call once after GL context is available.
        /// </summary>
        public void InitializeBuffers()
        {
            if (_buffersInitialized) return;

            _vbo = GL.GenBuffer();
            _ibo = GL.GenBuffer();
            _buffersInitialized = true;
        }

        /// <summary>
        /// Upload vertex data to GPU if changed.
        /// </summary>
        public void UploadData()
        {
            if (_dataUploaded || _vertexData == null || _indexData == null) return;
            if (!_buffersInitialized) InitializeBuffers();

            // Upload only the used prefix of the reusable backing buffers,
            // not their full capacity.
            int vertFloats = _lastVertexCount * 9;
            int idxInts = _lastIndexCount;

            GL.BindBuffer(BufferTarget.ArrayBuffer, _vbo);
            GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr)(vertFloats * sizeof(float)),
                          _vertexData, BufferUsageHint.DynamicDraw);

            GL.BindBuffer(BufferTarget.ElementArrayBuffer, _ibo);
            GL.BufferData(BufferTarget.ElementArrayBuffer, (IntPtr)(idxInts * sizeof(int)),
                          _indexData, BufferUsageHint.DynamicDraw);

            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, 0);

            _dataUploaded = true;
        }

        /// <summary>
        /// Render the voxel mesh.
        /// </summary>
        public void Render()
        {
            if (_indexCount == 0 || _vertexData == null) return;
            if (!_dataUploaded) UploadData();
            if (!_buffersInitialized) return;

            if (_enableLighting)
            {
                GL.Enable(EnableCap.Lighting);
                GL.Enable(EnableCap.Light0);
                GL.Enable(EnableCap.ColorMaterial);
                GL.ColorMaterial(MaterialFace.FrontAndBack, ColorMaterialParameter.AmbientAndDiffuse);

                float[] ambient = { _ambientStrength, _ambientStrength, _ambientStrength, 1f };
                float[] diffuse = { 0.7f, 0.7f, 0.7f, 1f };
                float[] lightPos = { 0.5f, 1f, 0.8f, 0f }; // Directional light
                GL.Light(LightName.Light0, LightParameter.Ambient, ambient);
                GL.Light(LightName.Light0, LightParameter.Diffuse, diffuse);
                GL.Light(LightName.Light0, LightParameter.Position, lightPos);
            }
            else
            {
                GL.Disable(EnableCap.Lighting);
            }

            GL.BindBuffer(BufferTarget.ArrayBuffer, _vbo);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, _ibo);

            int stride = 9 * sizeof(float);
            GL.EnableClientState(ArrayCap.VertexArray);
            GL.EnableClientState(ArrayCap.ColorArray);
            GL.EnableClientState(ArrayCap.NormalArray);

            GL.VertexPointer(3, VertexPointerType.Float, stride, IntPtr.Zero);
            GL.ColorPointer(3, ColorPointerType.Float, stride, (IntPtr)(3 * sizeof(float)));
            GL.NormalPointer(NormalPointerType.Float, stride, (IntPtr)(6 * sizeof(float)));

            GL.DrawElements(PrimitiveType.Triangles, _indexCount, DrawElementsType.UnsignedInt, IntPtr.Zero);

            GL.DisableClientState(ArrayCap.VertexArray);
            GL.DisableClientState(ArrayCap.ColorArray);
            GL.DisableClientState(ArrayCap.NormalArray);

            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, 0);

            if (_enableLighting)
            {
                GL.Disable(EnableCap.Lighting);
                GL.Disable(EnableCap.Light0);
                GL.Disable(EnableCap.ColorMaterial);
            }
        }

        /// <summary>
        /// Mark data as needing re-upload (call after mesh rebuild).
        /// </summary>
        public void InvalidateData()
        {
            _dataUploaded = false;
        }

        /// <summary>
        /// Clean up OpenGL resources.
        /// </summary>
        public void Dispose()
        {
            if (_buffersInitialized)
            {
                if (_vbo != -1) GL.DeleteBuffer(_vbo);
                if (_ibo != -1) GL.DeleteBuffer(_ibo);
                _vbo = -1;
                _ibo = -1;
                _buffersInitialized = false;
            }
            _vertexData = null;
            _indexData = null;
        }
    }
}
