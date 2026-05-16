// ============================================================
// VoxelMeshBuilder.cs - Voxel mesh storage, building & rendering
// ============================================================
// Handles voxel/block ingestion from MeshDataModel, persisted
// storage with LRU eviction, face-culled mesh rebuild with
// debounce, gap filling, and GL rendering via pinned arrays.
// ============================================================

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using NOMAD.MissionPlanner.SLAM3D.Models;
using OpenTK.Graphics.OpenGL;

namespace NOMAD.MissionPlanner.SLAM3D.Rendering
{
    /// <summary>
    /// Manages voxel data storage and produces face-culled GL vertex arrays.
    /// Thread-safe: data ingestion can happen on a background thread; rendering
    /// reads the latest vertex snapshot on the GL thread.
    /// </summary>
    public class VoxelMeshBuilder
    {
        // ---- Configuration (operational defaults; relaxed in ParityMode) ----
        private int _maxPersistedVoxels = 5000;
        private int _voxelMaxAge = 10;
        private double _renderCubeScale = 1.01;
        private TimeSpan _minRebuildInterval = TimeSpan.FromMilliseconds(250);
        private bool _parityMode;

        /// <summary>
        /// Minimum brightness for voxels at maximum age (0=fully dark, 1=no decay).
        /// Voxels fade linearly from full brightness when fresh to this value just
        /// before they expire, so the map dims gracefully rather than blinking out.
        /// </summary>
        public float DecayMinBrightness { get; set; } = 0.2f;

        /// <summary>
        /// Parity mode for RViz validation runs. When enabled:
        ///   - Retention cap is raised far above normal so map density matches RViz.
        ///   - Age-based expiry is disabled so voxels persist as long as RViz would show them.
        ///   - Render cube scale is exactly 1.0 (no quantization padding).
        ///   - Rebuild debounce drops to ~16ms so mesh updates are reflected immediately.
        /// Must be set before mesh ingestion begins to take effect on all geometry.
        /// </summary>
        public bool ParityMode
        {
            get { lock (_meshLock) return _parityMode; }
            set
            {
                lock (_meshLock)
                {
                    _parityMode = value;
                    if (value)
                    {
                        _maxPersistedVoxels = int.MaxValue;
                        _voxelMaxAge = int.MaxValue;
                        _renderCubeScale = 1.0;
                        _minRebuildInterval = TimeSpan.FromMilliseconds(16);
                    }
                    else
                    {
                        _maxPersistedVoxels = 5000;
                        _voxelMaxAge = 10;
                        _renderCubeScale = 1.01;
                        _minRebuildInterval = TimeSpan.FromMilliseconds(250);
                    }
                }
            }
        }

        // ---- Voxel storage ----
        private Dictionary<long, uint> _persistedBlocks = new Dictionary<long, uint>();
        private Dictionary<long, int> _voxelLastSeen = new Dictionary<long, int>();
        private int _meshGeneration;
        private Queue<long> _voxelInsertionOrder = new Queue<long>();
        private HashSet<long> _queuedForEviction = new HashSet<long>();
        private double _currentVoxelSize = 0.05;
        private string _currentMeshMode = "";
        private float _focusCenterX;
        private float _focusCenterY;
        private float _focusCenterZ;
        private float _focusRadiusM = 3f;
        private bool _focusSphereEnabled;

        // ---- GL vertex data (rebuilt when mesh changes) ----
        private float[] _voxelVerts;
        private int[] _voxelIndices;
        private int _voxelIndexCount;

        // ---- Mesh rebuild tracking ----
        private bool _meshDirty;
        private int _lastRenderedCount;
        private bool _pendingMeshUpdate;
        private long _lastMeshRebuildStamp = -1;
        private DateTime _lastMeshRebuild = DateTime.MinValue;

        // ---- Thread safety ----
        private readonly object _meshLock = new object();

        // ---- Statistics ----
        /// <summary>Current number of persisted voxels.</summary>
        public int VoxelCount { get { lock (_meshLock) return _persistedBlocks.Count; } }

        // ==================== Public API ====================

        /// <summary>
        /// Process incoming mesh data (call from WebSocket receive thread).
        /// </summary>
        public void UpdateMesh(MeshDataModel meshData)
        {
            try
            {
                lock (_meshLock)
                {
                    if (meshData?.Clear == true)
                    {
                        ClearInternal();
                    }

                    if (meshData?.Removed != null)
                    {
                        foreach (var r in meshData.Removed)
                        {
                            int rx = -r.Y, ry = r.Z, rz = -r.X;
                            var key = PackVoxelKey(rx, ry, rz);
                            if (_persistedBlocks.Remove(key))
                            {
                                UnqueueForEviction(key);
                                _meshDirty = true;
                            }
                        }
                    }

                    if ((meshData?.Mode == "voxel" || meshData?.Mode == "voxels") &&
                        meshData.Voxels != null && meshData.Voxels.Count > 0)
                    {
                        ProcessVoxels(meshData);
                    }
                    else if (meshData?.Blocks != null && meshData.Blocks.Count > 0)
                    {
                        ProcessBlocks(meshData);
                    }

                    if (_meshDirty)
                    {
                        _pendingMeshUpdate = true;
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Mesh update error: {ex.Message}");
            }
        }

        /// <summary>
        /// Process pending mesh rebuild if debounce window has elapsed.
        /// Call from GL render thread before drawing.
        /// </summary>
        public void ProcessPendingRebuild()
        {
            lock (_meshLock)
            {
                if (!_pendingMeshUpdate || !_meshDirty)
                {
                    _pendingMeshUpdate = false;
                    return;
                }

                long elapsedMs = ElapsedMsSince(_lastMeshRebuildStamp);
                if (elapsedMs >= (long)_minRebuildInterval.TotalMilliseconds)
                {
                    if (_focusSphereEnabled && !_parityMode)
                    {
                        CullOutsideFocusSphere();
                    }

                    // Preserve exact nvblox geometry; do not synthesize gap voxels.
                    RebuildVoxelMesh();
                    _pendingMeshUpdate = false;
                }
            }
        }

        /// <summary>
        /// Keep only voxels inside a local sphere centered on the current pose.
        /// </summary>
        public void SetFocusSphere(float centerX, float centerY, float centerZ, float radiusMeters)
        {
            lock (_meshLock)
            {
                float clampedRadius = Math.Max(1f, radiusMeters);
                bool changed = !_focusSphereEnabled
                    || Math.Abs(_focusCenterX - centerX) > 0.01f
                    || Math.Abs(_focusCenterY - centerY) > 0.01f
                    || Math.Abs(_focusCenterZ - centerZ) > 0.01f
                    || Math.Abs(_focusRadiusM - clampedRadius) > 0.01f;

                _focusSphereEnabled = true;
                _focusCenterX = centerX;
                _focusCenterY = centerY;
                _focusCenterZ = centerZ;
                _focusRadiusM = clampedRadius;

                if (changed && !_parityMode)
                {
                    _meshDirty = true;
                    _pendingMeshUpdate = true;
                }
            }
        }

        /// <summary>
        /// Render the voxel mesh using pinned client-side vertex arrays.
        /// Call on GL thread after ProcessPendingRebuild().
        /// </summary>
        public void Render()
        {
            if (_voxelVerts == null || _voxelIndexCount == 0) return;

            GL.Enable(EnableCap.Lighting);
            GL.EnableClientState(ArrayCap.VertexArray);
            GL.EnableClientState(ArrayCap.ColorArray);
            GL.EnableClientState(ArrayCap.NormalArray);

            int stride = 9 * sizeof(float);
            var vertHandle = GCHandle.Alloc(_voxelVerts, GCHandleType.Pinned);
            var idxHandle = GCHandle.Alloc(_voxelIndices, GCHandleType.Pinned);
            try
            {
                IntPtr basePtr = vertHandle.AddrOfPinnedObject();
                GL.VertexPointer(3, VertexPointerType.Float, stride, basePtr);
                GL.ColorPointer(3, ColorPointerType.Float, stride, basePtr + 3 * sizeof(float));
                GL.NormalPointer(NormalPointerType.Float, stride, basePtr + 6 * sizeof(float));
                GL.DrawElements(PrimitiveType.Triangles, _voxelIndexCount,
                    DrawElementsType.UnsignedInt, idxHandle.AddrOfPinnedObject());
            }
            finally
            {
                idxHandle.Free();
                vertHandle.Free();
            }

            GL.DisableClientState(ArrayCap.NormalArray);
            GL.DisableClientState(ArrayCap.ColorArray);
            GL.DisableClientState(ArrayCap.VertexArray);
        }

        /// <summary>Clear all voxel data and vertex arrays.</summary>
        public void Clear()
        {
            lock (_meshLock)
            {
                ClearInternal();
            }
        }

        // ==================== Private: Data Ingestion ====================

        private void ProcessVoxels(MeshDataModel meshData)
        {
            // Parity mode preserves the exact server-reported voxel size; the
            // default path rounds to 2 decimals for cache-friendly grid keys.
            double incomingSize = meshData.VoxelSize > 0 ? meshData.VoxelSize : 0.15;
            double vs = _parityMode ? incomingSize : NormalizeCellSize(incomingSize);

            if (_currentMeshMode != "voxel")
            {
                ClearInternal();
                _currentMeshMode = "voxel";
            }
            _currentVoxelSize = vs;
            _meshGeneration++;

            foreach (var voxel in meshData.Voxels)
            {
                if (voxel.Position == null || voxel.Position.Count < 3) continue;

                // ROS→GL coordinate conversion at quantization.
                // Use Floor (not Round) for consistent grid mapping: every
                // position in [n*vs, (n+1)*vs) maps to grid index n.
                // Round(AwayFromZero) skips grid cell 0 when positions
                // straddle zero (-0.5 → -1, +0.5 → +1), creating visible gaps.
                int qx = (int)Math.Floor(-voxel.Position[1] / vs);
                int qy = (int)Math.Floor(voxel.Position[2] / vs);
                int qz = (int)Math.Floor(-voxel.Position[0] / vs);
                long key = PackVoxelKey(qx, qy, qz);

                uint colorKey = PackColor(voxel.Color);
                _voxelLastSeen[key] = _meshGeneration;

                if (!_persistedBlocks.ContainsKey(key) || _persistedBlocks[key] != colorKey)
                {
                    if (!_persistedBlocks.ContainsKey(key))
                        QueueForEviction(key);
                    _persistedBlocks[key] = colorKey;
                    _meshDirty = true;
                }
            }

            ExpireOldVoxels();
        }

        private void ProcessBlocks(MeshDataModel meshData)
        {
            double incomingBlockSize = meshData.BlockSize > 0 ? meshData.BlockSize : 0.05;
            double bs = _parityMode ? incomingBlockSize : NormalizeCellSize(incomingBlockSize);

            if (_currentMeshMode != "block")
            {
                ClearInternal();
                _currentMeshMode = "block";
            }
            _currentVoxelSize = bs;
            _meshGeneration++;

            foreach (var block in meshData.Blocks)
            {
                if (block.Index == null || block.Index.Count < 3) continue;

                int bix = -block.Index[1];
                int biy = block.Index[2];
                int biz = -block.Index[0];
                long key = PackVoxelKey(bix, biy, biz);

                uint colorKey = PackColor(block.Color);
                _voxelLastSeen[key] = _meshGeneration;

                if (!_persistedBlocks.ContainsKey(key) || _persistedBlocks[key] != colorKey)
                {
                    if (!_persistedBlocks.ContainsKey(key))
                        QueueForEviction(key);
                    _persistedBlocks[key] = colorKey;
                    _meshDirty = true;
                }
            }

            ExpireOldVoxels();
            EvictOldVoxels();
        }

        private static double NormalizeCellSize(double size)
        {
            if (size <= 0.0) return 0.15;
            return Math.Round(size, 2, MidpointRounding.AwayFromZero);
        }

        private static uint PackColor(List<int> color)
        {
            if (color != null && color.Count >= 3)
            {
                byte r = (byte)Math.Min(255, Math.Max(0, color[0]));
                byte g = (byte)Math.Min(255, Math.Max(0, color[1]));
                byte b = (byte)Math.Min(255, Math.Max(0, color[2]));
                return ((uint)r << 16) | ((uint)g << 8) | (uint)b;
            }
            return uint.MaxValue;
        }

        // ==================== Private: Mesh Building ====================

        private void RebuildVoxelMesh()
        {
            _lastMeshRebuild = DateTime.UtcNow;
            _lastMeshRebuildStamp = Stopwatch.GetTimestamp();
            _meshDirty = false;
            _lastRenderedCount = _persistedBlocks.Count;

            double vs = _currentVoxelSize;
            float half = (float)(vs * 0.5 * _renderCubeScale);

            var verts = new List<float>(_persistedBlocks.Count * 100);
            var indices = new List<int>(_persistedBlocks.Count * 36);
            int vertOffset = 0;

            foreach (var kvp in _persistedBlocks)
            {
                UnpackVoxelKey(kvp.Key, out int ix, out int iy, out int iz);
                // Center cube in its grid cell: floor-based quantization maps
                // [n*vs, (n+1)*vs) to index n, so the cell center is (n+0.5)*vs.
                float cx = (float)((ix + 0.5) * vs);
                float cy = (float)((iy + 0.5) * vs);
                float cz = (float)((iz + 0.5) * vs);

                uint ck = kvp.Value;
                float cr, cg, cb;
                if (ck == uint.MaxValue) { cr = 0.56f; cg = 0.56f; cb = 0.63f; }
                else
                {
                    cr = ((ck >> 16) & 0xFF) / 255f;
                    cg = ((ck >> 8) & 0xFF) / 255f;
                    cb = (ck & 0xFF) / 255f;
                }

                // Age-based brightness decay: fade voxels toward DecayMinBrightness
                // as they approach _voxelMaxAge without being refreshed by nvblox.
                if (!_parityMode && _voxelMaxAge > 0 && _voxelMaxAge != int.MaxValue)
                {
                    int lastSeen = _voxelLastSeen.TryGetValue(kvp.Key, out int ls) ? ls : 0;
                    float ageFrac = Math.Max(0f, Math.Min(1f, (float)(_meshGeneration - lastSeen) / _voxelMaxAge));
                    float brightness = 1f - ageFrac * (1f - DecayMinBrightness);
                    cr *= brightness; cg *= brightness; cb *= brightness;
                }

                // Face-culled cube: only emit faces not adjacent to another voxel
                if (!_persistedBlocks.ContainsKey(PackVoxelKey(ix + 1, iy, iz)))
                    AddQuad(verts, indices, ref vertOffset, cx + half, cy - half, cz - half, cx + half, cy + half, cz - half, cx + half, cy + half, cz + half, cx + half, cy - half, cz + half, cr, cg, cb, 1, 0, 0);
                if (!_persistedBlocks.ContainsKey(PackVoxelKey(ix - 1, iy, iz)))
                    AddQuad(verts, indices, ref vertOffset, cx - half, cy - half, cz + half, cx - half, cy + half, cz + half, cx - half, cy + half, cz - half, cx - half, cy - half, cz - half, cr, cg, cb, -1, 0, 0);
                if (!_persistedBlocks.ContainsKey(PackVoxelKey(ix, iy + 1, iz)))
                    AddQuad(verts, indices, ref vertOffset, cx - half, cy + half, cz - half, cx - half, cy + half, cz + half, cx + half, cy + half, cz + half, cx + half, cy + half, cz - half, cr, cg, cb, 0, 1, 0);
                if (!_persistedBlocks.ContainsKey(PackVoxelKey(ix, iy - 1, iz)))
                    AddQuad(verts, indices, ref vertOffset, cx - half, cy - half, cz + half, cx - half, cy - half, cz - half, cx + half, cy - half, cz - half, cx + half, cy - half, cz + half, cr, cg, cb, 0, -1, 0);
                if (!_persistedBlocks.ContainsKey(PackVoxelKey(ix, iy, iz + 1)))
                    AddQuad(verts, indices, ref vertOffset, cx - half, cy - half, cz + half, cx + half, cy - half, cz + half, cx + half, cy + half, cz + half, cx - half, cy + half, cz + half, cr, cg, cb, 0, 0, 1);
                if (!_persistedBlocks.ContainsKey(PackVoxelKey(ix, iy, iz - 1)))
                    AddQuad(verts, indices, ref vertOffset, cx + half, cy - half, cz - half, cx - half, cy - half, cz - half, cx - half, cy + half, cz - half, cx + half, cy + half, cz - half, cr, cg, cb, 0, 0, -1);
            }

            _voxelVerts = verts.ToArray();
            _voxelIndices = indices.ToArray();
            _voxelIndexCount = indices.Count;
        }

        private static void AddQuad(List<float> verts, List<int> indices, ref int offset,
            float x0, float y0, float z0, float x1, float y1, float z1,
            float x2, float y2, float z2, float x3, float y3, float z3,
            float r, float g, float b, float nx, float ny, float nz)
        {
            verts.Add(x0); verts.Add(y0); verts.Add(z0); verts.Add(r); verts.Add(g); verts.Add(b); verts.Add(nx); verts.Add(ny); verts.Add(nz);
            verts.Add(x1); verts.Add(y1); verts.Add(z1); verts.Add(r); verts.Add(g); verts.Add(b); verts.Add(nx); verts.Add(ny); verts.Add(nz);
            verts.Add(x2); verts.Add(y2); verts.Add(z2); verts.Add(r); verts.Add(g); verts.Add(b); verts.Add(nx); verts.Add(ny); verts.Add(nz);
            verts.Add(x3); verts.Add(y3); verts.Add(z3); verts.Add(r); verts.Add(g); verts.Add(b); verts.Add(nx); verts.Add(ny); verts.Add(nz);
            indices.Add(offset); indices.Add(offset + 1); indices.Add(offset + 2);
            indices.Add(offset); indices.Add(offset + 2); indices.Add(offset + 3);
            offset += 4;
        }

        // ==================== Private: Gap Filling ====================

        private void FillGaps()
        {
            var fills = new Dictionary<long, uint>();

            foreach (var kvp in _persistedBlocks)
            {
                UnpackVoxelKey(kvp.Key, out int ix, out int iy, out int iz);
                CheckAndFill(fills, ix, iy, iz, 1, 0, 0, kvp.Value);
                CheckAndFill(fills, ix, iy, iz, 0, 1, 0, kvp.Value);
                CheckAndFill(fills, ix, iy, iz, 0, 0, 1, kvp.Value);
            }

            foreach (var kvp in fills)
            {
                if (!_persistedBlocks.ContainsKey(kvp.Key))
                    QueueForEviction(kvp.Key);
                _persistedBlocks[kvp.Key] = kvp.Value;
            }

            if (_currentMeshMode == "block")
                EvictOldVoxels();
        }

        private void CheckAndFill(Dictionary<long, uint> fills, int ix, int iy, int iz,
            int dx, int dy, int dz, uint c1)
        {
            int nx = ix + dx, ny = iy + dy, nz = iz + dz;
            long nk = PackVoxelKey(nx, ny, nz);
            if (_persistedBlocks.ContainsKey(nk) || fills.ContainsKey(nk)) return;

            int fx = ix + dx * 2, fy = iy + dy * 2, fz = iz + dz * 2;
            long fk = PackVoxelKey(fx, fy, fz);
            uint c2;
            if (!_persistedBlocks.TryGetValue(fk, out c2)) return;

            if (c1 == uint.MaxValue) c1 = c2;
            if (c2 == uint.MaxValue) c2 = c1;
            uint r = (((c1 >> 16) & 0xFF) + ((c2 >> 16) & 0xFF)) / 2;
            uint g = (((c1 >> 8) & 0xFF) + ((c2 >> 8) & 0xFF)) / 2;
            uint b = ((c1 & 0xFF) + (c2 & 0xFF)) / 2;
            fills[nk] = (r << 16) | (g << 8) | b;
        }

        // ==================== Private: Eviction / Expiration ====================

        private void EvictOldVoxels()
        {
            if (_parityMode) return; // Parity mode: never cap retention.
            while (_persistedBlocks.Count > _maxPersistedVoxels && _voxelInsertionOrder.Count > 0)
            {
                long key = _voxelInsertionOrder.Dequeue();
                _queuedForEviction.Remove(key);
                if (_persistedBlocks.Remove(key))
                {
                    _voxelLastSeen.Remove(key);
                    _meshDirty = true;
                }
            }
        }

        private void ExpireOldVoxels()
        {
            if (_parityMode) return; // Parity mode: never age voxels out.
            int cutoff = _meshGeneration - _voxelMaxAge;
            if (cutoff < 0) return;

            var expired = new List<long>();
            foreach (var kvp in _voxelLastSeen)
            {
                if (kvp.Value < cutoff)
                    expired.Add(kvp.Key);
            }

            bool compactQueue = false;
            foreach (var key in expired)
            {
                _voxelLastSeen.Remove(key);
                if (_persistedBlocks.Remove(key))
                {
                    if (_queuedForEviction.Remove(key))
                        compactQueue = true;
                    _meshDirty = true;
                }
            }

            if (compactQueue && _voxelInsertionOrder.Count > 0)
            {
                var rebuilt = new Queue<long>(_voxelInsertionOrder.Count);
                while (_voxelInsertionOrder.Count > 0)
                {
                    long queued = _voxelInsertionOrder.Dequeue();
                    if (_queuedForEviction.Contains(queued))
                        rebuilt.Enqueue(queued);
                }
                _voxelInsertionOrder = rebuilt;
            }
        }

        // ==================== Private: Key Helpers ====================

        private static long PackVoxelKey(int ix, int iy, int iz)
        {
            return ((long)(ix + 32768) << 32) | ((long)(iy + 32768) << 16) | (long)(iz + 32768);
        }

        private static void UnpackVoxelKey(long key, out int ix, out int iy, out int iz)
        {
            iz = (int)((key & 0xFFFF) - 32768);
            iy = (int)(((key >> 16) & 0xFFFF) - 32768);
            ix = (int)(((key >> 32) & 0xFFFF) - 32768);
        }

        // ==================== Private: Eviction Queue Helpers ====================

        private void QueueForEviction(long key)
        {
            // Parity mode disables both EvictOldVoxels and ExpireOldVoxels, so any
            // keys queued here would never be drained -- _voxelInsertionOrder would
            // grow unbounded for the duration of a parity validation run.
            if (_parityMode) return;
            if (_queuedForEviction.Add(key))
                _voxelInsertionOrder.Enqueue(key);
        }

        private void UnqueueForEviction(long key)
        {
            if (!_queuedForEviction.Remove(key) || _voxelInsertionOrder.Count == 0)
                return;
            var rebuilt = new Queue<long>(_voxelInsertionOrder.Count);
            while (_voxelInsertionOrder.Count > 0)
            {
                long queued = _voxelInsertionOrder.Dequeue();
                if (queued != key)
                    rebuilt.Enqueue(queued);
            }
            _voxelInsertionOrder = rebuilt;
        }

        private void ClearInternal()
        {
            _persistedBlocks.Clear();
            _voxelInsertionOrder.Clear();
            _queuedForEviction.Clear();
            _voxelLastSeen.Clear();
            _voxelVerts = null;
            _voxelIndices = null;
            _voxelIndexCount = 0;
            _meshDirty = false;
            _pendingMeshUpdate = false;
            _lastRenderedCount = 0;
        }

        private void CullOutsideFocusSphere()
        {
            double radiusSq = _focusRadiusM * _focusRadiusM;
            var toRemove = new List<long>();

            foreach (var kvp in _persistedBlocks)
            {
                UnpackVoxelKey(kvp.Key, out int ix, out int iy, out int iz);
                float cx = (float)((ix + 0.5) * _currentVoxelSize);
                float cy = (float)((iy + 0.5) * _currentVoxelSize);
                float cz = (float)((iz + 0.5) * _currentVoxelSize);

                double dx = cx - _focusCenterX;
                double dy = cy - _focusCenterY;
                double dz = cz - _focusCenterZ;
                if ((dx * dx) + (dy * dy) + (dz * dz) > radiusSq)
                {
                    toRemove.Add(kvp.Key);
                }
            }

            if (toRemove.Count == 0)
                return;

            foreach (var key in toRemove)
            {
                _persistedBlocks.Remove(key);
                _voxelLastSeen.Remove(key);
                UnqueueForEviction(key);
            }

            _meshDirty = true;
        }

        private static long ElapsedMsSince(long startStamp)
        {
            if (startStamp < 0) return long.MaxValue;
            long elapsedTicks = Stopwatch.GetTimestamp() - startStamp;
            return (elapsedTicks * 1000) / Stopwatch.Frequency;
        }
    }
}
