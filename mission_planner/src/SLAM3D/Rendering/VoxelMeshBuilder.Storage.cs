// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// VoxelMeshBuilder.Storage.cs - Voxel retention bookkeeping
// ============================================================
// Eviction (LRU cap), age-based expiration, voxel key packing,
// eviction-queue maintenance, and clearing. All methods assume
// the caller holds _meshLock.
// ============================================================

using System.Collections.Generic;

namespace NOMAD.MissionPlanner.SLAM3D.Rendering
{
    public partial class VoxelMeshBuilder
    {
        // ==================== Eviction / Expiration ====================

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

        // ==================== Key Helpers ====================

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

        // ==================== Eviction Queue Helpers ====================

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
    }
}
