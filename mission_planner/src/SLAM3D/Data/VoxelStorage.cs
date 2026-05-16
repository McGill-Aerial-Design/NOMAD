// ============================================================
// VoxelStorage.cs - Voxel data storage with occlusion tracking
// ============================================================
// Handles voxel persistence, eviction, and dynamic object removal.
// Tracks when voxels are "unseen" to remove objects that have moved.
// ============================================================

using System;
using System.Collections.Generic;
using System.Linq;
using NOMAD.MissionPlanner.SLAM3D.Models;

namespace NOMAD.MissionPlanner.SLAM3D.Data
{
    /// <summary>
    /// Simple voxel coordinate and color data.
    /// </summary>
    public struct VoxelData
    {
        public int X;
        public int Y;
        public int Z;
        public uint Color;
    }

    /// <summary>
    /// Stores voxel data with LRU eviction and occlusion-based removal.
    /// </summary>
    public class VoxelStorage
    {
        // ==================== Configuration ====================
        
        /// <summary>Maximum number of persisted voxels before eviction.</summary>
        public int MaxVoxels { get; set; } = 5000;
        
        /// <summary>Voxel expires after this many updates without being seen.</summary>
        public int MaxAgeUpdates { get; set; } = 10;
        
        /// <summary>Current voxel size in meters (used for key packing).</summary>
        public double VoxelSize { get; set; } = 0.05;
        
        // ==================== Storage ====================
        
        // Color by packed voxel key
        private readonly Dictionary<long, uint> _voxels = new Dictionary<long, uint>();
        
        // Last update generation when voxel was seen
        private readonly Dictionary<long, int> _lastSeen = new Dictionary<long, int>();
        
        // Unseen count for occlusion-based removal
        private readonly Dictionary<long, int> _unseenCount = new Dictionary<long, int>();
        
        // LRU eviction queue. Each entry records the generation it was enqueued
        // at so re-observations can supersede older entries: an old (key, gen)
        // pair is "stale" if _lastSeen[key] != gen and must be skipped on
        // eviction, otherwise re-seen voxels would be evicted as if they were
        // never touched (FIFO instead of LRU).
        private readonly Queue<(long Key, int Gen)> _insertionOrder = new Queue<(long, int)>();
        private readonly HashSet<long> _queuedForEviction = new HashSet<long>();
        
        // Occupancy set for quick existence checks
        private readonly HashSet<long> _occupancySet = new HashSet<long>();
        
        // Update generation counter
        private int _generation = 0;
        
        // Thread safety
        private readonly object _lock = new object();
        
        // Statistics
        private int _totalAdded;
        private int _totalEvicted;
        private int _totalRemovedByOcclusion;

        // ==================== Constructor ====================

        /// <summary>
        /// Create voxel storage with specified capacity.
        /// </summary>
        /// <param name="maxVoxels">Maximum voxels before LRU eviction</param>
        public VoxelStorage(int maxVoxels = 5000)
        {
            MaxVoxels = maxVoxels;
        }
        
        // ==================== Properties ====================
        
        /// <summary>Current number of stored voxels.</summary>
        public int Count { get { lock (_lock) return _voxels.Count; } }
        
        /// <summary>Current mesh generation number.</summary>
        public int Generation { get { lock (_lock) return _generation; } }
        
        // ==================== Events ====================
        
        /// <summary>Fired when storage changes (for mesh rebuild).</summary>
        public event Action OnStorageChanged;
        
        // ==================== Public Methods ====================
        
        /// <summary>
        /// Add or update voxels from mesh data.
        /// </summary>
        /// <param name="meshData">Mesh data containing voxels</param>
        /// <returns>Number of new voxels added</returns>
        public int ProcessMeshUpdate(MeshDataModel meshData)
        {
            if (meshData == null) return 0;
            
            lock (_lock)
            {
                _generation++;
                int added = 0;
                
                // Handle clear request
                if (meshData.Clear)
                {
                    ClearInternal();
                    return 0;
                }
                
                // Update voxel size if provided
                if (meshData.VoxelSize > 0)
                {
                    VoxelSize = meshData.VoxelSize;
                }
                
                // Process removed voxels first (dynamic object removal)
                if (meshData.Removed != null && meshData.Removed.Count > 0)
                {
                    foreach (var removed in meshData.Removed)
                    {
                        long key = PackKey(removed.X, removed.Y, removed.Z);
                        if (_voxels.Remove(key))
                        {
                            _lastSeen.Remove(key);
                            _unseenCount.Remove(key);
                            _occupancySet.Remove(key);
                            _totalRemovedByOcclusion++;
                        }
                    }
                }
                
                // Process new/updated voxels
                if (meshData.Voxels != null)
                {
                    foreach (var voxel in meshData.Voxels)
                    {
                        if (voxel.Position == null || voxel.Position.Count < 3) continue;
                        
                        // Convert world position to grid coordinates
                        int gx = (int)Math.Round(voxel.Position[0] / VoxelSize);
                        int gy = (int)Math.Round(voxel.Position[1] / VoxelSize);
                        int gz = (int)Math.Round(voxel.Position[2] / VoxelSize);
                        long key = PackKey(gx, gy, gz);
                        
                        // Pack color (default gray if no color)
                        uint color = 0xFF808080;
                        if (voxel.Color != null && voxel.Color.Count >= 3)
                        {
                            byte r = (byte)MathHelper.Clamp(voxel.Color[0], 0, 255);
                            byte g = (byte)MathHelper.Clamp(voxel.Color[1], 0, 255);
                            byte b = (byte)MathHelper.Clamp(voxel.Color[2], 0, 255);
                            color = (uint)((0xFF << 24) | (b << 16) | (g << 8) | r);
                        }
                        
                        bool isNew = !_voxels.ContainsKey(key);
                        _voxels[key] = color;
                        _lastSeen[key] = _generation;
                        _unseenCount[key] = 0; // Reset unseen count when seen

                        // Enqueue every observation (new or re-seen). The
                        // generation tag lets EvictOldVoxels skip stale entries
                        // so re-observed voxels move to the back of the LRU.
                        _insertionOrder.Enqueue((key, _generation));

                        if (isNew)
                        {
                            _occupancySet.Add(key);
                            added++;
                            _totalAdded++;
                        }
                    }
                }
                
                // Evict old voxels if over capacity
                EvictOldVoxels();
                
                // Expire voxels not seen recently
                ExpireStaleVoxels();
                
                if (added > 0 || (meshData.Removed?.Count ?? 0) > 0)
                {
                    OnStorageChanged?.Invoke();
                }
                
                return added;
            }
        }
        
        /// <summary>
        /// Mark voxels as "unseen" because camera can see past them.
        /// Voxels with high unseen count will be removed.
        /// </summary>
        /// <param name="visibleButEmptyKeys">Voxel keys where camera sees empty space</param>
        public void MarkUnseenVoxels(IEnumerable<long> visibleButEmptyKeys)
        {
            lock (_lock)
            {
                var toRemove = new List<long>();
                
                foreach (var key in visibleButEmptyKeys)
                {
                    if (!_voxels.ContainsKey(key)) continue;
                    
                    if (!_unseenCount.TryGetValue(key, out int count))
                    {
                        count = 0;
                    }
                    
                    count++;
                    _unseenCount[key] = count;
                    
                    // Remove after being unseen for 3 consecutive updates
                    if (count >= 3)
                    {
                        toRemove.Add(key);
                    }
                }
                
                foreach (var key in toRemove)
                {
                    _voxels.Remove(key);
                    _lastSeen.Remove(key);
                    _unseenCount.Remove(key);
                    _occupancySet.Remove(key);
                    _totalRemovedByOcclusion++;
                }
                
                if (toRemove.Count > 0)
                {
                    OnStorageChanged?.Invoke();
                }
            }
        }
        
        /// <summary>
        /// Get all voxels for rendering.
        /// </summary>
        /// <returns>Dictionary of packed key -> packed color</returns>
        public Dictionary<long, uint> GetVoxels()
        {
            lock (_lock)
            {
                return new Dictionary<long, uint>(_voxels);
            }
        }
        
        /// <summary>
        /// Get voxel count for status display.
        /// </summary>
        public (int current, int total, int evicted, int occlusionRemoved) GetStats()
        {
            lock (_lock)
            {
                return (_voxels.Count, _totalAdded, _totalEvicted, _totalRemovedByOcclusion);
            }
        }
        
        /// <summary>
        /// Clear all voxels.
        /// </summary>
        public void Clear()
        {
            lock (_lock)
            {
                ClearInternal();
                OnStorageChanged?.Invoke();
            }
        }
        
        /// <summary>
        /// Check if a voxel exists at the given grid coordinates.
        /// </summary>
        public bool Contains(int gx, int gy, int gz)
        {
            long key = PackKey(gx, gy, gz);
            lock (_lock)
            {
                return _occupancySet.Contains(key);
            }
        }
        
        /// <summary>
        /// Unpack a voxel key to grid coordinates.
        /// </summary>
        // Coordinate offset for packing: half of the 20-bit range so that a
        // signed coordinate in [-0x80000, 0x7FFFF] survives a round-trip
        // through the 20-bit mask. Using 0x100000 here (the full range) wraps
        // positive inputs to negative on unpack.
        private const int PACK_OFFSET = 0x80000;
        private const long PACK_MASK = 0xFFFFF;

        public static (int x, int y, int z) UnpackKey(long key)
        {
            int x = (int)((key >> 40) & PACK_MASK) - PACK_OFFSET;
            int y = (int)((key >> 20) & PACK_MASK) - PACK_OFFSET;
            int z = (int)(key & PACK_MASK) - PACK_OFFSET;
            return (x, y, z);
        }
        
        /// <summary>
        /// Unpack a color uint to RGB.
        /// </summary>
        public static (byte r, byte g, byte b) UnpackColor(uint color)
        {
            byte r = (byte)(color & 0xFF);
            byte g = (byte)((color >> 8) & 0xFF);
            byte b = (byte)((color >> 16) & 0xFF);
            return (r, g, b);
        }

        /// <summary>
        /// Unpack key with out parameters (for VoxelRenderer delegate).
        /// </summary>
        public static void UnpackKey(long key, out int x, out int y, out int z)
        {
            x = (int)((key >> 40) & PACK_MASK) - PACK_OFFSET;
            y = (int)((key >> 20) & PACK_MASK) - PACK_OFFSET;
            z = (int)(key & PACK_MASK) - PACK_OFFSET;
        }

        /// <summary>
        /// Get voxel dictionary for renderer.
        /// </summary>
        public Dictionary<long, uint> GetVoxelDictionary()
        {
            lock (_lock)
            {
                return new Dictionary<long, uint>(_voxels);
            }
        }

        /// <summary>
        /// Add voxels from coordinate/color data.
        /// </summary>
        public void AddVoxels(IEnumerable<VoxelData> voxels)
        {
            if (voxels == null) return;

            lock (_lock)
            {
                _generation++;
                int added = 0;

                foreach (var v in voxels)
                {
                    long key = PackKey(v.X, v.Y, v.Z);
                    bool isNew = !_voxels.ContainsKey(key);

                    _voxels[key] = v.Color;
                    _lastSeen[key] = _generation;
                    _unseenCount[key] = 0;

                    _insertionOrder.Enqueue((key, _generation));

                    if (isNew)
                    {
                        _occupancySet.Add(key);
                        added++;
                        _totalAdded++;
                    }
                }

                if (added > 0)
                {
                    EvictOldVoxels();
                    OnStorageChanged?.Invoke();
                }
            }
        }

        /// <summary>
        /// Remove voxels at specified coordinates.
        /// </summary>
        public void RemoveVoxels(IEnumerable<(int X, int Y, int Z)> coordinates)
        {
            if (coordinates == null) return;

            lock (_lock)
            {
                int removed = 0;

                foreach (var (x, y, z) in coordinates)
                {
                    long key = PackKey(x, y, z);
                    if (_voxels.Remove(key))
                    {
                        _lastSeen.Remove(key);
                        _unseenCount.Remove(key);
                        _occupancySet.Remove(key);
                        _totalRemovedByOcclusion++;
                        removed++;
                    }
                }

                if (removed > 0)
                {
                    OnStorageChanged?.Invoke();
                }
            }
        }
        
        // ==================== Private Methods ====================
        
        private void ClearInternal()
        {
            _voxels.Clear();
            _lastSeen.Clear();
            _unseenCount.Clear();
            _insertionOrder.Clear();
            _queuedForEviction.Clear();
            _occupancySet.Clear();
            _generation = 0;
        }
        
        private static long PackKey(int x, int y, int z)
        {
            long lx = (long)(x + PACK_OFFSET) & PACK_MASK;
            long ly = (long)(y + PACK_OFFSET) & PACK_MASK;
            long lz = (long)(z + PACK_OFFSET) & PACK_MASK;
            return (lx << 40) | (ly << 20) | lz;
        }
        
        private void EvictOldVoxels()
        {
            while (_voxels.Count > MaxVoxels && _insertionOrder.Count > 0)
            {
                var (oldKey, oldGen) = _insertionOrder.Dequeue();

                // Skip if already removed
                if (!_occupancySet.Contains(oldKey)) continue;

                // Skip stale LRU entry — the voxel was re-observed at a later
                // generation, so a fresher entry exists deeper in the queue.
                if (_lastSeen.TryGetValue(oldKey, out int lastGen) && lastGen != oldGen)
                {
                    continue;
                }

                _voxels.Remove(oldKey);
                _lastSeen.Remove(oldKey);
                _unseenCount.Remove(oldKey);
                _occupancySet.Remove(oldKey);
                _totalEvicted++;
            }

            // Drain dead/stale entries from the queue so it doesn't grow
            // unbounded when occlusion removes voxels or re-observations leave
            // superseded entries behind.
            int maxQueue = Math.Max(MaxVoxels, _voxels.Count * 2);
            int scanned = 0;
            int scanLimit = _insertionOrder.Count;
            while (_insertionOrder.Count > maxQueue && scanned < scanLimit)
            {
                var (checkKey, checkGen) = _insertionOrder.Dequeue();
                scanned++;
                if (!_occupancySet.Contains(checkKey)) continue;
                if (_lastSeen.TryGetValue(checkKey, out int lastGen) && lastGen != checkGen)
                {
                    continue; // stale duplicate — drop
                }
                _insertionOrder.Enqueue((checkKey, checkGen));
            }
        }
        
        private void ExpireStaleVoxels()
        {
            if (MaxAgeUpdates <= 0) return;
            
            int threshold = _generation - MaxAgeUpdates;
            var staleKeys = _lastSeen
                .Where(kv => kv.Value < threshold)
                .Select(kv => kv.Key)
                .ToList();
            
            foreach (var key in staleKeys)
            {
                _voxels.Remove(key);
                _lastSeen.Remove(key);
                _unseenCount.Remove(key);
                _occupancySet.Remove(key);
                _totalEvicted++;
            }
        }
    }
}
