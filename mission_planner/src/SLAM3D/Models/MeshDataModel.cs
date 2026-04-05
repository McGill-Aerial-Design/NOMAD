// ============================================================
// MeshDataModel.cs - Data structures for SLAM mesh
// ============================================================

using System.Collections.Generic;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner.SLAM3D.Models
{
    /// <summary>
    /// Root model for mesh data received from Edge Core WebSocket.
    /// </summary>
    public class MeshDataModel
    {
        [JsonProperty("blocks")]
        public List<MeshBlockModel> Blocks { get; set; }
        
        [JsonProperty("block_size")]
        public double BlockSize { get; set; }
        
        [JsonProperty("total_blocks")]
        public int TotalBlocks { get; set; }
        
        [JsonProperty("mode")]
        public string Mode { get; set; }
        
        [JsonProperty("timestamp")]
        public double Timestamp { get; set; }
        
        [JsonProperty("frame_id")]
        public string FrameId { get; set; }
        
        [JsonProperty("clear")]
        public bool Clear { get; set; }
        
        [JsonProperty("voxels")]
        public List<VoxelModel> Voxels { get; set; }
        
        [JsonProperty("voxel_size")]
        public double VoxelSize { get; set; }
        
        [JsonProperty("total_voxels")]
        public int TotalVoxels { get; set; }
        
        [JsonProperty("removed")]
        public List<RemovedVoxelModel> Removed { get; set; }
    }

    /// <summary>
    /// Individual voxel with position and optional color.
    /// </summary>
    public class VoxelModel
    {
        [JsonProperty("p")]
        public List<double> Position { get; set; }
        
        [JsonProperty("c")]
        public List<int> Color { get; set; }
    }

    /// <summary>
    /// Voxel that has been removed (for dynamic object tracking).
    /// </summary>
    public class RemovedVoxelModel
    {
        [JsonProperty("x")]
        public int X { get; set; }
        
        [JsonProperty("y")]
        public int Y { get; set; }
        
        [JsonProperty("z")]
        public int Z { get; set; }
    }

    /// <summary>
    /// Block of mesh data (legacy format, kept for compatibility).
    /// </summary>
    public class MeshBlockModel
    {
        [JsonProperty("index")]
        public List<int> Index { get; set; }
        
        [JsonProperty("i")]
        private List<int> CompactIndex
        {
            set
            {
                if ((Index == null || Index.Count == 0) && value != null)
                    Index = value;
            }
        }
        
        [JsonProperty("color")]
        public List<int> Color { get; set; }
        
        [JsonProperty("c")]
        private List<int> CompactColor
        {
            set
            {
                if ((Color == null || Color.Count == 0) && value != null)
                    Color = value;
            }
        }
        
        [JsonProperty("vertices")]
        public List<List<double>> Vertices { get; set; }
        
        [JsonProperty("v")]
        private List<List<double>> CompactVertices
        {
            set
            {
                if ((Vertices == null || Vertices.Count == 0) && value != null)
                    Vertices = value;
            }
        }
        
        [JsonProperty("triangles")]
        public List<int> Triangles { get; set; }
        
        [JsonProperty("t")]
        private List<int> CompactTriangles
        {
            set
            {
                if ((Triangles == null || Triangles.Count == 0) && value != null)
                    Triangles = value;
            }
        }
    }
}
