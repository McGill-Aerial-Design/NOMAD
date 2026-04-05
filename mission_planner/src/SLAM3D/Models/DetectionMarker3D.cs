// ============================================================
// DetectionMarker3D.cs - 3D detection marker model
// ============================================================

namespace NOMAD.MissionPlanner.SLAM3D.Models
{
    /// <summary>
    /// Represents a detected object marker in 3D space.
    /// </summary>
    public class DetectionMarker3D
    {
        /// <summary>Label/class of the detected object.</summary>
        public string Label { get; set; }
        
        /// <summary>X position in world frame (meters).</summary>
        public double X { get; set; }
        
        /// <summary>Y position in world frame (meters).</summary>
        public double Y { get; set; }
        
        /// <summary>Z position in world frame (meters).</summary>
        public double Z { get; set; }
        
        /// <summary>Detection confidence (0-1).</summary>
        public double Confidence { get; set; }
        
        /// <summary>Number of times this object has been seen.</summary>
        public int SeenCount { get; set; }
        
        /// <summary>HSV color string (e.g., "180,50,100").</summary>
        public string HsvColor { get; set; }
        
        /// <summary>Whether the color matches expected target.</summary>
        public bool ColorMatch { get; set; }
        
        /// <summary>Whether this detection needs manual review.</summary>
        public bool NeedsReview { get; set; }
    }
}
