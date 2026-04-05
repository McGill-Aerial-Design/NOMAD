// ============================================================
// DroneRenderer.cs - Draws the drone model in 3D view
// ============================================================

using System;
using OpenTK.Graphics.OpenGL;

namespace NOMAD.MissionPlanner.SLAM3D.Rendering
{
    /// <summary>
    /// Renders the drone model (tricopter) with orientation.
    /// </summary>
    public class DroneRenderer
    {
        // ==================== Configuration ====================
        
        /// <summary>Drone body length in meters.</summary>
        public float Length { get; set; } = 0.35f;
        
        /// <summary>Drone body width in meters.</summary>
        public float Width { get; set; } = 0.35f;
        
        /// <summary>Drone body height in meters.</summary>
        public float Height { get; set; } = 0.08f;
        
        /// <summary>Motor arm length in meters.</summary>
        public float ArmLength { get; set; } = 0.18f;
        
        /// <summary>Motor radius in meters.</summary>
        public float MotorRadius { get; set; } = 0.03f;
        
        /// <summary>Heading offset in degrees (for camera vs drone orientation).</summary>
        public float HeadingOffsetDeg { get; set; } = 0f;
        
        /// <summary>Show heading indicator arrow.</summary>
        public bool ShowHeadingIndicator { get; set; } = true;
        
        /// <summary>Scale factor for entire model.</summary>
        public float Scale { get; set; } = 1.0f;
        
        // ==================== Public Methods ====================
        
        /// <summary>
        /// Draw the drone at the specified position and orientation.
        /// </summary>
        /// <param name="x">X position (OpenGL frame)</param>
        /// <param name="y">Y position (OpenGL frame)</param>
        /// <param name="z">Z position (OpenGL frame)</param>
        /// <param name="roll">Roll angle in radians</param>
        /// <param name="pitch">Pitch angle in radians</param>
        /// <param name="yaw">Yaw angle in radians</param>
        public void Draw(float x, float y, float z, float roll, float pitch, float yaw)
        {
            GL.PushMatrix();
            
            // Position
            GL.Translate(x, y, z);
            
            // Rotation (apply heading offset, then yaw, pitch, roll)
            float totalYaw = yaw + HeadingOffsetDeg * MathHelper.PI / 180f;
            float yawDeg = totalYaw * 180f / MathHelper.PI;
            float pitchDeg = pitch * 180f / MathHelper.PI;
            float rollDeg = roll * 180f / MathHelper.PI;
            
            GL.Rotate(yawDeg, 0, 1, 0);   // Yaw (around Y-up)
            GL.Rotate(pitchDeg, 1, 0, 0); // Pitch (around X)
            GL.Rotate(rollDeg, 0, 0, 1);  // Roll (around Z-forward)
            
            // Scale
            GL.Scale(Scale, Scale, Scale);
            
            // Draw components
            DrawBody();
            DrawMotorArms();
            DrawMotors();
            
            if (ShowHeadingIndicator)
            {
                DrawHeadingIndicator();
            }
            
            GL.PopMatrix();
        }
        
        // ==================== Private Methods ====================
        
        private void DrawBody()
        {
            float hx = Length / 2;
            float hy = Height / 2;
            float hz = Width / 2;
            
            // Main body - dark gray
            GL.Color3(0.3f, 0.3f, 0.35f);
            
            GL.Begin(PrimitiveType.Quads);
            
            // Top
            GL.Normal3(0, 1, 0);
            GL.Vertex3(-hx, hy, -hz);
            GL.Vertex3(hx, hy, -hz);
            GL.Vertex3(hx, hy, hz);
            GL.Vertex3(-hx, hy, hz);
            
            // Bottom
            GL.Normal3(0, -1, 0);
            GL.Vertex3(-hx, -hy, hz);
            GL.Vertex3(hx, -hy, hz);
            GL.Vertex3(hx, -hy, -hz);
            GL.Vertex3(-hx, -hy, -hz);
            
            // Front (positive X)
            GL.Color3(0.2f, 0.5f, 0.2f); // Green tint for front
            GL.Normal3(1, 0, 0);
            GL.Vertex3(hx, -hy, -hz);
            GL.Vertex3(hx, -hy, hz);
            GL.Vertex3(hx, hy, hz);
            GL.Vertex3(hx, hy, -hz);
            
            // Back
            GL.Color3(0.3f, 0.3f, 0.35f);
            GL.Normal3(-1, 0, 0);
            GL.Vertex3(-hx, -hy, hz);
            GL.Vertex3(-hx, -hy, -hz);
            GL.Vertex3(-hx, hy, -hz);
            GL.Vertex3(-hx, hy, hz);
            
            // Right
            GL.Normal3(0, 0, 1);
            GL.Vertex3(-hx, -hy, hz);
            GL.Vertex3(-hx, hy, hz);
            GL.Vertex3(hx, hy, hz);
            GL.Vertex3(hx, -hy, hz);
            
            // Left
            GL.Normal3(0, 0, -1);
            GL.Vertex3(hx, -hy, -hz);
            GL.Vertex3(hx, hy, -hz);
            GL.Vertex3(-hx, hy, -hz);
            GL.Vertex3(-hx, -hy, -hz);
            
            GL.End();
        }
        
        private void DrawMotorArms()
        {
            float armWidth = 0.02f;
            float armHeight = 0.015f;
            
            GL.Color3(0.25f, 0.25f, 0.25f);
            
            // Front-right arm
            DrawArm(Length / 2, 0, Width / 4, 30f, armWidth, armHeight);
            
            // Front-left arm
            DrawArm(Length / 2, 0, -Width / 4, -30f, armWidth, armHeight);
            
            // Rear arm (center for tricopter)
            DrawArm(-Length / 2, 0, 0, 180f, armWidth, armHeight);
        }
        
        private void DrawArm(float startX, float startY, float startZ, float angleDeg, float width, float height)
        {
            GL.PushMatrix();
            GL.Translate(startX, startY, startZ);
            GL.Rotate(angleDeg, 0, 1, 0);
            
            float hw = width / 2;
            float hh = height / 2;
            
            GL.Begin(PrimitiveType.Quads);
            
            // Top
            GL.Vertex3(0, hh, -hw);
            GL.Vertex3(ArmLength, hh, -hw);
            GL.Vertex3(ArmLength, hh, hw);
            GL.Vertex3(0, hh, hw);
            
            // Bottom
            GL.Vertex3(0, -hh, hw);
            GL.Vertex3(ArmLength, -hh, hw);
            GL.Vertex3(ArmLength, -hh, -hw);
            GL.Vertex3(0, -hh, -hw);
            
            // Front
            GL.Vertex3(ArmLength, -hh, -hw);
            GL.Vertex3(ArmLength, -hh, hw);
            GL.Vertex3(ArmLength, hh, hw);
            GL.Vertex3(ArmLength, hh, -hw);
            
            // Sides
            GL.Vertex3(0, -hh, hw);
            GL.Vertex3(0, hh, hw);
            GL.Vertex3(ArmLength, hh, hw);
            GL.Vertex3(ArmLength, -hh, hw);
            
            GL.Vertex3(ArmLength, -hh, -hw);
            GL.Vertex3(ArmLength, hh, -hw);
            GL.Vertex3(0, hh, -hw);
            GL.Vertex3(0, -hh, -hw);
            
            GL.End();
            GL.PopMatrix();
        }
        
        private void DrawMotors()
        {
            // Motor positions (tricopter layout)
            float frontX = Length / 2 + ArmLength * MathHelper.Cos(30f * MathHelper.PI / 180f);
            float frontZ = ArmLength * MathHelper.Sin(30f * MathHelper.PI / 180f);
            float rearX = -Length / 2 - ArmLength;
            
            // Front-right motor
            DrawMotor(frontX, Height / 2 + 0.01f, frontZ + Width / 4);
            
            // Front-left motor
            DrawMotor(frontX, Height / 2 + 0.01f, -frontZ - Width / 4);
            
            // Rear motor
            DrawMotor(rearX, Height / 2 + 0.01f, 0);
        }
        
        private void DrawMotor(float x, float y, float z)
        {
            const int segments = 12;
            
            GL.PushMatrix();
            GL.Translate(x, y, z);
            
            // Motor body
            GL.Color3(0.2f, 0.2f, 0.2f);
            GL.Begin(PrimitiveType.TriangleFan);
            GL.Vertex3(0, 0.02f, 0);
            for (int i = 0; i <= segments; i++)
            {
                float angle = i * 2 * MathHelper.PI / segments;
                GL.Vertex3(MotorRadius * MathHelper.Cos(angle), 0, MotorRadius * MathHelper.Sin(angle));
            }
            GL.End();
            
            // Prop disc (translucent)
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            GL.Color4(0.5f, 0.5f, 0.5f, 0.3f);
            
            float propRadius = MotorRadius * 3;
            GL.Begin(PrimitiveType.TriangleFan);
            GL.Vertex3(0, 0.025f, 0);
            for (int i = 0; i <= segments * 2; i++)
            {
                float angle = i * 2 * MathHelper.PI / (segments * 2);
                GL.Vertex3(propRadius * MathHelper.Cos(angle), 0.025f, propRadius * MathHelper.Sin(angle));
            }
            GL.End();
            
            GL.Disable(EnableCap.Blend);
            GL.PopMatrix();
        }
        
        private void DrawHeadingIndicator()
        {
            // Arrow pointing forward (positive X in drone frame)
            float arrowLength = Length * 0.8f;
            float arrowWidth = 0.02f;
            float arrowHeadLength = 0.06f;
            float arrowHeadWidth = 0.05f;
            
            GL.Color3(1.0f, 0.3f, 0.3f); // Red arrow
            
            GL.Begin(PrimitiveType.Triangles);
            
            // Arrow shaft
            GL.Vertex3(0, Height / 2 + 0.01f, -arrowWidth);
            GL.Vertex3(arrowLength - arrowHeadLength, Height / 2 + 0.01f, -arrowWidth);
            GL.Vertex3(arrowLength - arrowHeadLength, Height / 2 + 0.01f, arrowWidth);
            
            GL.Vertex3(0, Height / 2 + 0.01f, -arrowWidth);
            GL.Vertex3(arrowLength - arrowHeadLength, Height / 2 + 0.01f, arrowWidth);
            GL.Vertex3(0, Height / 2 + 0.01f, arrowWidth);
            
            // Arrow head
            GL.Vertex3(arrowLength - arrowHeadLength, Height / 2 + 0.01f, -arrowHeadWidth);
            GL.Vertex3(arrowLength, Height / 2 + 0.01f, 0);
            GL.Vertex3(arrowLength - arrowHeadLength, Height / 2 + 0.01f, arrowHeadWidth);
            
            GL.End();
        }
    }
}
