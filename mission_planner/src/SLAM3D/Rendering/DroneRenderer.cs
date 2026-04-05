// ============================================================
// DroneRenderer.cs - Draws the drone model in 3D view
// ============================================================
// Supports tricopter and quadcopter frames with servo camera
// mount, avoidance envelope, and configurable dimensions.
// ============================================================

using System;
using OpenTK.Graphics.OpenGL;

namespace NOMAD.MissionPlanner.SLAM3D.Rendering
{
    /// <summary>
    /// Renders the drone model with orientation, servo camera, and avoidance envelope.
    /// Supports tricopter and quadcopter frame types.
    /// </summary>
    public class DroneRenderer
    {
        // ==================== Configuration ====================

        /// <summary>Drone body length in meters.</summary>
        public float LengthM { get; set; } = 0.35f;

        /// <summary>Drone body width in meters.</summary>
        public float WidthM { get; set; } = 0.35f;

        /// <summary>Drone body height in meters.</summary>
        public float HeightM { get; set; } = 0.08f;

        /// <summary>Heading offset in degrees (for camera vs drone orientation).</summary>
        public float HeadingOffsetDeg { get; set; } = 0f;

        /// <summary>Frame type: "Tricopter" or "Quadcopter".</summary>
        public string FrameType { get; set; } = "Tricopter";

        /// <summary>Camera forward offset from center in meters.</summary>
        public float CameraForwardOffsetM { get; set; } = 0f;

        /// <summary>Camera downward offset from center in meters.</summary>
        public float CameraDownOffsetM { get; set; } = 0f;

        /// <summary>Show avoidance envelope wireframe around drone.</summary>
        public bool ShowAvoidanceEnvelope { get; set; } = true;

        /// <summary>Avoidance envelope padding in meters.</summary>
        public float AvoidanceEnvelopePadM { get; set; } = 0.1f;

        // ==================== Public Methods ====================

        /// <summary>
        /// Draw the drone at the specified GL position with ROS attitude and servo angle.
        /// Position should already be in OpenGL frame.
        /// </summary>
        /// <param name="glX">X position (OpenGL frame)</param>
        /// <param name="glY">Y position (OpenGL frame)</param>
        /// <param name="glZ">Z position (OpenGL frame)</param>
        /// <param name="yawRad">Raw yaw in radians (ROS frame)</param>
        /// <param name="pitchRad">Raw pitch in radians (ROS frame)</param>
        /// <param name="rollRad">Raw roll in radians (ROS frame)</param>
        /// <param name="servoDeg">Servo angle in degrees (90=level)</param>
        public void Draw(float glX, float glY, float glZ,
                         float yawRad, float pitchRad, float rollRad,
                         float servoDeg = 90f)
        {
            // Rotation mapping: ROS → OpenGL
            // Model geometry alignment: add 180 deg so model nose matches true heading.
            float headingDeg = (float)(yawRad * 180.0 / Math.PI) + HeadingOffsetDeg + 180f;
            float bodyPitchDeg = (float)(pitchRad * 180.0 / Math.PI);
            float bodyRollDeg = (float)(rollRad * 180.0 / Math.PI);

            GL.Disable(EnableCap.Lighting);
            GL.PushMatrix();
            GL.Translate(glX, glY, glZ);
            GL.Rotate(headingDeg, 0f, 1f, 0f);
            GL.Rotate(bodyPitchDeg, 1f, 0f, 0f);
            GL.Rotate(bodyRollDeg, 0f, 0f, 1f);

            // Draw body based on frame type
            if (FrameType == "Quadcopter")
                DrawQuadcopterBody();
            else
                DrawTricopterBody();

            // Camera servo mount
            DrawServoCamera(servoDeg);

            // Avoidance envelope
            if (ShowAvoidanceEnvelope)
            {
                float pad = AvoidanceEnvelopePadM;
                GL.Color4(0f, 0.86f, 0.86f, 0.2f);
                DrawWireBox(0, 0, 0, LengthM + pad * 2, HeightM + pad * 2, WidthM + pad * 2);
            }

            GL.Enable(EnableCap.Lighting);
            GL.PopMatrix();
        }

        // ==================== Private: Frame Types ====================

        private void DrawTricopterBody()
        {
            GL.Color3(0f, 0.86f, 0.86f); // cyan

            // Central body wireframe
            DrawWireBox(0, 0, 0, WidthM * 0.3f, HeightM, LengthM * 0.4f);

            // Front-left arm
            float frontX = WidthM * 0.4f;
            float frontZ = LengthM * 0.35f;
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0); GL.Vertex3(-frontX, 0, frontZ);
            GL.End();

            // Front-right arm
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0); GL.Vertex3(frontX, 0, frontZ);
            GL.End();

            // Rear arm
            float rearZ = -LengthM * 0.4f;
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0); GL.Vertex3(0, 0, rearZ);
            GL.End();

            // Motor discs
            float motorR = Math.Min(LengthM, WidthM) * 0.15f;
            DrawMotorDisc(frontX, 0, frontZ, motorR);
            DrawMotorDisc(-frontX, 0, frontZ, motorR);
            DrawMotorDisc(0, 0, rearZ, motorR);

            DrawForwardIndicator();
        }

        private void DrawQuadcopterBody()
        {
            GL.Color3(1f, 0.65f, 0f); // orange

            // Central body wireframe
            DrawWireBox(0, 0, 0, WidthM * 0.3f, HeightM, LengthM * 0.3f);

            // X configuration arms
            float armAngle = 45f * (float)Math.PI / 180f;
            float cosA = (float)Math.Cos(armAngle);
            float sinA = (float)Math.Sin(armAngle);

            float flX = -WidthM * 0.4f * cosA, flZ = LengthM * 0.35f * sinA;
            float frX = WidthM * 0.4f * cosA, frZ = LengthM * 0.35f * sinA;
            float rlX = -WidthM * 0.4f * cosA, rlZ = -LengthM * 0.35f * sinA;
            float rrX = WidthM * 0.4f * cosA, rrZ = -LengthM * 0.35f * sinA;

            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0); GL.Vertex3(flX, 0, flZ);
            GL.Vertex3(0, 0, 0); GL.Vertex3(frX, 0, frZ);
            GL.Vertex3(0, 0, 0); GL.Vertex3(rlX, 0, rlZ);
            GL.Vertex3(0, 0, 0); GL.Vertex3(rrX, 0, rrZ);
            GL.End();

            // Motor discs
            float motorR = Math.Min(LengthM, WidthM) * 0.15f;
            DrawMotorDisc(flX, 0, flZ, motorR);
            DrawMotorDisc(frX, 0, frZ, motorR);
            DrawMotorDisc(rlX, 0, rlZ, motorR);
            DrawMotorDisc(rrX, 0, rrZ, motorR);

            DrawForwardIndicator();
        }

        // ==================== Private: Sub-components ====================

        private void DrawServoCamera(float servoDeg)
        {
            GL.PushMatrix();
            GL.Translate(0, -CameraDownOffsetM, CameraForwardOffsetM);
            // Servo tilts camera: rotation around X (lateral axis in body frame)
            // servo=90 is level, >90 tilts up
            GL.Rotate(90f - servoDeg, 1f, 0f, 0f);

            // Camera box (yellow)
            GL.Color3(0.9f, 0.85f, 0.2f);
            DrawWireBox(0.04f * 0.3f, 0, 0, 0.04f, 0.025f, 0.03f);

            // Camera look direction (green)
            GL.Color3(0.2f, 1f, 0.2f);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0);
            GL.Vertex3(0, 0, 0.15f);
            GL.End();

            GL.PopMatrix();
        }

        private void DrawForwardIndicator()
        {
            GL.Color3(1f, 0.3f, 0.3f);
            GL.LineWidth(2f);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0);
            GL.Vertex3(0, 0, LengthM * 0.6f);
            GL.End();
            GL.LineWidth(1f);
        }

        // ==================== Private: Primitives ====================

        private static void DrawWireBox(float cx, float cy, float cz, float sx, float sy, float sz)
        {
            float hx = sx / 2, hy = sy / 2, hz = sz / 2;
            GL.Begin(PrimitiveType.Lines);
            for (int i = 0; i < 2; i++)
            {
                float y = (i == 0) ? cy - hy : cy + hy;
                GL.Vertex3(cx - hx, y, cz - hz); GL.Vertex3(cx + hx, y, cz - hz);
                GL.Vertex3(cx + hx, y, cz - hz); GL.Vertex3(cx + hx, y, cz + hz);
                GL.Vertex3(cx + hx, y, cz + hz); GL.Vertex3(cx - hx, y, cz + hz);
                GL.Vertex3(cx - hx, y, cz + hz); GL.Vertex3(cx - hx, y, cz - hz);
            }
            GL.Vertex3(cx - hx, cy - hy, cz - hz); GL.Vertex3(cx - hx, cy + hy, cz - hz);
            GL.Vertex3(cx + hx, cy - hy, cz - hz); GL.Vertex3(cx + hx, cy + hy, cz - hz);
            GL.Vertex3(cx + hx, cy - hy, cz + hz); GL.Vertex3(cx + hx, cy + hy, cz + hz);
            GL.Vertex3(cx - hx, cy - hy, cz + hz); GL.Vertex3(cx - hx, cy + hy, cz + hz);
            GL.End();
        }

        private static void DrawMotorDisc(float cx, float cy, float cz, float radius)
        {
            GL.Begin(PrimitiveType.LineLoop);
            for (int i = 0; i < 12; i++)
            {
                float angle = (float)(i * 2 * Math.PI / 12);
                GL.Vertex3(cx + radius * (float)Math.Cos(angle), cy,
                           cz + radius * (float)Math.Sin(angle));
            }
            GL.End();
        }
    }
}
