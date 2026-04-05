// ============================================================
// CameraController.cs - 3D camera control for SLAM view
// ============================================================

using System;
using NOMAD.MissionPlanner.SLAM3D.Models;

namespace NOMAD.MissionPlanner.SLAM3D.Camera
{
    /// <summary>
    /// Controls 3D camera position and orientation for the SLAM view.
    /// </summary>
    public class CameraController
    {
        // ==================== Configuration ====================
        
        /// <summary>Current camera view mode.</summary>
        public CameraViewMode ViewMode { get; set; } = CameraViewMode.ThirdPerson;
        
        /// <summary>Orbit distance from target (ThirdPerson/FreeOrbit).</summary>
        public float OrbitDistance { get; set; } = 12f;
        
        /// <summary>Minimum orbit distance.</summary>
        public float MinOrbitDistance { get; set; } = 2f;
        
        /// <summary>Maximum orbit distance.</summary>
        public float MaxOrbitDistance { get; set; } = 100f;
        
        /// <summary>Orbit yaw angle (degrees, 0=behind drone).</summary>
        public float OrbitYaw { get; set; } = 45f;
        
        /// <summary>Orbit pitch angle (degrees, positive=looking down).</summary>
        public float OrbitPitch { get; set; } = 30f;
        
        /// <summary>Minimum orbit pitch (degrees).</summary>
        public float MinOrbitPitch { get; set; } = -89f;
        
        /// <summary>Maximum orbit pitch (degrees).</summary>
        public float MaxOrbitPitch { get; set; } = 89f;
        
        /// <summary>Vertical field of view (degrees).</summary>
        public float FieldOfView { get; set; } = 60f;
        
        /// <summary>Minimum FOV (degrees).</summary>
        public float MinFov { get; set; } = 20f;
        
        /// <summary>Maximum FOV (degrees).</summary>
        public float MaxFov { get; set; } = 120f;
        
        /// <summary>Near clip plane distance.</summary>
        public float NearClip { get; set; } = 0.1f;
        
        /// <summary>Far clip plane distance.</summary>
        public float FarClip { get; set; } = 500f;
        
        /// <summary>First-person camera height offset.</summary>
        public float FirstPersonHeightOffset { get; set; } = 0.1f;
        
        /// <summary>Third-person follow distance behind drone.</summary>
        public float ThirdPersonBehindDistance { get; set; } = 3f;
        
        /// <summary>Third-person height above drone.</summary>
        public float ThirdPersonHeightOffset { get; set; } = 1.5f;
        
        // ==================== Orbit Center (FreeOrbit mode) ====================
        
        /// <summary>Orbit center X (FreeOrbit mode only).</summary>
        public float OrbitCenterX { get; set; }
        
        /// <summary>Orbit center Y (FreeOrbit mode only).</summary>
        public float OrbitCenterY { get; set; }
        
        /// <summary>Orbit center Z (FreeOrbit mode only).</summary>
        public float OrbitCenterZ { get; set; }
        
        // ==================== Output (computed camera state) ====================
        
        /// <summary>Camera position X.</summary>
        public float CamPosX { get; private set; }
        
        /// <summary>Camera position Y.</summary>
        public float CamPosY { get; private set; }
        
        /// <summary>Camera position Z.</summary>
        public float CamPosZ { get; private set; }
        
        /// <summary>Camera look-at target X.</summary>
        public float LookAtX { get; private set; }
        
        /// <summary>Camera look-at target Y.</summary>
        public float LookAtY { get; private set; }
        
        /// <summary>Camera look-at target Z.</summary>
        public float LookAtZ { get; private set; }
        
        /// <summary>Camera up vector X.</summary>
        public float UpX { get; private set; }
        
        /// <summary>Camera up vector Y.</summary>
        public float UpY { get; private set; } = 1f;
        
        /// <summary>Camera up vector Z.</summary>
        public float UpZ { get; private set; }
        
        // ==================== Public Methods ====================
        
        /// <summary>
        /// Update camera position based on drone pose.
        /// </summary>
        /// <param name="droneX">Drone X position (OpenGL frame)</param>
        /// <param name="droneY">Drone Y position (OpenGL frame)</param>
        /// <param name="droneZ">Drone Z position (OpenGL frame)</param>
        /// <param name="droneYaw">Drone yaw angle (radians)</param>
        public void Update(float droneX, float droneY, float droneZ, float droneYaw)
        {
            switch (ViewMode)
            {
                case CameraViewMode.FirstPerson:
                    UpdateFirstPerson(droneX, droneY, droneZ, droneYaw);
                    break;
                    
                case CameraViewMode.ThirdPerson:
                    UpdateThirdPerson(droneX, droneY, droneZ, droneYaw);
                    break;
                    
                case CameraViewMode.FreeOrbit:
                    UpdateFreeOrbit();
                    break;
            }
        }
        
        /// <summary>
        /// Rotate the orbit camera by delta angles.
        /// </summary>
        public void RotateOrbit(float deltaYawDeg, float deltaPitchDeg)
        {
            OrbitYaw += deltaYawDeg;
            OrbitPitch = MathHelper.Clamp(OrbitPitch + deltaPitchDeg, MinOrbitPitch, MaxOrbitPitch);
        }
        
        /// <summary>
        /// Zoom the orbit camera by delta distance.
        /// </summary>
        public void Zoom(float delta)
        {
            OrbitDistance = MathHelper.Clamp(OrbitDistance + delta, MinOrbitDistance, MaxOrbitDistance);
        }
        
        /// <summary>
        /// Pan the orbit center (FreeOrbit mode only).
        /// </summary>
        public void Pan(float deltaX, float deltaY)
        {
            if (ViewMode != CameraViewMode.FreeOrbit) return;
            
            // Pan in camera-relative coordinates
            float yawRad = OrbitYaw * MathHelper.PI / 180f;
            float cosYaw = MathHelper.Cos(yawRad);
            float sinYaw = MathHelper.Sin(yawRad);
            
            // Right vector (perpendicular to look direction in XZ plane)
            float rightX = cosYaw;
            float rightZ = -sinYaw;
            
            // Up vector is always Y
            OrbitCenterX += rightX * deltaX;
            OrbitCenterZ += rightZ * deltaX;
            OrbitCenterY += deltaY;
        }
        
        /// <summary>
        /// Center the orbit on the current drone position.
        /// </summary>
        public void CenterOnDrone(float droneX, float droneY, float droneZ)
        {
            OrbitCenterX = droneX;
            OrbitCenterY = droneY;
            OrbitCenterZ = droneZ;
        }
        
        /// <summary>
        /// Reset camera to default third-person view.
        /// </summary>
        public void Reset()
        {
            ViewMode = CameraViewMode.ThirdPerson;
            OrbitYaw = 45f;
            OrbitPitch = 30f;
            OrbitDistance = 12f;
            FieldOfView = 60f;
        }
        
        /// <summary>
        /// Cycle to the next view mode.
        /// </summary>
        public void CycleViewMode()
        {
            ViewMode = ViewMode switch
            {
                CameraViewMode.FirstPerson => CameraViewMode.ThirdPerson,
                CameraViewMode.ThirdPerson => CameraViewMode.FreeOrbit,
                CameraViewMode.FreeOrbit => CameraViewMode.FirstPerson,
                _ => CameraViewMode.ThirdPerson
            };
        }
        
        /// <summary>
        /// Get projection parameters for OpenGL setup.
        /// </summary>
        public (float fovDeg, float near, float far) GetProjectionParams()
        {
            return (MathHelper.Clamp(FieldOfView, MinFov, MaxFov), NearClip, FarClip);
        }
        
        // ==================== Private Methods ====================
        
        private void UpdateFirstPerson(float droneX, float droneY, float droneZ, float droneYaw)
        {
            // Camera at drone position, looking forward
            CamPosX = droneX;
            CamPosY = droneY + FirstPersonHeightOffset;
            CamPosZ = droneZ;
            
            // Look in drone's forward direction
            float lookDist = 10f;
            LookAtX = droneX + lookDist * MathHelper.Sin(droneYaw);
            LookAtY = droneY + FirstPersonHeightOffset;
            LookAtZ = droneZ + lookDist * MathHelper.Cos(droneYaw);
            
            UpX = 0; UpY = 1; UpZ = 0;
        }
        
        private void UpdateThirdPerson(float droneX, float droneY, float droneZ, float droneYaw)
        {
            // Camera behind and above drone, looking at drone
            float yawRad = droneYaw + OrbitYaw * MathHelper.PI / 180f;
            float pitchRad = OrbitPitch * MathHelper.PI / 180f;
            
            // Spherical to Cartesian
            float horizDist = OrbitDistance * MathHelper.Cos(pitchRad);
            float vertDist = OrbitDistance * MathHelper.Sin(pitchRad);
            
            CamPosX = droneX - horizDist * MathHelper.Sin(yawRad);
            CamPosY = droneY + vertDist;
            CamPosZ = droneZ - horizDist * MathHelper.Cos(yawRad);
            
            LookAtX = droneX;
            LookAtY = droneY;
            LookAtZ = droneZ;
            
            UpX = 0; UpY = 1; UpZ = 0;
        }
        
        private void UpdateFreeOrbit()
        {
            // Camera orbits around fixed center point
            float yawRad = OrbitYaw * MathHelper.PI / 180f;
            float pitchRad = OrbitPitch * MathHelper.PI / 180f;
            
            float horizDist = OrbitDistance * MathHelper.Cos(pitchRad);
            float vertDist = OrbitDistance * MathHelper.Sin(pitchRad);
            
            CamPosX = OrbitCenterX - horizDist * MathHelper.Sin(yawRad);
            CamPosY = OrbitCenterY + vertDist;
            CamPosZ = OrbitCenterZ - horizDist * MathHelper.Cos(yawRad);
            
            LookAtX = OrbitCenterX;
            LookAtY = OrbitCenterY;
            LookAtZ = OrbitCenterZ;
            
            UpX = 0; UpY = 1; UpZ = 0;
        }
    }
}
