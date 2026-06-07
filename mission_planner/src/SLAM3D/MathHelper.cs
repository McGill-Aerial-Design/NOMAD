// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
// ============================================================
// MathHelper.cs - Math utilities for .NET Framework 4.8
// ============================================================

using System;

namespace NOMAD.MissionPlanner.SLAM3D
{
    /// <summary>
    /// Math helper methods for .NET Framework 4.8 compatibility.
    /// </summary>
    internal static class MathHelper
    {
        /// <summary>Clamps a value between min and max.</summary>
        public static float Clamp(float value, float min, float max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        /// <summary>Clamps an int value between min and max.</summary>
        public static int Clamp(int value, int min, int max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        /// <summary>Sine (float version).</summary>
        public static float Sin(float radians) => (float)Math.Sin(radians);

        /// <summary>Cosine (float version).</summary>
        public static float Cos(float radians) => (float)Math.Cos(radians);

        /// <summary>Tangent (float version).</summary>
        public static float Tan(float radians) => (float)Math.Tan(radians);

        /// <summary>Arc tangent 2 (float version).</summary>
        public static float Atan2(float y, float x) => (float)Math.Atan2(y, x);

        /// <summary>Arc sine (float version).</summary>
        public static float Asin(float value) => (float)Math.Asin(value);

        /// <summary>Arc cosine (float version).</summary>
        public static float Acos(float value) => (float)Math.Acos(value);

        /// <summary>Square root (float version).</summary>
        public static float Sqrt(float value) => (float)Math.Sqrt(value);

        /// <summary>Absolute value (float version).</summary>
        public static float Abs(float value) => Math.Abs(value);

        /// <summary>Copy sign (float version).</summary>
        public static float CopySign(float value, float sign) => Math.Sign(sign) * Math.Abs(value);

        /// <summary>Maximum of two floats.</summary>
        public static float Max(float a, float b) => a > b ? a : b;

        /// <summary>Minimum of two floats.</summary>
        public static float Min(float a, float b) => a < b ? a : b;

        /// <summary>PI constant as float.</summary>
        public const float PI = 3.14159265358979323846f;
    }
}
