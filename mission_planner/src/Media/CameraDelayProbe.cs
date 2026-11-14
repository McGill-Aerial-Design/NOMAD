// SPDX-License-Identifier: Apache-2.0
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace NOMAD.MissionPlanner
{
    // Pure scene-change measurements for the camera delay check: brightness samples, a noise-based change
    // threshold and the delay estimate. The player owns gimbal motion and frame timing.
    internal static class CameraDelayProbe
    {
        internal const int Columns = 32, Rows = 24;
        internal const float StepDegrees = 10;
        internal const int Trials = 4;

        // A coarse brightness grid; enough to see a whole-scene shift and cheap at any resolution.
        internal static float[] Sample(Bitmap image)
        {
            var samples = new float[Columns * Rows];
            for (int row = 0; row < Rows; row++)
            {
                for (int column = 0; column < Columns; column++)
                {
                    var color = image.GetPixel((int)((column + .5) * image.Width / Columns),
                        (int)((row + .5) * image.Height / Rows));
                    samples[row * Columns + column] = .299f * color.R + .587f * color.G + .114f * color.B;
                }
            }
            return samples;
        }

        internal static float Difference(float[] left, float[] right)
        {
            float total = 0;
            for (int index = 0; index < left.Length; index++) { total += Math.Abs(left[index] - right[index]); }
            return total / left.Length;
        }

        // A move must clearly exceed the sensor noise and compression shimmer of a still scene.
        internal static float ChangeThreshold(IReadOnlyList<float> stillDifferences) =>
            Math.Max(4, Math.Max(2 * stillDifferences.Max(), stillDifferences.Average() + 6));

        // Median of the trials, less the receiver jitter buffer that the profile already counts separately.
        internal static int Estimate(IReadOnlyList<double> trialSeconds, int receiverLatencyMs)
        {
            var ordered = trialSeconds.OrderBy(value => value).ToArray();
            double median = ordered.Length % 2 == 1 ? ordered[ordered.Length / 2]
                : (ordered[ordered.Length / 2 - 1] + ordered[ordered.Length / 2]) / 2;
            return (int)Math.Round(Math.Max(0, Math.Min(2000, median * 1000 - receiverLatencyMs)));
        }

        // Tilt toward the middle of the calibrated range so the step never hits a limit.
        internal static float StepTarget(float pitch, int minimum, int maximum) =>
            pitch + StepDegrees <= maximum ? pitch + StepDegrees : Math.Max(minimum, pitch - StepDegrees);
    }
}
