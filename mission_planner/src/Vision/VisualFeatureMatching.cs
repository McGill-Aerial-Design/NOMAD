// SPDX-License-Identifier: Apache-2.0
using System.Collections.Generic;
using System.Linq;
using OpenCvSharp;

namespace NOMAD.MissionPlanner
{
    internal static class VisualFeatureMatching
    {
        internal static DMatch[] Mutual(Mat reference, Mat current)
        {
            if (reference.Rows < 2 || current.Rows < 2) { return new DMatch[0]; }
            using (var matcher = new BFMatcher(NormTypes.Hamming))
            using (var candidates = new Mat())
            {
                var forward = matcher.KnnMatch(reference, current, 2)
                    .Where(pair => Distinct(pair)).Select(pair => pair[0]).ToArray();
                if (forward.Length == 0) { return forward; }
                var rows = new Dictionary<int, int>();
                foreach (var match in forward)
                {
                    if (rows.ContainsKey(match.TrainIdx)) { continue; }
                    rows.Add(match.TrainIdx, rows.Count);
                    using (var row = current.Row(match.TrainIdx)) { candidates.PushBack(row); }
                }
                // Only forward candidates can survive the mutual test; retain the full reverse search reference.
                var reverse = matcher.KnnMatch(candidates, reference, 2);
                return forward.Where(match => Distinct(reverse[rows[match.TrainIdx]]) &&
                    reverse[rows[match.TrainIdx]][0].TrainIdx == match.QueryIdx).ToArray();
            }
        }

        private static bool Distinct(DMatch[] pair) => pair.Length == 2 &&
            pair[0].Distance < 64 && pair[0].Distance < pair[1].Distance * .72;
    }
}
