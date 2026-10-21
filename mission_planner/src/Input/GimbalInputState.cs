// SPDX-License-Identifier: Apache-2.0
using System;
namespace NOMAD.MissionPlanner
{
    // Reconnecting or losing input requires a real centered sample before further stick motion.
    internal sealed class GimbalInputState
    {
        private long _generation = -1;
        private bool _centered;
        internal void Reset()
        {
            _centered = false;
        }
        internal bool CanMove(bool valid, float x, float y, long generation)
        {
            if (_generation != generation)
            {
                _generation = generation;
                _centered = false;
            }
            if (!valid || float.IsNaN(x) || float.IsNaN(y) || float.IsInfinity(x) ||
                float.IsInfinity(y) || Math.Abs(x) > 1 || Math.Abs(y) > 1)
            {
                _centered = false;
                return false;
            }
            if (x == 0 && y == 0)
            {
                _centered = true;
                return false;
            }
            return _centered;
        }
    }
}
