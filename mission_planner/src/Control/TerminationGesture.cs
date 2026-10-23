// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
namespace NOMAD.MissionPlanner
{
    internal sealed class TerminationGesture
    {
        private bool _released, _left, _right, _sawLeft, _sawRight;
        private long _firstPress = -1;

        internal void Reset()
        {
            _released = _left = _right = _sawLeft = _sawRight = false;
            _firstPress = -1;
        }

        internal bool Update(bool eligible, bool left, bool right, long milliseconds)
        {
            if (!eligible) { Reset(); return false; }
            if (!_released)
            {
                if (!left && !right) { _released = true; }
                _left = left;
                _right = right;
                return false;
            }
            if (_firstPress >= 0 && milliseconds - _firstPress > 750)
            { Reset(); return false; }
            bool newLeft = left && !_left;
            bool newRight = right && !_right;
            if (_firstPress < 0 && (newLeft || newRight)) { _firstPress = milliseconds; }
            _sawLeft |= newLeft;
            _sawRight |= newRight;
            _left = left;
            _right = right;
            if (!left || !right || !_sawLeft || !_sawRight) { return false; }
            _released = false;
            _sawLeft = _sawRight = false;
            _firstPress = -1;
            return true;
        }
    }
}
