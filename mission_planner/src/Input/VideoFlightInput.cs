// SPDX-License-Identifier: Apache-2.0
using System;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    [Flags]
    internal enum VideoFlightKeys
    {
        None = 0, Forward = 1, Back = 2, Left = 4, Right = 8,
        YawLeft = 16, YawRight = 32, Up = 64, Down = 128,
    }

    internal struct VideoFlightVelocity
    {
        internal float Forward, Right, Down, YawRate;
        internal bool Valid => Finite(Forward) && Finite(Right) && Finite(Down) && Finite(YawRate) &&
            Forward * Forward + Right * Right <= 4.001f && Math.Abs(Down) <= 1 && Math.Abs(YawRate) <= .524f;
        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }

    internal sealed class VideoFlightInput
    {
        private static object _owner;
        private bool _released;
        internal static bool Captured => Volatile.Read(ref _owner) != null;
        internal bool IsOwner => ReferenceEquals(Volatile.Read(ref _owner), this);
        internal bool Claim()
        {
            _released = false;
            return Interlocked.CompareExchange(ref _owner, this, null) == null;
        }
        internal void Release()
        {
            _released = false;
            Interlocked.CompareExchange(ref _owner, null, this);
        }
        internal bool TrySample(VideoFlightKeys keys, out VideoFlightVelocity velocity)
        {
            velocity = default;
            if (!IsOwner) { return false; }
            if (!_released)
            {
                _released = keys == VideoFlightKeys.None;
                return false;
            }
            int forward = Axis(keys, VideoFlightKeys.Forward, VideoFlightKeys.Back);
            int right = Axis(keys, VideoFlightKeys.Right, VideoFlightKeys.Left);
            float scale = forward != 0 && right != 0 ? (float)Math.Sqrt(2) : 2;
            velocity.Forward = forward * scale;
            velocity.Right = right * scale;
            velocity.Down = Axis(keys, VideoFlightKeys.Down, VideoFlightKeys.Up);
            velocity.YawRate = Axis(keys, VideoFlightKeys.YawRight, VideoFlightKeys.YawLeft) * (float)(Math.PI / 6);
            return true;
        }
        private static int Axis(VideoFlightKeys keys, VideoFlightKeys positive, VideoFlightKeys negative) =>
            ((keys & positive) != 0 ? 1 : 0) - ((keys & negative) != 0 ? 1 : 0);
    }
}
