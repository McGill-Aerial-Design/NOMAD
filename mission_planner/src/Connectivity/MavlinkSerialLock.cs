// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner
{
    /// <summary>
    /// Process-wide lock serializing direct MAVLink writes over Mission
    /// Planner's comPort (gimbal stick stream, motor-music script install,
    /// EKF source switch) so concurrent writers cannot interleave frames.
    /// Discrete output commands do not use this: they go through the C++ core
    /// boundary, which owns its own link.
    /// </summary>
    internal static class MavlinkSerialLock
    {
        private static readonly SemaphoreSlim s_lock = new SemaphoreSlim(1, 1);

        public static Task<bool> WaitAsync(int millisecondsTimeout)
            => s_lock.WaitAsync(millisecondsTimeout);

        public static Task<bool> WaitAsync(int millisecondsTimeout, System.Threading.CancellationToken token)
            => s_lock.WaitAsync(millisecondsTimeout, token);

        public static void Release() => s_lock.Release();
    }
}
