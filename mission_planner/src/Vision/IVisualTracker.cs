// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.Threading;

namespace NOMAD.MissionPlanner
{
    // One frame worker owns the tracker. Each call borrows its bitmap only until the call returns.
    internal interface IVisualTracker : IDisposable
    {
        void Initialize(Bitmap frame, RectangleF bounds, CancellationToken cancellationToken);
        bool TryUpdate(Bitmap frame, CancellationToken cancellationToken, out RectangleF bounds);
    }
}
