// SPDX-License-Identifier: Apache-2.0
using System;
using System.Diagnostics;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void NativeMediaTransport()
    {
        MediaWorkerHost.RequestStart();
        using (var session = new VideoSession(MediaWorkerPipeline.Create))
        {
            const string pipeline = "videotestsrc is-live=true pattern=ball ! " +
                "video/x-raw,width=1920,height=1080,framerate=30/1 ! videoconvert ! " +
                "video/x-raw,format=BGRA ! appsink name=outsink sync=false";
            session.Start(pipeline);
            var startup = Stopwatch.StartNew(); long nextDiagnostic = 6000;
            WaitMedia(() =>
            {
                if (startup.ElapsedMilliseconds >= nextDiagnostic)
                { Console.WriteLine("  Native startup: " + session.State + ", " + MediaWorkerHost.DiagnosticStatus);
                  nextDiagnostic += 5000; }
                return session.State == VideoState.Streaming || session.State == VideoState.Stopped;
            }, "native synthetic media startup", 120000);
            Check(session.State == VideoState.Streaming, "Native synthetic media failed: " + session.Error);
            using (var first = NextVideoFrame(session))
            { Check(first.Image.Size == new Size(1920, 1080), "Native transport changed the frame dimensions"); }
            var clock = Stopwatch.StartNew();
            int frames = 0, ticks = 0; long previous = 0, last = 0, maximumGap = 0;
            using (var timer = new System.Windows.Forms.Timer { Interval = 15 })
            {
                timer.Tick += (sender, args) => ticks++;
                timer.Start();
                while (clock.ElapsedMilliseconds < 2200)
                {
                    using (var frame = session.TakeFrame())
                    {
                        if (frame != null)
                        {
                            Check(frame.Sequence > previous && frame.Fresh, "Native transport reordered or freshened old frames");
                            previous = frame.Sequence; frames++;
                            maximumGap = Math.Max(maximumGap, clock.ElapsedMilliseconds - last);
                            last = clock.ElapsedMilliseconds;
                        }
                    }
                    Application.DoEvents(); Thread.Sleep(2);
                }
                timer.Stop();
            }
            Check(frames >= 44 && maximumGap <= 250 && ticks >= 40,
                "Native 1080p transport missed responsiveness/cadence budget: frames=" + frames +
                ", gap=" + maximumGap + ", UI ticks=" + ticks);
            Console.WriteLine("  Native synthetic 1080p transport: " + (frames / clock.Elapsed.TotalSeconds).ToString("F1") +
                " FPS, maximum gap=" + maximumGap + " ms, UI ticks=" + ticks);
            if (!string.IsNullOrEmpty(Environment.GetEnvironmentVariable("NOMAD_TEST_TAM_MODEL")))
            {
                var loading = Task.Run(() => new MediaWorkerTracker(CancellationToken.None, VisualTrackingMode.Object, true));
                WaitMedia(() => loading.IsCompleted, "isolated GPU model load", 120000);
                using (var tracker = loading.GetAwaiter().GetResult())
                using (var frame = NextVideoFrame(session))
                {
                    Check(tracker.Provider?.Contains("NVIDIA") == true, "Worker GPU tracker did not select NVIDIA");
                    var request = Task.Run(() =>
                    {
                        tracker.Initialize(frame.Image, new RectangleF(700, 300, 240, 300), CancellationToken.None);
                        return tracker.Update(frame.Image, CancellationToken.None);
                    });
                    WaitMedia(() => request.IsCompleted, "isolated GPU prompt/update", 15000);
                    Check(Enum.IsDefined(typeof(VisualTrackingState), request.GetAwaiter().GetResult().State),
                        "GPU worker returned invalid tracking state");
                    Console.WriteLine("  Worker GPU native load/prompt/update: " + tracker.Provider +
                        "; this check does not establish scene accuracy");
                }
            }
            else { Console.WriteLine("SKIP: worker GPU model/prompt (NOMAD_TEST_TAM_MODEL not set)"); }
            session.Stop(); Wait(session.Completion, "native media cleanup");
        }
    }
}
