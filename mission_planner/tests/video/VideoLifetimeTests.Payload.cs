// SPDX-License-Identifier: Apache-2.0
using System;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;
using NOMAD.MissionPlanner;

internal static partial class VideoLifetimeTests
{
    private static void ContinuousPayloadSlider()
    {
        using (var peer = new TrackingPeer())
        using (var form = new Form { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual,
            Location = new Point(-2000, -2000), ClientSize = new Size(900, 200) })
        {
            peer.Config.ActuatorDefinitions = new JArray(new JObject
            {
                ["id"] = "camera", ["name"] = "Camera pitch", ["behavior"] = "ServoPosition", ["channel"] = 14,
                ["pwm_min"] = 1000, ["pwm_max"] = 1800, ["pwm_neutral"] = 1300, ["safe_pwm"] = 1000,
                ["hazardous"] = false, ["confirmation_count"] = 0,
            });
            PayloadController.ApplyProfile(peer.Config);
            try
            {
                using (var panel = new ActuatorControlPanel(peer.Config) { Dock = DockStyle.Fill })
                {
                    form.Controls.Add(panel); form.Show(); peer.Pump(80);
                    var slider = panel.Controls.Find("camera:slider", true).OfType<TrackBar>().Single();
                    var pwm = panel.Controls.Find("camera:pwm", true).OfType<Label>().Single();
                    var safe = panel.Controls.Find("camera:safe", true).OfType<Button>().Single();
                    Check(panel.Controls.Find("camera:position", true).Length == 0 && slider.Value == 1300 &&
                        pwm.Text.Contains("1300") && peer.ServoTargets.Length == 0,
                        "Continuous payload retained Activate, changed neutral or sent during construction");
                    int originalWidth = slider.Width;
                    form.ClientSize = new Size(1100, 200); peer.Pump(10);
                    Check(slider.Width > originalWidth && slider.Width >= 350,
                        "Payload slider did not use the available horizontal space");
                    panel.BackColor = NOMADTheme.CARD_BG;
                    using (var image = new Bitmap(panel.Width, panel.Height))
                    { panel.DrawToBitmap(image, new Rectangle(Point.Empty, image.Size));
                      image.Save(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "payload-continuous.png")); }
                    var scroll = typeof(TrackBar).GetMethod("OnScroll", BindingFlags.Instance | BindingFlags.NonPublic);
                    slider.Value = 1100; scroll.Invoke(slider, new object[] { EventArgs.Empty });
                    slider.Value = 1800; scroll.Invoke(slider, new object[] { EventArgs.Empty });
                    Check(pwm.Text.Contains("1800"), "Slider did not preview the requested PWM immediately");
                    peer.Until(() => peer.ServoTargets.Length == 1 && PayloadController.Current.TargetPwm("camera") == 1800,
                        "direct slider command and acknowledgement");
                    Check(peer.ServoTargets[0] == (14, 1800), "Slider sent intermediate or uncalibrated PWM");
                    safe.PerformClick();
                    peer.Until(() => PayloadController.Current.TargetPwm("camera") == 1000 &&
                        peer.ServoTargets.Length == 2, "explicit Safe after slider movement");
                    Check(pwm.Text.Contains("1000") && slider.Value == 1000 && peer.ServoTargets[1] == (14, 1000),
                        "Safe did not update the payload target and PWM readout");
                    slider.Value = 1600; scroll.Invoke(slider, new object[] { EventArgs.Empty });
                    panel.Dispose(); peer.Pump(100);
                    Check(peer.ServoTargets.Length == 2, "Disposed payload view sent its pending slider value");
                }
            }
            finally { PayloadController.Shutdown(); }
        }
    }
}
