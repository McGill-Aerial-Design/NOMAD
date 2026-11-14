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
                    var slider = FindPayloadControl<System.Windows.Controls.Slider>(panel, "camera:slider");
                    var pwm = FindPayloadControl<System.Windows.Controls.TextBlock>(panel, "camera:pwm");
                    var safe = FindPayloadControl<System.Windows.Controls.Button>(panel, "camera:safe");
                    Check(FindPayloadControls(panel.WpfRoot, "camera:position").Count() == 0 && slider.Value == 1300 &&
                        pwm.Text.Contains("1300") && peer.ServoTargets.Length == 0,
                        "Continuous payload retained Activate, changed neutral or sent during construction");
                    double originalWidth = slider.ActualWidth;
                    form.ClientSize = new Size(1100, 200); peer.Pump(10);
                    Check(slider.ActualWidth > originalWidth && slider.ActualWidth >= 350,
                        "Payload slider did not use the available horizontal space");
                    var image = new System.Windows.Media.Imaging.RenderTargetBitmap(
                        (int)panel.WpfRoot.ActualWidth, (int)panel.WpfRoot.ActualHeight, 96, 96,
                        System.Windows.Media.PixelFormats.Pbgra32);
                    image.Render(panel.WpfRoot);
                    var encoder = new System.Windows.Media.Imaging.PngBitmapEncoder();
                    encoder.Frames.Add(System.Windows.Media.Imaging.BitmapFrame.Create(image));
                    using (var output = File.Create(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "payload-continuous.png")))
                    { encoder.Save(output); }
                    slider.Value = 1100;
                    slider.Value = 1800;
                    Check(pwm.Text.Contains("1800"), "Slider did not preview the requested PWM immediately");
                    peer.Until(() => peer.ServoTargets.Length == 1 && PayloadController.Current.TargetPwm("camera") == 1800,
                        "direct slider command and acknowledgement");
                    Check(peer.ServoTargets[0] == (14, 1800), "Slider sent intermediate or uncalibrated PWM");
                    safe.RaiseEvent(new System.Windows.RoutedEventArgs(System.Windows.Controls.Primitives.ButtonBase.ClickEvent));
                    peer.Until(() => PayloadController.Current.TargetPwm("camera") == 1000 &&
                        peer.ServoTargets.Length == 2, "explicit Safe after slider movement");
                    Check(pwm.Text.Contains("1000") && slider.Value == 1000 && peer.ServoTargets[1] == (14, 1000),
                        "Safe did not update the payload target and PWM readout");
                    slider.Value = 1600;
                    panel.Dispose(); peer.Pump(100);
                    Check(peer.ServoTargets.Length == 2, "Disposed payload view sent its pending slider value");
                }
            }
            finally { PayloadController.Shutdown(); }
        }
    }
    private static T FindPayloadControl<T>(ActuatorControlPanel panel, string id) where T : System.Windows.DependencyObject =>
        FindPayloadControls(panel.WpfRoot, id).OfType<T>().Single();

    private static System.Collections.Generic.IEnumerable<System.Windows.DependencyObject> FindPayloadControls(
        System.Windows.DependencyObject parent, string id)
    {
        if (System.Windows.Automation.AutomationProperties.GetAutomationId(parent) == id) { yield return parent; }
        foreach (object child in System.Windows.LogicalTreeHelper.GetChildren(parent))
        {
            if (child is System.Windows.DependencyObject element)
            {
                foreach (var result in FindPayloadControls(element, id)) { yield return result; }
            }
        }
    }

}
