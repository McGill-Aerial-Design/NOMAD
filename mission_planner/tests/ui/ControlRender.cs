// SPDX-License-Identifier: Apache-2.0
using System.Drawing;
using System.IO;
using System.Windows.Forms;
using System.Windows.Forms.Integration;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace NOMAD.Testing
{
    public static class ControlRender
    {
        public static Bitmap Capture(Control root)
        {
            var image = new Bitmap(root.Width, root.Height);
            try
            {
                root.DrawToBitmap(image, new Rectangle(Point.Empty, root.Size));
                using (var graphics = Graphics.FromImage(image)) { DrawWpf(root, root, graphics); }
                return image;
            }
            catch { image.Dispose(); throw; }
        }

        private static void DrawWpf(Control root, Control control, Graphics graphics)
        {
            var host = control as ElementHost;
            if (host != null && host.Visible && host.Child != null && host.Width > 0 && host.Height > 0)
            {
                var visual = host.Child;
                visual.UpdateLayout();
                var dpi = VisualTreeHelper.GetDpi(visual);
                var bitmap = new RenderTargetBitmap(host.Width, host.Height, 96 * dpi.DpiScaleX,
                    96 * dpi.DpiScaleY, PixelFormats.Pbgra32);
                bitmap.Render(visual);
                var encoder = new PngBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(bitmap));
                using (var stream = new MemoryStream())
                {
                    encoder.Save(stream); stream.Position = 0;
                    using (var overlay = new Bitmap(stream))
                    {
                        var position = root.PointToClient(host.PointToScreen(Point.Empty));
                        graphics.DrawImageUnscaled(overlay, position);
                    }
                }
            }
            foreach (Control child in control.Controls) { DrawWpf(root, child, graphics); }
        }
    }
}
