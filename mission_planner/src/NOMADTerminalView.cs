// ============================================================
// NOMAD Terminal View
// ============================================================

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Utilities;
using Newtonsoft.Json;

namespace NOMAD.MissionPlanner
{
    public class NOMADTerminalView : NOMADViewBase
    {
        private readonly NOMADConfig _config;
        private JetsonTerminalControl _terminal;
        
        public NOMADTerminalView(NOMADConfig config)
        {
            _config = config;
            InitializeUI();
        }
        
        private void InitializeUI()
        {
            try
            {
                _terminal = new JetsonTerminalControl(_config);
                _terminal.Dock = DockStyle.Fill;
                this.Controls.Add(_terminal);
            }
            catch (Exception ex)
            {
                var errorLabel = new Label
                {
                    Text = $"Terminal unavailable: {ex.Message}",
                    Font = new Font("Segoe UI", 12),
                    ForeColor = ERROR_COLOR,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                };
                this.Controls.Add(errorLabel);
            }
        }
        
        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                _terminal?.Dispose();
            }
            base.Dispose(disposing);
        }
    }
}
