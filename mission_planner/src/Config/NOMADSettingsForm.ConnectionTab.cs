// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    public partial class NOMADSettingsForm
    {
        private TabPage CreateConnectionTab()
        {
            var tab = CreateTabPage("Connection");
            AddLabel(tab, "NOMAD uses the live aircraft selected in Mission Planner.", 20, 20);
            AddLabel(tab, "Use Mission Planner for connection, arming, flight modes, LAND and missions.", 20, 50);
            return tab;
        }

        private TabPage CreateMultiLinkTab()
        {
            var tab = CreateTabPage("Multi-Link");
            int y = 15;
            AddSectionLabel(tab, "Aircraft links", ref y);
            _chkRouterClientEnabled = AddCheckBox(tab, "Run the router inside Mission Planner", 20, y, Color.LimeGreen);
            y += 40;
            AddLabel(tab, "Mission Planner UDPCl port:", 20, y);
            _numRouterLocalPort = AddNumericUpDown(tab, 215, y, 80, 1, 65535, 14600);
            y += 35;
            AddLabel(tab, "Preferred link ID:", 20, y);
            _txtRouterPreferredLink = AddTextBox(tab, 215, y, 250);
            y += 35;
            AddLabel(tab, "Physical links (JSON array):", 20, y);
            y += 25;
            _txtRouterLinks = new TextBox { Location = new Point(20, y), Size = new Size(580, 170),
                Multiline = true, ScrollBars = ScrollBars.Both, WordWrap = false };
            tab.Controls.Add(_txtRouterLinks);
            y += 180;
            var hint = new Label
            {
                Text = "Save physical links in this aircraft profile; an empty list opens no connections.\n" +
                    "Each link needs Id, Name, Transport (UDP, TCP or COM) and its endpoint or Device.\n" +
                    "Use the ELRS link ID as preferred, with a lower Priority for the LTE backup.\n" +
                    "Connect Mission Planner using UDPCl to 127.0.0.1 and the port above.\n" +
                    "Changing profiles or link settings stops and replaces the router.",
                Location = new Point(20, y), AutoSize = true, MaximumSize = new Size(580, 0),
                ForeColor = Color.Gray
            };
            tab.Controls.Add(hint);
            return tab;
        }
    }
}
