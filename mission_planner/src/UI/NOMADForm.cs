// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System.Windows.Forms;

namespace NOMAD.MissionPlanner
{
    [global::MissionPlanner.Utilities.PreventTheming]
    internal class NOMADForm : Form
    {
        public NOMADForm()
        {
            BackColor = NOMADTheme.BG_DARK;
            ForeColor = NOMADTheme.TEXT_PRIMARY;
            Font = NOMADTheme.Font();
        }
    }
}
