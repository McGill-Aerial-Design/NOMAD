// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors

using System;

namespace NOMAD.MissionPlanner
{
    public partial class MAVLinkConnectionManager
    {
        private void StartManagementClient()
        {
            if (_standalone != null)
            {
                try { _standalone.Dispose(); } catch { }
                _standalone = null;
            }

            _standalone = new StandaloneRouterClient(_config);
            _standalone.LinkStatusChanged += OnStandaloneLinkStatusChanged;
            _standalone.FailoverOccurred += OnStandaloneFailover;
            _standalone.ActiveLinkChanged += OnStandaloneActiveLinkChanged;
            _standalone.LogMessage += OnStandaloneLogMessage;
            _standalone.Start();
        }

        private void OnStandaloneLinkStatusChanged(object sender, LinkStatusChangedEventArgs args)
        {
            LinkStatusChanged?.Invoke(this, args);
        }

        private void OnStandaloneFailover(object sender, FailoverEventArgs args)
        {
            FailoverOccurred?.Invoke(this, args);
        }

        private void OnStandaloneActiveLinkChanged(object sender, string link)
        {
            ActiveLinkChanged?.Invoke(this, link);
        }

        private void OnStandaloneLogMessage(object sender, string message)
        {
            LogMessage?.Invoke(this, message);
        }
    }
}
