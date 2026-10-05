// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 The NOMAD Authors
using System;
using System.Globalization;
using System.Threading;
using System.Threading.Tasks;

namespace NOMAD.MissionPlanner.Connectivity
{
    public sealed partial class NomadCoreClient
    {
        public Task<NomadCoreRequestResult> GetActuatorsAsync(CancellationToken cancellationToken = default) =>
            RunCoreAsync("get-actuators", cancellationToken);

        public Task<NomadCoreRequestResult> ConfigureActuatorsAsync(string configurationJson,
            CancellationToken cancellationToken = default) =>
            RunCoreAsync("configure-actuators", cancellationToken, configurationJson);

        public Task<NomadCoreRequestResult> ActuatorActionAsync(string actuatorId, string operation,
            string inputSource = "ui", int? inputSlot = null, double? value = null,
            CancellationToken cancellationToken = default, Func<bool> inputStillCurrent = null)
        {
            if (string.IsNullOrWhiteSpace(actuatorId) || string.IsNullOrWhiteSpace(operation) ||
                (value.HasValue && (double.IsNaN(value.Value) || double.IsInfinity(value.Value))))
            {
                return Task.FromResult(RejectLocal("An actuator ID, operation and finite value are required."));
            }
            return _runtimeClient.RunAsync("actuator-action", new[] { actuatorId, operation, inputSource,
                inputSlot?.ToString(CultureInfo.InvariantCulture) ?? "", value?.ToString("R", CultureInfo.InvariantCulture) ?? "" },
                cancellationToken, inputStillCurrent);
        }
    }
}
