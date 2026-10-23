// SPDX-License-Identifier: Apache-2.0
using Newtonsoft.Json;
namespace NOMAD.MissionPlanner
{
    public partial class NOMADConfig
    {
        internal string GetInputMappingError()
        {
            try { InputBinding.Parse(Inputs); return null; }
            catch (JsonException error) { return error.Message; }
        }
    }
}
