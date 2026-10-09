// SPDX-License-Identifier: Apache-2.0
using System.Drawing;
using GMap.NET.WindowsForms;
using MissionPlanner.GCSViews;

namespace NOMAD.MissionPlanner
{
    public static partial class MapOverlayManager
    {
        private static readonly Color SOFT_BOUNDARY_STROKE = Color.Yellow;
        private const int SOFT_BOUNDARY_WIDTH = 2;
        private static readonly Color HARD_BOUNDARY_STROKE = Color.Red;
        private const int HARD_BOUNDARY_WIDTH = 3;

        private static GMapControl GetMapControl() => FlightData.mymap;
        private static GMapControl GetPlanMapControl() => FlightPlanner.instance?.MainMap;

        public static void DrawBoundaries(GeofenceConfig config)
        {
            if (config == null)
            {
                return;
            }
            ConfigureBoundaryZoneRendering(config);
        }
    }
}
