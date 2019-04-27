using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;
using NLog;

namespace Asv.Mavlink
{
    


   


    public class VehicleArdupilot : VehicleBase
    {
        private readonly IMavlinkV2Protocol _mavlink;
        private readonly VehicleBaseConfig _config;

        public VehicleArdupilot(IMavlinkV2Protocol mavlink, VehicleBaseConfig config) : base(mavlink, config)
        {
            _mavlink = mavlink;
            _config = config;
        }

        protected override  Task<bool> CheckGuidedMode(CancellationToken cancel)
        {
            return Task.FromResult(
                _mavlink.Rtt.RawHeartbeat.Value.BaseMode.HasFlag(MavModeFlag.MavModeFlagCustomModeEnabled) &&
                _mavlink.Rtt.RawHeartbeat.Value.CustomMode == 4);
        }

        protected override async Task EnsureInGuidedMode(CancellationToken cancel)
        {
            if (!await CheckGuidedMode(cancel))
            {
                await _mavlink.Common.SetMode(1, 4, cancel);
            }
        }

        public override async Task GoTo(GeoPoint location,CancellationToken cancel)
        {
            await EnsureInGuidedMode(cancel);
            if (location.Altitude.HasValue == false)
            {
                throw new MavlinkException(RS.VehicleArdupilot_GoTo_Altitude_of_end_point_is_null);
            }
            await _mavlink.Common.SetPositionTargetGlobalInt(0, MavFrame.MavFrameGlobalRelativeAltInt, (int) (location.Latitude*10000000), (int) (location.Longitude*10000000), (float) location.Altitude.Value,0,0,0,0,0,0,0,0, (PositionTargetTypemask) 6528,cancel);
        }

    }
}
