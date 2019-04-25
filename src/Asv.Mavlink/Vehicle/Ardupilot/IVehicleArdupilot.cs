using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    


   


    public class VehicleArdupilot : VehicleBase
    {
        private readonly IMavlinkV2Protocol _mavlink;

        public VehicleArdupilot(IMavlinkV2Protocol mavlink, VehicleBaseConfig config) : base(mavlink, config)
        {
            _mavlink = mavlink;
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
                await _mavlink.Mode.SetMode(1, 4, cancel);
            }
        }
    }
}
