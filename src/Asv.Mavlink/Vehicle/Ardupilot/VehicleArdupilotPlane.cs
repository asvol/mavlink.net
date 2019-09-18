using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Ardupilotmega;
using Asv.Mavlink.V2.Common;
using NLog;
using MavCmd = Asv.Mavlink.V2.Common.MavCmd;

namespace Asv.Mavlink
{
    public class VehicleArdupilotPlane : VehicleArdupilot
    {
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();

        public VehicleArdupilotPlane(IMavlinkClient mavlink, VehicleBaseConfig config) : base(mavlink, config)
        {
        }

        protected override VehicleClass InterpretVehicleClass(HeartbeatPayload heartbeatPacket)
        {
            return VehicleClass.Plane;
        }

        #region Modes

        public override IEnumerable<VehicleCustomMode> AvailableModes => GetModes();
        private VehicleCustomMode[] _modes;
        private IEnumerable<VehicleCustomMode> GetModes()
        {
            return _modes ?? (_modes = Enum.GetValues(typeof(PlaneMode)).Cast<PlaneMode>()
                       .Select(_ => new VehicleCustomMode { Name = _.ToString("G"), Value = (uint)_ }).ToArray());
        }

        protected override VehicleMode Interpret(HeartbeatPayload heartbeat)
        {
            var customMode = (PlaneMode)(heartbeat.CustomMode);
            return new VehicleMode
            {
                BaseMode = heartbeat.BaseMode,
                CustomMode = new VehicleCustomMode
                {
                    Name = customMode.ToString("G"),
                    Value = (uint)customMode
                }
            };
        }

        #endregion

        protected override Task<bool> CheckGuidedMode(CancellationToken cancel)
        {
            return Task.FromResult(
                Mavlink.Heartbeat.RawHeartbeat.Value.BaseMode.HasFlag(MavModeFlag.MavModeFlagCustomModeEnabled) &&
                Mavlink.Heartbeat.RawHeartbeat.Value.CustomMode == (int)PlaneMode.PlaneModeGuided);
        }

        protected override async Task EnsureInGuidedMode(CancellationToken cancel)
        {
            if (!await CheckGuidedMode(cancel))
            {
                await Mavlink.Common.SetMode(1, (int)PlaneMode.PlaneModeGuided, cancel);
            }
        }

        protected override async Task InternalGoToGlob(GeoPoint location, CancellationToken cancel, double? yawDeg)
        {
            if (location.Altitude == null)
                throw new MavlinkException(RS.VehicleArdupilot_GoTo_Altitude_of_end_point_is_null);

            if (!Home.Value.HasValue) // we have no Home position and can't calculate relative altitude
            {
                _logger.Debug("We have no Home position and can't calculate relative altitude");
                while (!cancel.IsCancellationRequested)
                {
                    await RequestHome(cancel);
                    await Task.Delay(1000, cancel);
                    if (Home.Value.HasValue) break;
                }
            }

            var rel = location.Altitude.Value - this.Home.Value.Value.Altitude;

            if (rel == null)
                    throw new MavlinkException(RS.VehicleArdupilotPlane_InternalGoToGlob_Altitude_of_home_position_is_null);

            await Mavlink.Mission.MissionItem(MavFrame.MavFrameGlobalRelativeAlt, MavCmd.MavCmdNavWaypoint, true,
                    true, 0, 0, 0,
                    0, (float) location.Latitude, (float) location.Longitude, (float) rel.Value,
                    MavMissionType.MavMissionTypeMission, 3, cancel);
        }

        public override Task DoRtl(CancellationToken cancel)
        {
            return Mavlink.Common.SetMode(1, (int)PlaneMode.PlaneModeRtl, cancel);
        }
    }
}