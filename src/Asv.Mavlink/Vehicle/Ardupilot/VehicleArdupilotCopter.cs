using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Ardupilotmega;
using Asv.Mavlink.V2.Common;
using Asv.Mavlink.Client;

namespace Asv.Mavlink
{
    public class VehicleArdupilotCopter : VehicleArdupilot
    {
        public VehicleArdupilotCopter(IMavlinkClient mavlink, VehicleBaseConfig config) : base(mavlink, config)
        {
        }

        protected override VehicleClass InterpretVehicleClass(HeartbeatPayload heartbeatPacket)
        {
            return VehicleClass.Copter;
        }

        #region Modes

        public override IEnumerable<VehicleCustomMode> AvailableModes => GetModes();
        private VehicleCustomMode[] _modes;
        private IEnumerable<VehicleCustomMode> GetModes()
        {
            return _modes ?? (_modes = Enum.GetValues(typeof(CopterMode)).Cast<CopterMode>()
                       .Select(_ => new VehicleCustomMode {Name = _.ToString("G"), Value = (uint) _}).ToArray());
        }

        protected override VehicleMode Interpret(HeartbeatPayload heartbeat)
        {
            var customMode = (CopterMode) (heartbeat.CustomMode);
            return new VehicleMode
            {
                BaseMode = heartbeat.BaseMode,
                CustomMode = new VehicleCustomMode
                {
                    Name = customMode.ToString("G"),
                    Value = (uint) customMode
                }
            };
        }

        #endregion

        #region GoTo

        public Task GoToRel(GeoPoint location, CancellationToken cancel, double? yawDeg = null)
        {
            return GoTo(location, MavFrame.MavFrameGlobalRelativeAltInt, cancel, yawDeg);
        }

        private async Task GoTo(GeoPoint location, MavFrame frame, CancellationToken cancel, double? yawDeg = null, double? vx = null, double? vy = null, double? vz = null)
        {
            await EnsureInGuidedMode(cancel);
            if (location.Altitude.HasValue == false)
            {
                throw new MavlinkException(RS.VehicleArdupilot_GoTo_Altitude_of_end_point_is_null);
            }

            var yaw = yawDeg.HasValue ? (float?)GeoMath.DegreesToRadians(yawDeg.Value) : null;
            await Mavlink.Common.SetPositionTargetGlobalInt(0, frame, cancel, (int)(location.Latitude * 10000000), (int)(location.Longitude * 10000000), (float)location.Altitude.Value, yaw: yaw);

        }

        protected override Task InternalGoToGlob(GeoPoint location, CancellationToken cancel, double? yawDeg)
        {
            return GoTo(location, MavFrame.MavFrameGlobalInt, cancel, yawDeg);
        }

        #endregion

        protected override Task<bool> CheckGuidedMode(CancellationToken cancel)
        {
            return Task.FromResult(
                Mavlink.Heartbeat.RawHeartbeat.Value.BaseMode.HasFlag(MavModeFlag.MavModeFlagCustomModeEnabled) &&
                Mavlink.Heartbeat.RawHeartbeat.Value.CustomMode == (int)CopterMode.CopterModeGuided);
        }

        protected override async Task EnsureInGuidedMode(CancellationToken cancel)
        {
            if (!await CheckGuidedMode(cancel))
            {
                await Mavlink.Common.SetMode(1, (int)CopterMode.CopterModeGuided, cancel);
            }
        }

        public override Task DoRtl(CancellationToken cancel)
        {
            return Mavlink.Common.SetMode(1, (int)CopterMode.CopterModeRtl, cancel);
        }


    }
}