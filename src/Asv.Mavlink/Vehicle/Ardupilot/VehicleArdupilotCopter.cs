using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Ardupilotmega;
using Asv.Mavlink.V2.Common;
using Asv.Mavlink.Client;
using Newtonsoft.Json;

namespace Asv.Mavlink
{

    public static class CopterModeHelper
    {
        public static string GetDisplayName(this CopterMode customMode)
        {
            switch (customMode)
            {
                case CopterMode.CopterModeStabilize:
                    return "Stabilize";
                case CopterMode.CopterModeAcro:
                    return "Acro";
                case CopterMode.CopterModeAltHold:
                    return "AltHold";
                case CopterMode.CopterModeAuto:
                    return "Auto";
                case CopterMode.CopterModeGuided:
                    return "Guided";
                case CopterMode.CopterModeLoiter:
                    return "Loiter";
                case CopterMode.CopterModeRtl:
                    return "RTL";
                case CopterMode.CopterModeCircle:
                    return "Circle";
                case CopterMode.CopterModeLand:
                    return "Land";
                case CopterMode.CopterModeDrift:
                    return "Drift";
                case CopterMode.CopterModeSport:
                    return "Sport";
                case CopterMode.CopterModeFlip:
                    return "Flip";
                case CopterMode.CopterModeAutotune:
                    return "Autotune";
                case CopterMode.CopterModePoshold:
                    return "Position hold";
                case CopterMode.CopterModeBrake:
                    return "Brake";
                case CopterMode.CopterModeThrow:
                    return "Throw";
                case CopterMode.CopterModeAvoidAdsb:
                    return "Avoid ADSB";
                case CopterMode.CopterModeGuidedNogps:
                    return "Guided no GPS";
                case CopterMode.CopterModeSmartRtl:
                    return "Smart RTL";
                default:
                    return customMode.ToString("G").Replace("CopterMode", string.Empty);
            }
        }
    }

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
                       .Select(_ => new VehicleCustomMode {Name = _.GetDisplayName(), Value = (uint) _}).ToArray());
        }

        protected override VehicleMode Interpret(HeartbeatPayload heartbeat)
        {
            var customMode = (CopterMode) (heartbeat.CustomMode);
            return new VehicleMode
            {
                BaseMode = heartbeat.BaseMode,
                CustomMode = new VehicleCustomMode
                {
                    Name = customMode.GetDisplayName(),
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

        public override async Task FlyByLineGlob(GeoPoint start, GeoPoint stop, double precisionMet, CancellationToken cancel, Action firstPointComplete = null)
        {
            await GoToGlobAndWait(start, new Microsoft.Progress<double>(_ => { }), precisionMet, cancel);
            firstPointComplete?.Invoke();
            await GoToGlobAndWait(stop, new Microsoft.Progress<double>(_ => { }), precisionMet, cancel);
        }

        public override Task DoRtl(CancellationToken cancel)
        {
            return Mavlink.Common.SetMode(1, (int)CopterMode.CopterModeRtl, cancel);
        }
    }
}