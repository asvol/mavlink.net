using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Client;
using Asv.Mavlink.V2.Ardupilotmega;
using Asv.Mavlink.V2.Common;
using NLog;
using MavCmd = Asv.Mavlink.V2.Common.MavCmd;

namespace Asv.Mavlink
{
    public static class PlaneModeHelper
    {
        public static string GetDisplayName(this PlaneMode customMode)
        {
            switch (customMode)
            {
                case PlaneMode.PlaneModeManual:
                    return "Manual";
                case PlaneMode.PlaneModeCircle:
                    return "Circle";
                case PlaneMode.PlaneModeStabilize:
                    return "Stabilize";
                case PlaneMode.PlaneModeTraining:
                    return "Training";
                case PlaneMode.PlaneModeAcro:
                    return "Acro";
                case PlaneMode.PlaneModeFlyByWireA:
                    return "Fly by wire A";
                case PlaneMode.PlaneModeFlyByWireB:
                    return "Fly by wire B";
                case PlaneMode.PlaneModeCruise:
                    return "Cruise";
                case PlaneMode.PlaneModeAutotune:
                    return "Autotune";
                case PlaneMode.PlaneModeAuto:
                    return "Auto";
                case PlaneMode.PlaneModeRtl:
                    return "RTL";
                case PlaneMode.PlaneModeLoiter:
                    return "Loiter";
                case PlaneMode.PlaneModeAvoidAdsb:
                    return "Avoid ADSB";
                case PlaneMode.PlaneModeGuided:
                    return "Guided";
                case PlaneMode.PlaneModeInitializing:
                    return "Initializing";
                case PlaneMode.PlaneModeQstabilize:
                    return "Quad Stabilize";
                case PlaneMode.PlaneModeQhover:
                    return "Quad Hover";
                case PlaneMode.PlaneModeQloiter:
                    return "Quad Loiter";
                case PlaneMode.PlaneModeQland:
                    return "Quad Land";
                case PlaneMode.PlaneModeQrtl:
                    return "Quad RTL";
                default:
                    return customMode.ToString("G").Replace("PlaneMode", string.Empty);
            }
        }
    }

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
                       .Select(_ => new VehicleCustomMode { Name = _.GetDisplayName(), Value = (uint)_ }).ToArray());
        }

        protected override VehicleMode Interpret(HeartbeatPayload heartbeat)
        {
            var customMode = (PlaneMode)(heartbeat.CustomMode);
            return new VehicleMode
            {
                BaseMode = heartbeat.BaseMode,
                CustomMode = new VehicleCustomMode
                {
                    Name = customMode.GetDisplayName(),
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

            
            await Mavlink.Mission.MissionItem(MavFrame.MavFrameGlobalRelativeAlt, MavCmd.MavCmdNavLoiterUnlim, true,
                    true, 0, 0, 100,
                    0, (float) location.Latitude, (float) location.Longitude, (float) rel.Value,
                    MavMissionType.MavMissionTypeMission, 3, cancel);
                //            await Mavlink.Mission.MissionItem(MavFrame.MavFrameGlobalRelativeAlt, MavCmd.MavCmdNavWaypoint, true,
                //                    true, 0, 0, 0,
                //                    0, (float) location.Latitude, (float) location.Longitude, (float) rel.Value,
                //                    MavMissionType.MavMissionTypeMission, 3, cancel);
        }

        public override async Task FlyByLineGlob(GeoPoint start, GeoPoint stop, double precisionMet, CancellationToken cancel, Action firstPointComplete = null)
        {
            const string paramName = "WP_LOITER_RAD";
            const int PrePointDistanceParts = 2;
            const int GoToPrecisionErrorMet = 20;
            var waitInPrePointTime = TimeSpan.FromSeconds(3);
            var waitForCorrectionTime = TimeSpan.FromSeconds(1);

            _logger.Info($"Try read circle radius param for plane '{paramName}'");
            var value = await this.Params.GetOrReadFromVehicleParam(paramName, cancel);
            var loiterRadius = value.IntegerValue.Value;
            _logger.Info($"{paramName} = {loiterRadius} meter");
            var prePointDistance = (int) (PrePointDistanceParts * loiterRadius);

            var azimuth = start.Azimuth(stop);
            var prePoint = start.RadialPoint(prePointDistance, azimuth - 180);
            _logger.Info($"GoTo prepoint {prePoint}");
            await GoToGlobAndWait(prePoint, new Microsoft.Progress<double>(), loiterRadius + GoToPrecisionErrorMet, cancel);
            _logger.Info($"Arrived at prepoint, wait {waitInPrePointTime:g}");
            await Task.Delay(waitInPrePointTime, cancel);

            while (true)
            {
                if (GpsLocation.Value.DistanceTo(start) > loiterRadius + GoToPrecisionErrorMet) break;
                var angleangle = GpsLocation.Value.Azimuth(start);
                var dist = start.DistanceTo(stop);
                var nextPoint = start.RadialPoint(dist, angleangle);
                _logger.Info($"Correct direction {nextPoint:g}");
                await GoToGlob(nextPoint,cancel);
                await Task.Delay(waitForCorrectionTime, cancel);
            }
        }

        public override Task DoRtl(CancellationToken cancel)
        {
            return Mavlink.Common.SetMode(1, (int)PlaneMode.PlaneModeRtl, cancel);
        }
    }
}