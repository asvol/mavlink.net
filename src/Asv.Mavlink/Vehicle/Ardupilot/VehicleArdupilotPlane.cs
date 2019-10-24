using System;
using System.Collections.Generic;
using System.Linq;
using System.Reactive.Linq;
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

    public class VehicleArdupilotPlane : VehicleArdupilot, IVehiclePlane
    {
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        

        public VehicleArdupilotPlane(IMavlinkClient mavlink, VehicleBaseConfig config) : base(mavlink, config)
        {
            _rotationDirection.OnNext(RotationDirection.Cw);
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

//        public override async Task FlyByLineGlob(GeoPoint start, GeoPoint stop, double precisionMet, CancellationToken cancel, Action firstPointComplete = null)
//        {
//            
//            const int PrePointDistanceParts = 2;
//            const int GoToPrecisionErrorMet = 20;
//            var waitInPrePointTime = TimeSpan.FromSeconds(3);
//            var waitForCorrectionTime = TimeSpan.FromSeconds(1);
//
//
//            var loiterRadius = (_planeRadius.Value ?? await _planeRadius.FirstAsync(_=>_.HasValue)).Value;
//            var prePointDistance = (int) (PrePointDistanceParts * loiterRadius);
//
//            var azimuth = start.Azimuth(stop);
//            var prePoint = start.RadialPoint(prePointDistance, azimuth - 180);
//            _logger.Info($"GoTo prepoint {prePoint}");
//            await GoToGlobAndWait(prePoint, new Microsoft.Progress<double>(), loiterRadius + GoToPrecisionErrorMet, cancel);
//            _logger.Info($"Arrived at prepoint, wait {waitInPrePointTime:g}");
//            await Task.Delay(waitInPrePointTime, cancel);
//
//            while (true)
//            {
//                if (GpsLocation.Value.DistanceTo(start) > loiterRadius + GoToPrecisionErrorMet) break;
//                var angleangle = GpsLocation.Value.Azimuth(start);
//                var dist = start.DistanceTo(stop);
//                var nextPoint = start.RadialPoint(dist, angleangle);
//                _logger.Info($"Correct direction {nextPoint:g}");
//                await GoToGlob(nextPoint,cancel);
//                await Task.Delay(waitForCorrectionTime, cancel);
//            }
//        }

        private bool IsInAzimuthLimits(double azimuth, double targetAzimuth, double precisionDegr)
        {
            if (targetAzimuth >= 360) targetAzimuth = targetAzimuth % 360.0;
            else if (targetAzimuth < 0) targetAzimuth = 360 - targetAzimuth % 360.0;

            if (azimuth >= 360) azimuth = azimuth % 360.0;
            else if (azimuth < 0) azimuth = 360 - azimuth % 360.0;

            var from = targetAzimuth - precisionDegr;
            var to = targetAzimuth + precisionDegr;

            if (from < 0 || to >= 360)
            {
                to = to - from;
                azimuth = azimuth - from;
                from = 0;
                if (azimuth >= 360) azimuth = azimuth % 360.0;
                else if (azimuth < 0) azimuth = 360 - azimuth % 360.0;
            }

            return azimuth <= to && azimuth >= from;
        }

        private async Task<double> GoToPointUntilReachAzimuth(GeoPoint point, double azimuth, double precisionMet, double precisionDegr, CancellationToken cancel, int attemptsCnt = 1)
        {
            _logger.Info($"GoTo point {point}");
            await GoToGlobAndWait(point, new Microsoft.Progress<double>(), precisionMet, cancel);
            _logger.Info("Start circling");

            var attempts = 0;
            while (!cancel.IsCancellationRequested)
            {
                var location = GpsLocation.Value;
                var currentAzimuth = point.Azimuth(location);
                _logger.Info($"Azimuth relative to the point {currentAzimuth:F1} deg");
                if (IsInAzimuthLimits(currentAzimuth, azimuth, precisionDegr))
                {
                    attempts++;
                    if (attempts >= attemptsCnt)
                        return GeoMath.Distance(location, point);
                }
                await Task.Delay(TimeSpan.FromMilliseconds(250), cancel);
            }

            return 0;
        }

        public override async Task FlyByLineGlob(GeoPoint start, GeoPoint stop, double precisionMet, CancellationToken cancel, Action firstPointComplete = null)
        {
            const int ApproachAngle = 90;
            const int PrePointDistanceParts = 2;
            const double GoToPrecisionErrorMet = 20.5;

            var waitForDetectingAzimuthTime = TimeSpan.FromSeconds(3);
            var waitInPrePointTime = TimeSpan.FromSeconds(3);
            var waitForCorrectionTime = TimeSpan.FromSeconds(1);

            var azimuth = start.Azimuth(stop);
            var reverseAzimuth = stop.Azimuth(start);
            var loiterRadius = (_planeRadius.Value ?? await _planeRadius.FirstAsync(_ => _.HasValue)).Value;
            var realLoiterRadius = (int)(loiterRadius + GoToPrecisionErrorMet);

            var start0 = start.SetAltitude(0);
            var stop0 = stop.SetAltitude(0);

            var startPrePoint0 = start.RadialPoint(PrePointDistanceParts * realLoiterRadius, reverseAzimuth);
            var glideAngle = GeoMath.RadiansToDegrees(Math.Atan(Math.Abs((start.Altitude ?? 0) - (stop.Altitude ?? 0)) / GeoMath.Distance(start0, stop0)));

            var height = GeoMath.HeightFromGroundRange(GeoMath.Distance(stop0, startPrePoint0), glideAngle);

            if ((start.Altitude ?? 0) - (stop.Altitude ?? 0) < 0)
                height = (stop.Altitude ?? 0) - height;
            else
                height = (stop.Altitude ?? 0) + height;

            //TODO if (height <= 0) нужно что-то делать!!!
            if (height <= 0)
            {
                _logger.Info($"Impossible fly by line from altitude {start.Altitude ?? 0}м to altitude {stop.Altitude ?? 0}m");
                return;
            }

            var firstPrePoint = startPrePoint0.RadialPoint(PrePointDistanceParts * realLoiterRadius, azimuth + ApproachAngle).SetAltitude(height);

            var r = await GoToPointUntilReachAzimuth(firstPrePoint, reverseAzimuth - 10, realLoiterRadius, 4, cancel, 2);
            var secondPrePoint = startPrePoint0.RadialPoint(r, azimuth + ApproachAngle).SetAltitude(height);

            await GoToPointUntilReachAzimuth(secondPrePoint, reverseAzimuth + ApproachAngle - 5, realLoiterRadius, 2, cancel, 2);
            await GoToGlobAndWait(stop, new Microsoft.Progress<double>(), realLoiterRadius, cancel);
        }

        public override Task DoRtl(CancellationToken cancel)
        {
            return Mavlink.Common.SetMode(1, (int)PlaneMode.PlaneModeRtl, cancel);
        }

        public override void StartListen()
        {
            base.StartListen();
            InitPlaneRadius();
        }

        #region Plane radius

        private void InitPlaneRadius()
        {
            const string paramName = "WP_LOITER_RAD";
            Params.OnParamUpdated.Where(_ => _.Name == paramName).Select(_ => (double?) _.IntegerValue.Value)
                .Subscribe(_planeRadius, DisposeCancel.Token);
            _updatePlaneRaidus = Observable.Timer(TimeSpan.FromMilliseconds(10)).Subscribe(_=>TryUpdateRadius(paramName));
            DisposeCancel.Token.Register(() => _updatePlaneRaidus?.Dispose());
        }

        private async void TryUpdateRadius(string paramName)
        {
            try
            {
                var value = await Params.GetOrReadFromVehicleParam(paramName, DisposeCancel.Token);
                _planeRadius.OnNext(value.IntegerValue);
            }
            catch (Exception e)
            {
                _logger.Error($"Error to read plane radius param '{paramName}':{e.Message}");
                _updatePlaneRaidus = Observable.Timer(TimeSpan.FromMilliseconds(3000)).Subscribe(_ => TryUpdateRadius(paramName));
            }
        }

        private enum RotationDirection
        {
            Cw,
            Ccw
        }
        
        private readonly RxValue<RotationDirection> _rotationDirection = new RxValue<RotationDirection>();
        private readonly RxValue<double?> _planeRadius = new RxValue<double?>();
        private IDisposable _updatePlaneRaidus;

        public IRxValue<double?> PlaneCircleRadius => _planeRadius;

        #endregion
    }
}