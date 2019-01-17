using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;
using NLog;

namespace Asv.Mavlink
{
    public interface IVehicle:IDisposable
    {
        IRxValue<Exception> PortError {get; }
        IObservable<DeserizliaePackageException> OnPacketErrors { get; }
    

        IRxValue<LinkState> Link { get; }
        IRxValue<int> PacketRateHz { get; }
        IRxValue<GeoPoint> RelGps { get; }
        IRxValue<GeoPoint> GlobGps { get; }
        IRxValue<GeoPoint> Home { get; }
        IRxValue<bool> Armed { get; }

        #region Raw data

        IRxValue<HeartbeatPayload> RawHeartbeat { get; }
        IRxValue<SysStatusPayload> RawSysStatus { get; }
        IRxValue<GpsRawIntPayload> RawGpsRawInt { get; }
        IRxValue<HighresImuPayload> RawHighresImu { get; }
        IRxValue<ExtendedSysStatePayload> RawExtendedSysState { get; }
        IRxValue<AltitudePayload> RawAltitude { get; }
        IRxValue<BatteryStatusPayload> RawBatteryStatus { get; }
        IRxValue<AttitudePayload> RawAttitude { get; }
        IRxValue<VfrHudPayload> RawVfrHud { get; }
        IRxValue<HomePositionPayload> RawHome { get; }
        IRxValue<StatustextPayload> RawStatusText { get; }

        #endregion

        Task<CommandAckPayload> SendCommand(MavCmd command, float param1, float param2, float param3, float param4, float param5, float param6, float param7, int atteptCount, CancellationToken cancel);

        IReadOnlyDictionary<string,MavParam> Params { get; }
        IRxValue<int?> ParamsCount { get; }
        IObservable<MavParam> OnParamUpdated { get; }
        Task ReadAllParams(CancellationToken cancel,IProgress<double> progress = null);
        Task<MavParam> ReadParam(string name, CancellationToken cancel);
        Task<MavParam> ReadParam(short index, CancellationToken cancel);
        Task<MavParam> WriteParam(MavParam param, CancellationToken cancel);

        /// <summary>
        /// 
        /// </summary>
        /// <param name="timeBootMs">Timestamp (time since system boot).</param>
        /// <param name="coordinateFrame"> Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9</param>
        /// <param name="typeMask">Bitmap to indicate which dimensions should be ignored by the vehicle.</param>
        /// <param name="x">X Position in NED frame</param>
        /// <param name="y">Y Position in NED frame</param>
        /// <param name="z">Z Position in NED frame (note, altitude is negative in NED)</param>
        /// <param name="vx">X velocity in NED frame</param>
        /// <param name="vy">Y velocity in NED frame</param>
        /// <param name="vz">Z velocity in NED frame</param>
        /// <param name="afx">X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N</param>
        /// <param name="afy">Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N</param>
        /// <param name="afz">Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N</param>
        /// <param name="yaw">yaw setpoint</param>
        /// <param name="yawRate">yaw rate setpoint</param>
        /// <param name="cancel"></param>
        /// <returns></returns>
        Task SetPositionTargetLocalNed(uint timeBootMs, MavFrame coordinateFrame, PositionTargetTypemask typeMask, float x,
            float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yawRate,
            CancellationToken cancel);

    }

    public static class VehicleHelper
    {
        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();



        public static async Task<MavParam> WriteParam(this IVehicle src, string name, float value, CancellationToken cancel)
        {
            MavParam param;
            if (!src.Params.TryGetValue(name, out param))
            {
                param = await src.ReadParam(name, cancel).ConfigureAwait(false);
            }

            return await src.WriteParam(new MavParam(param, value), cancel).ConfigureAwait(false);
        }

        public static async Task<MavParam> WriteParam(this IVehicle src, string name, long value, CancellationToken cancel)
        {
            MavParam param;
            if (!src.Params.TryGetValue(name, out param))
            {
                param = await src.ReadParam(name, cancel).ConfigureAwait(false);
            }

            return await src.WriteParam(new MavParam(param, value), cancel).ConfigureAwait(false);
        }

        /// <summary>
        /// Takeoff from ground / hand
        /// </summary>
        /// <param name="src"></param>
        /// <param name="minimumPitch">Minimum pitch (if airspeed sensor present), desired pitch without sensor</param>
        /// <param name="yawAngle">Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.</param>
        /// <param name="cancel"></param>
        /// <param name="cancel"></param>
        /// <returns></returns>
        public static Task<CommandAckPayload> TakeOff(this IVehicle src, float minimumPitch, float yawAngle, GeoPoint point, CancellationToken cancel)
        {
            return src.TakeOff(minimumPitch, yawAngle, (float) point.Latitude, (float) point.Longitude, (float) point.Altitude, cancel);
        }

        /// <summary>
        /// Takeoff from ground / hand
        /// </summary>
        /// <param name="src"></param>
        /// <param name="minimumPitch">Minimum pitch (if airspeed sensor present), desired pitch without sensor</param>
        /// <param name="yawAngle">Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.</param>
        /// <param name="latitude">Latitude</param>
        /// <param name="longitude">Longitude</param>
        /// <param name="altitude">Altitude</param>
        /// <param name="cancel"></param>
        /// <returns></returns>
        public static Task<CommandAckPayload> TakeOff(this IVehicle src, float minimumPitch, float yawAngle, float latitude, float longitude, float altitude, CancellationToken cancel)
        {
            return src.SendCommand(MavCmd.MavCmdNavTakeoff, minimumPitch, float.NaN, float.NaN, yawAngle, latitude,longitude,altitude,3,cancel);
        }

        /// <summary>
        /// Arms / Disarms a component
        /// </summary>
        /// <param name="src"></param>
        /// <param name="state">true to arm, false to disarm</param>
        /// <param name="cancel"></param>
        /// <returns></returns>
        public static Task<CommandAckPayload> ArmDisarm(this IVehicle src, bool state, CancellationToken cancel)
        {
            return src.SendCommand(MavCmd.MavCmdComponentArmDisarm, state ? 1:0, float.NaN, float.NaN, float.NaN, float.NaN, float.NaN, float.NaN, 1, cancel);
        }

        /// <summary>
        /// Arms a component
        /// </summary>
        /// <returns></returns>
        public static Task<CommandAckPayload> Arm(this IVehicle src, CancellationToken cancel)
        {
            return src.ArmDisarm(true, cancel);
        }
        /// <summary>
        /// Disarms a component
        /// </summary>
        /// <returns></returns>
        public static Task<CommandAckPayload> Disarm(this IVehicle src, CancellationToken cancel)
        {
            return src.ArmDisarm(false, cancel);
        }

        /// <summary>
        /// Reposition the vehicle to a specific WGS84 global position.
        /// </summary>
        /// <returns></returns>
        public static Task<CommandAckPayload> GoTo(this IVehicle src, float groundSpeed, GeoPoint newPosition , CancellationToken cancel)
        {
            return src.GoTo(groundSpeed,true,-1,(float) newPosition.Latitude,(float) newPosition.Longitude,(float) newPosition.Altitude.Value,cancel);
        }

        /// <summary>
        /// Reposition the vehicle to a specific WGS84 global position. Altitude - relative from home altitude.
        /// </summary>
        /// <returns></returns>
        public static Task<CommandAckPayload> GoToRelative(this IVehicle src, float groundSpeed, GeoPoint newRelativePosition, CancellationToken cancel)
        {
            return src.GoTo(groundSpeed, true, -1, (float)newRelativePosition.Latitude, (float)newRelativePosition.Longitude, (float)newRelativePosition.Altitude.Value + (float)src.Home.Value.Altitude, cancel);
        }
        
        public static async Task GoToAndWait(this IVehicle vehicle, GeoPoint geoPoint, double velocity, double precisionMet, int checkTimeMs, CancellationToken cancel, IProgress<double> progress)
        {
            progress = progress ?? new Progress<double>();
            var startLocation = vehicle.RelGps.Value;
            var startDistance = GeoMath.Distance(geoPoint, startLocation);

            Logger.Info("GoToAndWait {0} with V={1:F1} m/sec and precision {2:F1} m. Distance to target {3:F1}", geoPoint, velocity, precisionMet, startDistance);
            progress.Report(0);
            if (startDistance <= precisionMet)
            {
                Logger.Debug("Already in target, nothing to do", startLocation);
                progress.Report(1);
                return;
            }

            var sw = new Stopwatch();
            sw.Start();
            Logger.Debug("Send command GoTo to vehicle", startLocation);
            await vehicle.GoTo((float) velocity, geoPoint, cancel).ConfigureAwait(false);
            double dist = 0;
            while (!cancel.IsCancellationRequested)
            {
                var loc = vehicle.RelGps.Value;
                dist = Math.Abs(GeoMath.Distance(geoPoint, loc));
                var prog = 1 - dist / startDistance;
                Logger.Trace("Distance to target {0:F1}, location: {1}, progress {2:P2}", dist, loc, prog);
                progress.Report(prog);
                if (dist <= precisionMet) break;
                await Task.Delay(checkTimeMs, cancel).ConfigureAwait(false);
            }
            sw.Stop();
            Logger.Info($"Complete {sw.Elapsed:hh\\:mm\\:ss} location error {dist:F1} m");
            progress.Report(1);
        }

        /// <summary>
        /// Reposition the vehicle to a specific WGS84 global position.
        /// </summary>
        /// <returns></returns>
        public static Task<CommandAckPayload> GoTo(this IVehicle src, float groundSpeed,bool switchToGuided, float yaw, float lat,float lon,float alt, CancellationToken cancel)
        {
            return src.SendCommand(MavCmd.MavCmdDoReposition, groundSpeed, switchToGuided ? (float)MavDoRepositionFlags.MavDoRepositionFlagsChangeMode : 0,float.NaN,yaw,lat,lon,alt,3,cancel);
        }

        public static async Task MoveUp(this IVehicle vehicle, double moveDistance, double moveVelocity, CancellationToken cancel)
        {
            var loc = vehicle.RelGps.Value;
            loc = loc.AddAltitude(moveDistance);
            await vehicle.GoTo((float) moveVelocity,loc, cancel);
        }

        public static async Task MoveDown(this IVehicle vehicle, double moveDistance, double moveVelocity, CancellationToken cancel)
        {
            var loc = vehicle.RelGps.Value;
            loc = loc.AddAltitude(-moveDistance);
            await vehicle.GoTo((float) moveVelocity,loc, cancel);
        }

        public static Task MoveN(this IVehicle vehicle, double moveDistance, double moveVelocity, CancellationToken cancel)
        {
            return MoveRadial(vehicle, moveDistance, moveVelocity, 0, cancel);
        }

        public static Task MoveE(this IVehicle vehicle, double moveDistance, double moveVelocity, CancellationToken cancel)
        {
            return MoveRadial(vehicle, moveDistance, moveVelocity, 90, cancel);
        }

        public static Task MoveW(this IVehicle vehicle, double moveDistance, double moveVelocity, CancellationToken cancel)
        {
            return MoveRadial(vehicle, moveDistance, moveVelocity, 270, cancel);
        }

        public static Task MoveS(this IVehicle vehicle, double moveDistance, double moveVelocity, CancellationToken cancel)
        {
            return MoveRadial(vehicle, moveDistance, moveVelocity, 180, cancel);
        }

        public static async Task MoveRadial(this IVehicle vehicle, double moveDistance, double moveVelocity, int radial, CancellationToken cancel)
        {
            var loc = vehicle.RelGps.Value;
            var alt = loc.Altitude ?? 0;
            loc = GeoMath.RadialPoint(loc.Latitude, loc.Longitude, moveDistance, radial);
            loc = loc.AddAltitude(alt);
            await vehicle.GoTo((float) moveVelocity, loc, cancel).ConfigureAwait(false);
        }



        public static Task SetPositionTargetLocalNed(this IVehicle vehicle, uint timeBootMs, MavFrame coordinateFrame, float? x,
            float? y, float? z, float? vx, float? vy, float? vz, float? afx, float? afy, float? afz, float? yaw, float? yawRate,
            CancellationToken cancel)
        {
            PositionTargetTypemask mask = 0;
            if (!x.HasValue) mask |= PositionTargetTypemask.PositionTargetTypemaskXIgnore;
            if (!y.HasValue) mask |= PositionTargetTypemask.PositionTargetTypemaskYIgnore;
            if (!z.HasValue) mask |= PositionTargetTypemask.PositionTargetTypemaskZIgnore;

            if (!vx.HasValue) mask |= PositionTargetTypemask.PositionTargetTypemaskVxIgnore;
            if (!vy.HasValue) mask |= PositionTargetTypemask.PositionTargetTypemaskVyIgnore;
            if (!vz.HasValue) mask |= PositionTargetTypemask.PositionTargetTypemaskVzIgnore;

            if (!afx.HasValue) mask |= PositionTargetTypemask.PositionTargetTypemaskAxIgnore;
            if (!afy.HasValue) mask |= PositionTargetTypemask.PositionTargetTypemaskVyIgnore;
            if (!afz.HasValue) mask |= PositionTargetTypemask.PositionTargetTypemaskVzIgnore;

            if (!yaw.HasValue) mask |= PositionTargetTypemask.PositionTargetTypemaskYawIgnore;
            if (!yawRate.HasValue) mask |= PositionTargetTypemask.PositionTargetTypemaskYawRateIgnore;

            return vehicle.SetPositionTargetLocalNed(timeBootMs,coordinateFrame,mask,x ?? 0, y ?? 0, z ?? 0, vx ?? 0, vy ?? 0, vz ?? 0, afx ?? 0, afy ?? 0, afz ?? 0, yaw ?? 0, yawRate ?? 0, cancel);
        }

        

    }
}
