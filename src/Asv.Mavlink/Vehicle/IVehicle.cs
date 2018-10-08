using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public interface IVehicle:IDisposable
    {
        IRxValue<LinkState> Link { get; }
        IRxValue<int> PacketRateHz { get; }
        IRxValue<GeoPoint> Gps { get; }
        IRxValue<GeoPoint> Home { get; }

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

    }

    public static class VehicleHelper
    {
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
        /// <param name="latitude">Latitude</param>
        /// <param name="longitude">Longitude</param>
        /// <param name="altitude">Altitude</param>
        /// <param name="cancel"></param>
        /// <returns></returns>
        public static Task<CommandAckPayload> TakeOff(this IVehicle src, float minimumPitch, float yawAngle, float latitude, float longitude, float altitude, CancellationToken cancel)
        {
            return src.SendCommand(MavCmd.MavCmdNavTakeoff, minimumPitch, float.NaN, float.NaN, yawAngle, latitude,longitude,altitude,1,cancel);
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
        public static Task<CommandAckPayload> DoReposition(this IVehicle src, float groundSpeed,bool switchToGuided, float yaw, float lat,float lon,float alt, CancellationToken cancel)
        {
            return src.SendCommand(MavCmd.MavCmdDoReposition, groundSpeed, switchToGuided ? (float)MavDoRepositionFlags.MavDoRepositionFlagsChangeMode : 0,float.NaN,yaw,lat,lon,alt,1,cancel);
        }

    }
}
