using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
  
    public class MavParam
    {
        public ushort Index { get; set; }
        public string Name { get; set; }
        public MavParamType Type { get; set; }
        public float? RealValue { get; set; }
        public long? IntegerValue { get; set; }
    }

    public interface IVehicle:IDisposable
    {
        IRxValue<LinkState> Link { get; }
        IRxValue<int> PacketRateHz { get; }
        IRxValue<HeartbeatPayload> Heartbeat { get; }
        IRxValue<SysStatusPayload> SysStatus { get; }
        IRxValue<GpsRawIntPayload> GpsRawInt { get; }
        IRxValue<HighresImuPayload> HighresImu { get; }
        IRxValue<ExtendedSysStatePayload> ExtendedSysState { get; }
        IRxValue<AltitudePayload> Altitude { get; }
        IRxValue<BatteryStatusPayload> BatteryStatus { get; }
        IRxValue<AttitudePayload> Attitude { get; }
        IRxValue<VfrHudPayload> VfrHud { get; }
        IRxValue<GeoPoint> Gps { get; }
        IRxValue<HomePositionPayload> Home { get; }
        Task<CommandAckPayload> SendCommand(MavCmd command, float param1, float param2, float param3, float param4, float param5, float param6, float param7, int atteptCount, CancellationToken cancel);

        IReadOnlyDictionary<string,MavParam> Params { get; }
        IRxValue<int?> ParamsCount { get; }
        IObservable<MavParam> OnParamUpdated { get; }
        Task UpdateAllParams(CancellationToken cancel,IProgress<double> progress = null);
        Task<MavParam> UpdateParam(string name, CancellationToken cancel);
        Task<MavParam> UpdateParam(int index, CancellationToken cancel);
       
    }

    public static class VehicleHelper
    {
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
    }
}
