using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
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
        Task<CommandAckPayload> SendCommand(MavCmd command, float param1, float param2, float param3, float param4, float param5, float param6, float param7, int atteptCount, CancellationToken cancel);
    }

    public static class VehicleHelper
    {
        public static async Task TakeOff(this IVehicle src, float minimumPitch, float yawAngle, float latitude, float longitude, float altitude, CancellationToken cancel)
        {
            var a = await src.SendCommand(MavCmd.MavCmdNavTakeoff, minimumPitch, float.NaN, float.NaN, yawAngle, latitude,longitude,altitude,1,cancel).ConfigureAwait(false);
        }
    }
}
