using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public interface IMavlinkTelemetry
    {
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
        IRxValue<GlobalPositionIntPayload> RawGlobalPositionInt { get; }
        
        
    }
}