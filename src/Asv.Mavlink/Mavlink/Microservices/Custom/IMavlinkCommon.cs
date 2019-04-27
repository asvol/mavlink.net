using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public interface IMavlinkCommon:IDisposable
    {
        Task SetMode(uint baseMode, uint customMode, CancellationToken cancel);



        /// <summary>
        /// Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
        /// </summary>
        /// <param name="timeBootMs">Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.</param>
        /// <param name="coordFrame">Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11</param>
        /// <param name="latInt">X Position in WGS84 frame</param>
        /// <param name="lonInt">Y Position in WGS84 frame</param>
        /// <param name="alt">Altitude (MSL, Relative to home, or AGL - depending on frame)</param>
        /// <param name="vx">X velocity in NED frame</param>
        /// <param name="vy">Y velocity in NED frame</param>
        /// <param name="vz">Z velocity in NED frame</param>
        /// <param name="afx">X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N</param>
        /// <param name="afy">Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N</param>
        /// <param name="afz">Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N</param>
        /// <param name="yaw">yaw setpoint</param>
        /// <param name="yawRate">yaw rate setpoint</param>
        /// <param name="typeMask">Bitmap to indicate which dimensions should be ignored by the vehicle.</param>
        /// <param name="cancel"></param>
        /// <returns></returns>
        Task SetPositionTargetGlobalInt(uint timeBootMs, MavFrame coordFrame, int latInt, int lonInt, float alt, float vx,
            float vy, float vz, float afx, float afy, float afz, float yaw, float yawRate,
            PositionTargetTypemask typeMask, CancellationToken cancel);
    }
}