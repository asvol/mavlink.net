using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public interface IMissionClient
    {
        Task<MissionItemIntPayload[]> MissionRequest(int attemptCount, CancellationToken cancel);

        Task<MissionItemIntPayload> MissionRequestItem(ushort index, int attemptCount, CancellationToken cancel);
        /// <summary>
        /// Initiate mission download from a system by requesting the list of mission items.
        /// </summary>
        /// <param name="attemptCount"></param>
        /// <param name="cancel"></param>
        /// <returns></returns>
        Task<int> MissionRequestCount(int attemptCount, CancellationToken cancel);

        /// <summary>
        /// 
        /// </summary>
        /// <param name="command">The scheduled action for the mission item.</param>
        /// <param name="frame">The coordinate system of the COMMAND.</param>
        /// <param name="current"></param>
        /// <param name="autoContinue">autocontinue to next wp</param>
        /// <param name="param1">PARAM1, see MAV_CMD enum</param>
        /// <param name="param2">PARAM2, see MAV_CMD enum</param>
        /// <param name="param3">PARAM3, see MAV_CMD enum</param>
        /// <param name="param4">PARAM4, see MAV_CMD enum</param>
        /// <returns></returns>
        Task WriteMissionItem(MavFrame frame, MavCmd cmd, bool current, bool autoContinue, float param1, float param2, float param3, float param4, float x, float y, float z, MavMissionType missionType, int attemptCount, CancellationToken cancel);
    }

    

}
