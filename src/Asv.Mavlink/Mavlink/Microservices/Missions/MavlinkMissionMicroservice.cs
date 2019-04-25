using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;
using Nito.AsyncEx;

namespace Asv.Mavlink
{
    public class VehicleMissionProtocolConfig : MicroserviceConfigBase
    {

    }

    public class MavlinkMissionMicroservice : IMavlinkMissionMicroservice,IDisposable
    {
        private readonly IMavlinkV2Connection _mavlink;
        private readonly VehicleMissionProtocolConfig _config;
        private int _seq = 0;

        public MavlinkMissionMicroservice(IMavlinkV2Connection mavlink, VehicleMissionProtocolConfig config)
        {
            if (mavlink == null) throw new ArgumentNullException(nameof(mavlink));
            if (config == null) throw new ArgumentNullException(nameof(config));
            _mavlink = mavlink;
            _config = config;
        }

        public Task MissionItem(MavFrame frame, MavCmd cmd, bool current, bool autoContinue, float param1, float param2, float param3,
            float param4, float x, float y, float z, MavMissionType missionType, int attemptCount, CancellationToken cancel)
        {
            var packet = new MissionItemPacket()
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    TargetComponent = _config.TargetComponenId,
                    TargetSystem = _config.TargetSystemId,
                    Seq = 0,
                    Frame = frame,
                    Command = cmd,
                    Current = 2,
                    Autocontinue = (byte) (autoContinue? 1:0),
                    Param1 = param1,
                    Param2 = param2,
                    Param3 = param3,
                    Param4 = param4,
                    X = x,
                    Y = y,
                    Z = 20,
                    MissionType = missionType
                }
            };
            _mavlink.Send(packet, cancel);
            return Task.CompletedTask;
        }

        private bool FilterVehicle(IPacketV2<IPayload> packetV2)
        {
            if (_config.TargetSystemId != 0 && _config.TargetSystemId != packetV2.SystemId) return false;
            if (_config.TargetComponenId != 0 && _config.TargetComponenId != packetV2.ComponenId) return false;
            return true;
        }


        public void Dispose()
        {
            _mavlink?.Dispose();
        }
    }
}