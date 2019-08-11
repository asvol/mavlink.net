using System;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;
using Nito.AsyncEx;

namespace Asv.Mavlink
{
  
    public class MissionClient : IMissionClient,IDisposable
    {
        private readonly IMavlinkV2Connection _mavlink;
        private readonly PacketSequenceCalculator _seq;
        private readonly MavlinkClientIdentity _config;

        public MissionClient(IMavlinkV2Connection mavlink, PacketSequenceCalculator seq,
            MavlinkClientIdentity config)
        {
            if (mavlink == null) throw new ArgumentNullException(nameof(mavlink));
            if (seq == null) throw new ArgumentNullException(nameof(seq));
            if (config == null) throw new ArgumentNullException(nameof(config));
            _mavlink = mavlink;
            _seq = seq;
            _config = config;
        }

        public Task MissionItem(MavFrame frame, MavCmd cmd, bool current, bool autoContinue, float param1, float param2, float param3,
            float param4, float x, float y, float z, MavMissionType missionType, int attemptCount, CancellationToken cancel)
        {
            var packet = new MissionItemPacket()
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Sequence = _seq.GetNextSequenceNumber(),
                Payload =
                {
                    TargetComponent = _config.TargetComponentId,
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

        public void Dispose()
        {
            _mavlink?.Dispose();
        }
    }
}