using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;
using NLog;

namespace Asv.Mavlink
{
    public class MavlinkCommon : IMavlinkCommon
    {
        private readonly IMavlinkV2Connection _connection;
        private readonly MicroserviceConfigBase _config;
        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();

        public MavlinkCommon(IMavlinkV2Connection connection, MicroserviceConfigBase config)
        {
            _connection = connection;
            _config = config;
        }

        public Task SetMode(uint baseMode, uint customMode, CancellationToken cancel)
        {
            var packet = new SetModePacket()
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    TargetSystem = _config.TargetSystemId,
                    BaseMode = (MavMode)baseMode,
                    CustomMode = customMode,
                }
            };
            return _connection.Send(packet, cancel);
        }

        public Task RequestDataStream(int streamId, int rateHz, bool startStop,CancellationToken cancel)
        {
            var packet = new RequestDataStreamPacket
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    TargetSystem = _config.TargetSystemId,
                    TargetComponent = _config.TargetComponenId,
                    ReqStreamId = (byte) streamId,
                    ReqMessageRate = (byte) rateHz,
                    StartStop = (byte) (startStop ? 1:0),
                }
            };
            return _connection.Send(packet, cancel);
        }

        public Task SetPositionTargetGlobalInt(uint timeBootMs, MavFrame coordinateFrame, int latInt, int lonInt, float alt,
            float vx, float vy, float vz, float afx, float afy, float afz, float yaw,
            float yawRate, PositionTargetTypemask typeMask, CancellationToken cancel)
        {
            var packet = new SetPositionTargetGlobalIntPacket
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    TimeBootMs = timeBootMs,
                    TargetSystem = _config.TargetSystemId,
                    TargetComponent = _config.TargetComponenId,
                    CoordinateFrame = coordinateFrame,
                    LatInt = latInt,
                    LonInt = lonInt,
                    Alt = alt,
                    Vx = vx,
                    Vy = vy,
                    Vz = vz,
                    Afx = afx,
                    Afy = afy,
                    Afz = afz,
                    Yaw = yaw,
                    YawRate = yawRate,
                    TypeMask = typeMask,
                }
            };
            return _connection.Send(packet, cancel);
        }

        public void Dispose()
        {
            
        }
    }
}