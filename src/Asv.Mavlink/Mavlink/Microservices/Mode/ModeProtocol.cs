using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public class ModeProtocol : IMavlinkModeProtocol
    {
        private readonly IMavlinkV2Connection _connection;
        private readonly MicroserviceConfigBase _config;

        public ModeProtocol(IMavlinkV2Connection connection, MicroserviceConfigBase config)
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

        public void Dispose()
        {
            
        }
    }
}