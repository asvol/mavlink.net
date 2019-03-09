using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public class OffboardModeConfig:MicroserviceConfigBase
    {
        
    }

    public class OffboardMode : IOffboardMode
    {
        private readonly IMavlinkV2Connection _connection;
        private readonly OffboardModeConfig _config;

        public OffboardMode(IMavlinkV2Connection connection, OffboardModeConfig config)
        {
            _connection = connection;
            _config = config;
        }

        public async Task SetPositionTargetLocalNed(uint timeBootMs, MavFrame coordinateFrame, PositionTargetTypemask typeMask, float x,
            float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yawRate,
            CancellationToken cancel)
        {
            var packet = new SetPositionTargetLocalNedPacket
            {
                ComponenId = _config.ComponentId,
                SystemId = _config.SystemId,
                Payload =
                {
                    TimeBootMs = timeBootMs,
                    TargetComponent = _config.TargetComponenId,
                    TargetSystem = _config.TargetSystemId,
                    CoordinateFrame = coordinateFrame,
                    TypeMask = typeMask,
                    X = x,
                    Y=y,
                    Z=z,
                    Vx = vx,
                    Vy = vy,
                    Vz=vz,
                    Afx = afx,
                    Afy=afy,
                    Afz=afz,
                    Yaw=yaw,
                    YawRate = yawRate,

                }
            };
            await _connection.Send(packet, cancel).ConfigureAwait(false);
        }
    }
}