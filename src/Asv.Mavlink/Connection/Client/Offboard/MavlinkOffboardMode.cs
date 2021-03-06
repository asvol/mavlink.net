﻿using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Client;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    
    public class MavlinkOffboardMode : IMavlinkOffboardMode
    {
        private readonly IMavlinkV2Connection _connection;
        private readonly MavlinkClientIdentity _config;

        public MavlinkOffboardMode(IMavlinkV2Connection connection, MavlinkClientIdentity config)
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
                    TargetComponent = _config.TargetComponentId,
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

        public void Dispose()
        {
            
        }
    }
}