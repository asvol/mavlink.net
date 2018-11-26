using System.Reactive.Linq;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.V2.Common;

namespace Asv.Mavlink
{
    public class VehiclePx4 : Vehicle, IVehiclePx4
    {
        private readonly RxValue<Px4VehicleMode> _px4Mode = new RxValue<Px4VehicleMode>();

        public VehiclePx4(VehicleConfig config) : base(config)
        {
            InputPackets
                .Where(_ => _.MessageId == HeartbeatPacket.PacketMessageId)
                .Cast<HeartbeatPacket>()
                .Select(_ => new Px4VehicleMode(_.Payload))
                .Subscribe(_px4Mode);
        }

      
        public async Task<float> WriteXYCruise(float velocity, CancellationToken cancel)
        {
            var p = await this.WriteParam("MPC_XY_CRUISE", velocity, cancel);
            return p.RealValue ?? float.NaN;
        }

       
        public async Task<float> ReadXYCruise(CancellationToken cancel)
        {
            var p = await this.ReadParam("MPC_XY_CRUISE", cancel);
            return p.RealValue ?? float.NaN;
        }

      
        public async Task<float> WriteXYVelMax(float velocity, CancellationToken cancel)
        {
            var p = await this.WriteParam("MPC_XY_VEL_MAX", velocity, cancel);
            return p.RealValue ?? float.NaN;
        }

        public async Task<float> WriteZVelMaxDn(float velocity, CancellationToken cancel)
        {
            var p = await this.WriteParam("MPC_Z_VEL_MAX_DN", velocity, cancel);
            return p.RealValue ?? float.NaN;
        }

        public async Task<float> ReadZVelMaxUp(CancellationToken cancel)
        {
            var p = await this.ReadParam("MPC_Z_VEL_MAX_UP", cancel);
            return p.RealValue ?? float.NaN;
        }

        public async Task<float> WriteMissionTakeOffAltitude(float alt, CancellationToken cancel)
        {
            var p = await this.WriteParam("MIS_TAKEOFF_ALT", alt, cancel);
            return p.RealValue ?? float.NaN;
        }

        public async Task<float> ReadMissionTakeOffAltitude(CancellationToken cancel)
        {
            var p = await this.ReadParam("MIS_TAKEOFF_ALT", cancel);
            return p.RealValue ?? float.NaN;
        }

        public async Task<float> ReadZVelMaxDn(CancellationToken cancel)
        {
            var p = await this.ReadParam("MPC_Z_VEL_MAX_DN", cancel);
            return p.RealValue ?? float.NaN;
        }

        public async Task<float> WriteZVelMaxUp(float velocity, CancellationToken cancel)
        {
            var p = await this.WriteParam("MPC_Z_VEL_MAX_UP", velocity, cancel);
            return p.RealValue ?? float.NaN;
        }
       
        public async Task<float> ReadXYVelMax(CancellationToken cancel)
        {
            var p = await this.ReadParam("MPC_XY_VEL_MAX", cancel);
            return p.RealValue ?? float.NaN;
        }

        public IRxValue<Px4VehicleMode> Mode => _px4Mode;
    }
}