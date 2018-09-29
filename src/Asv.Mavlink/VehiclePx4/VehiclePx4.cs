using System.Reactive.Linq;
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

        public IRxValue<Px4VehicleMode> Mode => _px4Mode;
    }
}