using System;
using System.Reactive.Linq;

namespace Asv.Mavlink.Client
{
    public static class FilterVehicleHelper
    {
        public static IObservable<IPacketV2<IPayload>> FilterVehicle(this IMavlinkV2Connection src, MavlinkClientIdentity identity)
        {
            return src.Where(_ => FilterVehicle(_, identity));
        }

        public static bool FilterVehicle(IPacketV2<IPayload> packetV2, MavlinkClientIdentity identity)
        {
            if (identity.TargetSystemId != 0 && identity.TargetSystemId != packetV2.SystemId) return false;
            if (identity.TargetComponentId != 0 && identity.TargetComponentId != packetV2.ComponenId) return false;
            return true;
        }
    }
}
