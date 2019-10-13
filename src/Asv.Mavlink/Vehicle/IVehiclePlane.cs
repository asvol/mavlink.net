using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    public interface IVehiclePlane : IVehicle
    {
        IRxValue<double?> PlaneCircleRadius { get; }
    }
}