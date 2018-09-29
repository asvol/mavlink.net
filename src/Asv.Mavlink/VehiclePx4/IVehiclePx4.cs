namespace Asv.Mavlink
{
    public interface IVehiclePx4:IVehicle
    {
        IRxValue<Px4VehicleMode> Mode { get; }
    }
}
