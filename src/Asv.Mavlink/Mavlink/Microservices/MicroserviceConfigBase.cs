namespace Asv.Mavlink
{
    public class MicroserviceConfigBase
    {
        public byte SystemId { get; set; } = 254;
        public byte ComponentId { get; set; } = 254;
        public byte TargetSystemId { get; set; } = 1;
        public byte TargetComponenId { get; set; } = 1;
    }
}
