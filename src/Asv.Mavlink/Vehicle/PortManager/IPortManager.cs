using System;

namespace Asv.Mavlink
{
    public interface IPortManager
    {
        IPortInfo[] Ports { get; }
        void Add(PortSettings settings);
        bool Remove(string portId);
        void Load(PortManagerSettings settings);
        PortManagerSettings Save();
        void Enable(string portId);
        void Disable(string portId);
    }

    public interface IPortInfo
    {
        string Id { get; }
        PortSettings Settings { get; }
        long RxAcc { get; }
        long TxAcc { get; }
        PortType Type { get; set; }
        PortState State { get; set; }
        Exception LastException { get; set; }
    }

    

    public class PortSettings
    {
        public string Title { get; set; }
        public string ConnectionString { get; set; }
        public bool IsEnabled { get; set; }
    }

    public class PortManagerSettings
    {
        public PortSettings[] Ports { get; set; } = new PortSettings[0];
    }
}
