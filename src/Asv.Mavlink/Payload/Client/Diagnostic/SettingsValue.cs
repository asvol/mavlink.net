using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

namespace Asv.Mavlink
{
    public class SettingsValue : ISettingsValue
    {
        private readonly Func<KeyValueData, CancellationToken, Task> _sendUpdate;
        private readonly RxValue<string> _value = new RxValue<string>();

        public SettingsValue(string name, Func<KeyValueData, CancellationToken, Task> sendUpdate)
        {
            _sendUpdate = sendUpdate;
            Name = name;
        }

        public string Name { get; }

        public IRxValue<string> Value => _value;

        public void RemoteUpdate(string value)
        {
            _value.OnNext(value);
        }

        public Task Update(string value, CancellationToken cancel)
        {
            return _sendUpdate(new KeyValueData{ Key = Name,Value = value }, cancel);
        }

        public void Dispose()
        {
            _value.Dispose();
        }
    }
}
