using System;

namespace Asv.Mavlink
{
    public class DiagnosticValue<T> : IDiagnosticValue<T>, IDisposable
    {
        private readonly RxValue<T> _value = new RxValue<T>();

        public DiagnosticValue(string name)
        {
            Name = name;
        }

        public string Name { get; }
        public IRxValue<T> Value => _value;

        public void RemoteUpdate(T value)
        {
            _value.OnNext(value);
        }

        public void Dispose()
        {
            _value.Dispose();
        }
    }
}