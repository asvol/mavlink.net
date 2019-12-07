using System.Threading;
using System.Threading.Tasks;
using Asv.Avialab.Core;

namespace Asv.Mavlink
{
   

    public interface IDiagnosticClient
    {
        IReadonlyRxCollection<IDiagnosticValue<string>> Strings { get; }
        IReadonlyRxCollection<IDiagnosticValue<double>> Digits { get; }
        IReadonlyRxCollection<ISettingsValue> Settings { get; }
    }

    public interface IDiagnosticValue<out T>
    {
        string Name { get; }
        IRxValue<T> Value { get; }
    }

    public interface ISettingsValue
    {
        string Name { get; }
        IRxValue<string> Value { get; }
        Task Update(string newValue, CancellationToken cancel);
    }

}