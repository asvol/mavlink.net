using System;
using System.Collections.Generic;
using System.Reactive.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Threading;

namespace Asv.Mavlink
{
    public interface IDiagnosticValues<in T>
    {
        T this[string name] { set; }
        bool IsEmpty { get;}
    }


    public interface ISettingsValues
    {
        IObservable<KeyValuePair<string, string>> OnLocalChanged { get; }
        IObservable<KeyValuePair<string,string>> OnRemoteChanged { get; }
        bool IsEmpty { get; }
        string this[string name] { get; set; }
    }

    public static class SettingsValueHelper
    {
        public static IDisposable Set(this ISettingsValues src,string key,  string value, Action<string> onRemoteUpdateCallback)
        {
            src[key] = value;
            return src.OnRemoteChanged.Where(_=>string.Equals(_.Key,value, StringComparison.CurrentCultureIgnoreCase)).Select(_=>_.Value).Subscribe(onRemoteUpdateCallback);
        }
        public static void Set(this ISettingsValues src, string key, string value, Action<string> onRemoteUpdateCallback, CancellationToken cancel)
        {
            src[key] = value;
            src.OnRemoteChanged.Where(_ => string.Equals(_.Key, key, StringComparison.CurrentCultureIgnoreCase)).Select(_ => _.Value).Subscribe(onRemoteUpdateCallback, cancel);
        }
    }

    public interface IDiagnosticServer
    {
        ISettingsValues Settings { get; }
        IDiagnosticValues<double> Digits { get; }
        IDiagnosticValues<string> Strings { get; }
    }
}
