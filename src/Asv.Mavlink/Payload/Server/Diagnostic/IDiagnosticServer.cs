using System;
using System.Collections.Generic;

namespace Asv.Mavlink
{
    public interface IDiagnosticValues<in T>
    {
        T this[string name] { set; }
    }


    public interface ISettingsValues
    {
        IObservable<KeyValuePair<string, string>> OnLocalChanged { get; }
        IObservable<KeyValuePair<string,string>> OnRemoteChanged { get; }
        string this[string name] { get; set; }
    }

    public interface IDiagnosticServer
    {
        ISettingsValues Settings { get; }
        IDiagnosticValues<double> Digits { get; }
        IDiagnosticValues<string> Strings { get; }
    }
}