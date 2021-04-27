using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
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
        IObservable<KeyValueData> OnLocalChanged { get; }
        IObservable<KeyValueData> OnRemoteChanged { get; }
        bool IsEmpty { get; }
        string this[string name] { get; set; }
    }

    public static class SettingsValueHelper
    {
        public static IDisposable Set(this ISettingsValues src,string key,  string value, Action<string> onRemoteUpdateCallback)
        {
            src[key] = value;
            return src.OnRemoteChanged.Where(_=>string.Equals(_.Key, key, StringComparison.CurrentCultureIgnoreCase)).Select(_=>_.Value).Subscribe(onRemoteUpdateCallback);
        }

        public static IDisposable Set(this ISettingsValues src, string key, double value, Action<double?> onRemoteSuccessUpdateCallback, string formatString = "F1")
        {
            src[key] = value.ToString(formatString);
            return src.OnRemoteChanged.Where(_ => string.Equals(_.Key, key, StringComparison.CurrentCultureIgnoreCase)).Select(_ => _.Value).Select(SafeConvertDouble).Subscribe(onRemoteSuccessUpdateCallback);
        }

        public static void Set(this ISettingsValues src, string key, double value, Action<double?> onRemoteSuccessUpdateCallback, CancellationToken cancel, string formatString = "F1")
        {
            src[key] = value.ToString(formatString);
            src.OnRemoteChanged.Where(_ => string.Equals(_.Key, key, StringComparison.CurrentCultureIgnoreCase)).Select(_ => _.Value).Select(SafeConvertDouble).Subscribe(onRemoteSuccessUpdateCallback, cancel);
        }

        public static void Set(this ISettingsValues src, string key, int value, Action<int?> onRemoteSuccessUpdateCallback, CancellationToken cancel, string formatString = "F1")
        {
            src[key] = value.ToString(formatString);
            src.OnRemoteChanged.Where(_ => string.Equals(_.Key, key, StringComparison.CurrentCultureIgnoreCase)).Select(_ => _.Value).Select(SafeConvertInt).Subscribe(onRemoteSuccessUpdateCallback, cancel);
        }


        private static double? SafeConvertDouble(string arg)
        {
            arg = arg.Replace(",", ".");
            return double.TryParse(arg, NumberStyles.Any, CultureInfo.InvariantCulture,out var result) ? (double?) result : null;
        }

        private static int? SafeConvertInt(string arg)
        {
            return int.TryParse(arg, NumberStyles.Any, CultureInfo.InvariantCulture, out var result) ? (int?)result : null;
        }

        public static void Set(this ISettingsValues src, string key, string value, Action<string> onRemoteUpdateCallback, CancellationToken cancel)
        {
            src[key] = value;
            src.OnRemoteChanged.Where(_ => string.Equals(_.Key, key, StringComparison.CurrentCultureIgnoreCase)).Select(_ => _.Value).Subscribe(onRemoteUpdateCallback, cancel);
        }

        public static void Set<TEnum>(this ISettingsValues src, string key, TEnum value, Action<TEnum> onRemoteUpdateCallback, CancellationToken cancel)
            where TEnum:Enum
        {
            src[key] = value.ToString("G");
            src.OnRemoteChanged.Where(_ => string.Equals(_.Key, key, StringComparison.CurrentCultureIgnoreCase)).Select(_ => _.Value).Where(CheckEnum<TEnum>).Select(ConvertEnum<TEnum>).Subscribe(onRemoteUpdateCallback, cancel);
        }

        private static bool CheckEnum<TEnum>(string s)
            where TEnum : Enum
        {
            var val = Enum.GetNames(typeof(TEnum)).FirstOrDefault(_ => _.EndsWith(s));
            return val != null;
        }

        private static TEnum ConvertEnum<TEnum>(string s)
            where TEnum : Enum
        {
            return (TEnum) Enum.Parse(typeof(TEnum),Enum.GetNames(typeof(TEnum)).First(_ => _.EndsWith(s)));
        }
    }

    public interface IDiagnosticServer
    {
        ISettingsValues Settings { get; }
        IDiagnosticValues<double> Digits { get; }
        IDiagnosticValues<string> Strings { get; }
    }
}
