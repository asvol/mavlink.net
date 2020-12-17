using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Asv.Avialab.Core;

namespace Asv.Mavlink
{
    public class DiagnosticClientInterface:MavlinkPayloadClientInterfaceBase,IDiagnosticClient
    {
        private static TimeSpan DefaultTimeout = TimeSpan.FromSeconds(1);
        private static int DefaultAttemptCount = 3;

        private readonly CancellationTokenSource _cancel = new CancellationTokenSource();
        private readonly RxCollection<IDiagnosticValue<string>> _strings = new RxCollection<IDiagnosticValue<string>>();
        private readonly RxCollection<IDiagnosticValue<double>> _digits = new RxCollection<IDiagnosticValue<double>>();
        private readonly RxCollection<SettingsValue> _settings = new RxCollection<SettingsValue>();

        private readonly Dictionary<string, DiagnosticValue<string>> _stringDict = new Dictionary<string, DiagnosticValue<string>>();
        private readonly Dictionary<string, DiagnosticValue<double>> _digitDict = new Dictionary<string, DiagnosticValue<double>>();
        private readonly Dictionary<string, SettingsValue> _settingsDict = new Dictionary<string, SettingsValue>();

        private volatile int _isDigitBusy;
        private volatile int _isStringBusy;
        private volatile int _isSettingsBusy;
        private readonly object _sync = new object();


        public DiagnosticClientInterface() : base(WellKnownDiag.Diag)
        {
            
        }

        public override void Init(IMavlinkPayloadClient client)
        {
            base.Init(client);
            Register<Dictionary<string, string>>(WellKnownDiag.DiagStringsValueName).Subscribe(OnDataString, _cancel.Token);
            Register<Dictionary<string, double>>(WellKnownDiag.DiagDigitValueName).Subscribe(OnDataDigit, _cancel.Token);
            Register<Dictionary<string, string>>(WellKnownDiag.DiagSettingsValueName).Subscribe(OnDataSettings, _cancel.Token);
        }

        public Task QueryAll(CancellationToken cancel = default)
        {
            lock (_sync)
            {
                _stringDict.Clear();
                _digitDict.Clear();
                _settingsDict.Clear();
                _stringDict.Clear();
                _digitDict.Clear();
                _settingsDict.Clear();
            }
            
            return Send<PayloadVoid, PayloadVoid>(WellKnownDiag.DiagGetAll, PayloadVoid.Default, TimeSpan.FromSeconds(3), 1, cancel, null);
        }

        private void OnDataSettings(Result<Dictionary<string, string>> val)
        {
            if (Interlocked.CompareExchange(ref _isSettingsBusy, 1, 0) != 0) return;
            try
            {
                if (val.IsError) return;
                foreach (var item in val.Value)
                {
                    SettingsValue vm;
                    if (_settingsDict.TryGetValue(item.Key, out vm))
                    {
                        vm.RemoteUpdate(item.Value);
                    }
                    else
                    {
                        var newVal = new SettingsValue(item.Key, SendUpdate);
                        newVal.RemoteUpdate(item.Value);
                        _settingsDict.Add(newVal.Name, newVal);
                        _settings.Add(newVal);
                    }
                }

            }
            finally
            {
                Interlocked.Exchange(ref _isSettingsBusy, 0);
            }
        }

        private Task SendUpdate(KeyValueData value, CancellationToken cancel)
        {
            return Send<KeyValueData, PayloadVoid>(WellKnownDiag.DiagSettingsSetMethodName, value, DefaultTimeout, DefaultAttemptCount,  cancel, null);
        }

        private void OnDataDigit(Result<Dictionary<string, double>> val)
        {
            if (Interlocked.CompareExchange(ref _isDigitBusy, 1, 0) != 0) return;
            try
            {
                if (val.IsError) return;
                foreach (var item in val.Value)
                {
                    DiagnosticValue<double> vm;
                    if (_digitDict.TryGetValue(item.Key, out vm))
                    {
                        vm.RemoteUpdate(item.Value);
                    }
                    else
                    {
                        var newVal = new DiagnosticValue<double>(item.Key);
                        newVal.RemoteUpdate(item.Value);
                        _digitDict.Add(newVal.Name, newVal);
                        _digits.Add(newVal);
                    }
                }

            }
            finally
            {
                Interlocked.Exchange(ref _isDigitBusy, 0);
            }

        }

        private void OnDataString(Result<Dictionary<string, string>> val)
        {
            if (Interlocked.CompareExchange(ref _isStringBusy, 1,0) !=0 ) return;
            try
            {
                if (val.IsError) return;
                foreach (var item in val.Value)
                {
                    DiagnosticValue<string> vm;
                    if (_stringDict.TryGetValue(item.Key, out vm))
                    {
                        vm.RemoteUpdate(item.Value);
                    }
                    else
                    {
                        var newVal = new DiagnosticValue<string>(item.Key);
                        newVal.RemoteUpdate(item.Value);
                        _stringDict.Add(newVal.Name, newVal);
                        _strings.Add(newVal);
                    }
                }
                
            }
            finally
            {
                Interlocked.Exchange(ref _isStringBusy, 0);
            }
        }

        protected override void InternalDisposeOnce()
        {
            base.InternalDisposeOnce();

            foreach (var item in _stringDict.Values)
            {
                _strings.Remove(item);
                item.Dispose();
            }
            _stringDict.Clear();

            foreach (var item in _digitDict.Values)
            {
                _digits.Remove(item);
                item.Dispose();
            }
            _digitDict.Clear();

            foreach (var item in _settingsDict.Values)
            {
                _settings.Remove(item);
                item.Dispose();
            }
            _settingsDict.Clear();
        }

        public IReadonlyRxCollection<IDiagnosticValue<string>> Strings => _strings;
        public IReadonlyRxCollection<IDiagnosticValue<double>> Digits => _digits;
        public IReadonlyRxCollection<ISettingsValue> Settings => _settings;
    }

   
}
