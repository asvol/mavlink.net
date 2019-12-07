using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Threading;
using System.Threading.Tasks;
using Asv.Mavlink.Server;
using Asv.Mavlink.V2.Common;
using NLog;

namespace Asv.Mavlink
{
    public class DiagnosticServerConfig
    {
        public int StringUpdateTimeMs { get; set; } = 2000;
        public int DigitUpdateTimeMs { get; set; } = 1000;
        public int SettingsUpdateTimeMs { get; set; } = 2000;
    }

    public class DiagnosticValues<T> : IDiagnosticValues<T>
    {
        private readonly ConcurrentDictionary<string, T> _values;

        public DiagnosticValues(ConcurrentDictionary<string,T> values)
        {
            _values = values;
        }

        public T this[string name]
        {
            get
            {
                T value;
                return _values.TryGetValue(name, out value) ? value : default(T);
            }
            set
            {
                _values.AddOrUpdate(name, _ => value, (k, v) => value);
            }
        }
    }

   


   
    public class DiagnosticServerInterface:PayloadServerInterfaceBase, IDiagnosticServer
    {
        private static readonly TimeSpan UpdateTime = TimeSpan.FromSeconds(1);
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private readonly ConcurrentDictionary<string,string> _valuesStr = new ConcurrentDictionary<string, string>();
        private readonly ConcurrentDictionary<string, double> _valuesDig = new ConcurrentDictionary<string, double>();
        private readonly ConcurrentDictionary<string, string> _settingsDict = new ConcurrentDictionary<string, string>();

        private readonly DiagnosticValues<string> _valuesStrDict;
        private readonly DiagnosticValues<double> _valuesDigDict;

        private readonly DiagnosticServerConfig _cfg;
        private readonly SettingsValues _settings;

        private volatile int _isUpdateDigitInProgress;
        private volatile int _isUpdateStringsInProgress;
        private volatile int _isUpdateSettingsInProgress;

        public DiagnosticServerInterface(DiagnosticServerConfig config) : base(WellKnownDiag.Diag)
        {
            _cfg = config;
            _settings = new SettingsValues(_settingsDict);
            _disposeCancel.Token.Register(()=> _settings.Dispose());
            _valuesStrDict = new DiagnosticValues<string>(_valuesStr);
            _valuesDigDict = new DiagnosticValues<double>(_valuesDig);
        }

        public override void Init(IMavlinkPayloadServer server)
        {
            base.Init(server);
            if (_cfg.StringUpdateTimeMs > 0)
            {
                Observable.Timer(TimeSpan.FromMilliseconds(_cfg.StringUpdateTimeMs), TimeSpan.FromMilliseconds(_cfg.StringUpdateTimeMs)).Subscribe(_ => UpdateStrings(), _disposeCancel.Token);
            }
            if (_cfg.DigitUpdateTimeMs > 0)
            {
                Observable.Timer(TimeSpan.FromMilliseconds(_cfg.DigitUpdateTimeMs), TimeSpan.FromMilliseconds(_cfg.DigitUpdateTimeMs)).Subscribe(_ => UpdateDigits(), _disposeCancel.Token);
            }
            if (_cfg.SettingsUpdateTimeMs > 0)
            {
                Observable.Timer(TimeSpan.FromMilliseconds(_cfg.SettingsUpdateTimeMs), TimeSpan.FromMilliseconds(_cfg.SettingsUpdateTimeMs)).Subscribe(_ => UpdateSettings(), _disposeCancel.Token);
            }
            Register<KeyValuePair<string,string>,Void>(WellKnownDiag.DiagSettingsSetMethodName, OnValueSet);
        }

        private Task<Void> OnValueSet(DeviceIdentity devid, KeyValuePair<string, string> data)
        {
            _logger.Info($"Settings changed[sys:{devid.SystemId}, com:{devid.ComponentId}]: {data.Key} = {data.Value}");
            Status.Log(MavSeverity.MavSeverityInfo, $"Write {data.Key}={data.Value}");
            return Task.Factory.StartNew(() =>
            {
                _settings.OnRemoteUpdate(data);
                UpdateSettings();
                return new Void();
            });
        }

        private void UpdateSettings()
        {
            if (Interlocked.CompareExchange(ref _isUpdateSettingsInProgress, 1, 0) != 0) return;
            try
            {
                Send(new DeviceIdentity { ComponentId = 0, SystemId = 0 }, WellKnownDiag.DiagSettingsValueName, _settingsDict, CancellationToken.None);
            }
            catch (Exception e)
            {
                _logger.Error(e, $"Error to update settings:{e.Message}");
            }
            finally
            {
                Interlocked.Exchange(ref _isUpdateSettingsInProgress, 0);
            }
        }

        private void UpdateStrings()
        {
            if (Interlocked.CompareExchange(ref _isUpdateStringsInProgress,1,0)!=0) return;
            try
            {
                Send(new DeviceIdentity {ComponentId = 0, SystemId = 0}, WellKnownDiag.DiagStringsValueName, _valuesStr, CancellationToken.None);
            }
            catch (Exception e)
            {
                _logger.Error(e, $"Error to update digit:{e.Message}");
            }
            finally
            {
                Interlocked.Exchange(ref _isUpdateStringsInProgress, 0);
            }
        }

        private void UpdateDigits()
        {
            if (Interlocked.CompareExchange(ref _isUpdateDigitInProgress, 1, 0) != 0) return;
            try
            {
                Send(new DeviceIdentity { ComponentId = 0, SystemId = 0 }, WellKnownDiag.DiagDigitValueName, _valuesDig, CancellationToken.None);
            }
            catch (Exception e)
            {
                _logger.Error(e, $"Error to update digit:{e.Message}");
            }
            finally
            {
                Interlocked.Exchange(ref _isUpdateDigitInProgress, 0);
            }
        }

        protected override void InternalDisposeOnce()
        {
            base.InternalDisposeOnce();
            _disposeCancel?.Cancel(false);
            _disposeCancel?.Dispose();
        }


        public ISettingsValues Settings => _settings;
        public IDiagnosticValues<double> Digits => _valuesDigDict;
        public IDiagnosticValues<string> Strings => _valuesStrDict;
    }


    public class SettingsValues : ISettingsValues,IDisposable
    {
        private readonly ConcurrentDictionary<string, string> _values;
        private readonly Subject<KeyValuePair<string,string>> _localChanges = new Subject<KeyValuePair<string, string>>();
        private readonly Subject<KeyValuePair<string, string>> _remoteChanges = new Subject<KeyValuePair<string, string>>();
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();

        public SettingsValues(ConcurrentDictionary<string, string> values)
        {
            _values = values;
        }

        public IObservable<KeyValuePair<string, string>> OnLocalChanged => _localChanges;
        public IObservable<KeyValuePair<string, string>> OnRemoteChanged => _remoteChanges;

        public void OnRemoteUpdate(KeyValuePair<string, string> value)
        {
            _values.AddOrUpdate(value.Key, _ => value.Value, (k, v) => value.Value);
            try
            {
                _remoteChanges.OnNext(value);
            }
            catch (Exception e)
            {
                _logger.Error(e, $"Error to publish remote changes settings event:{e.Message}");
            }
        }

        public string this[string name]
        {
            get
            {
                string value;
                return _values.TryGetValue(name, out value) ? value : default(string);
            }
            set
            {
                _values.AddOrUpdate(name, _ => value, (k, v) => value);
                try
                {
                    _localChanges.OnNext(new KeyValuePair<string, string>(name, value));
                }
                catch (Exception e)
                {
                    _logger.Error(e,$"Error to publish local changes settings event:{e.Message}");
                }
                
            }
        }

        public void Dispose()
        {
            _localChanges.Dispose();
            _remoteChanges.Dispose();
        }
    }
}