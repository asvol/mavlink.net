using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
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
        public int MaxAgeToUpdateAllMs { get; set; } = 30_000;
    }

    public class ValueWithFlag<T>
    {
        public T Value { get; set; }
        public DateTime LastSyncTime { get; set; } = DateTime.MinValue;
    }

    public class DiagnosticValues<T> : IDiagnosticValues<T>
    {
        private readonly ConcurrentDictionary<string, ValueWithFlag<T>> _values;

        public DiagnosticValues(ConcurrentDictionary<string, ValueWithFlag<T>> values)
        {
            _values = values;
        }

        public T this[string name]
        {
            get
            {
                var item = _values.TryGetValue(name, out var value) ? value : new ValueWithFlag<T>();
                return item.Value;
            }
            set
            {
                _values.AddOrUpdate(name, _ => new ValueWithFlag<T>{Value = value , LastSyncTime = DateTime.MinValue }, (k, v) =>
                {
                    v.Value = value;
                    v.LastSyncTime = DateTime.MinValue;
                    return v;
                });
            }
        }

        public bool IsEmpty => _values.IsEmpty;
    }

   


   
    public class DiagnosticServerInterface:PayloadServerInterfaceBase, IDiagnosticServer
    {
        private static readonly TimeSpan UpdateTime = TimeSpan.FromSeconds(1);
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();
        private readonly ConcurrentDictionary<string, ValueWithFlag<string>> _valuesStr = new ConcurrentDictionary<string, ValueWithFlag<string>>();
        private readonly ConcurrentDictionary<string, ValueWithFlag<double>> _valuesDig = new ConcurrentDictionary<string, ValueWithFlag<double>>();
        private readonly ConcurrentDictionary<string, ValueWithFlag<string>> _settingsDict = new ConcurrentDictionary<string, ValueWithFlag<string>>();

        private readonly DiagnosticValues<string> _valuesStrDict;
        private readonly DiagnosticValues<double> _valuesDigDict;

        private readonly DiagnosticServerConfig _cfg;
        private readonly SettingsValues _settings;

        private volatile int _isUpdateInProgress;
        private TimeSpan _maxAgeToUpdateAll;

        public DiagnosticServerInterface(DiagnosticServerConfig config) : base(WellKnownDiag.Diag)
        {
            _cfg = config;
            _maxAgeToUpdateAll = TimeSpan.FromMilliseconds(_cfg.MaxAgeToUpdateAllMs);

            _settings = new SettingsValues(_settingsDict);
            _valuesStrDict = new DiagnosticValues<string>(_valuesStr);
            _valuesDigDict = new DiagnosticValues<double>(_valuesDig);
            _disposeCancel.Token.Register(() => _settings.Dispose());
        }

        public override void Init(IMavlinkPayloadServer server)
        {
            base.Init(server);
            Observable.Timer(UpdateTime, UpdateTime).Subscribe(CheckUpdates, _disposeCancel.Token);

            Register<KeyValuePair<string, string>, PayloadVoid>(WellKnownDiag.DiagGetAll, OnGetAll);

            Register<KeyValuePair<string,string>,PayloadVoid>(WellKnownDiag.DiagSettingsSetMethodName, OnValueSet);
        }

        private async void CheckUpdates(long obj)
        {
            if (Interlocked.CompareExchange(ref _isUpdateInProgress, 1, 0) != 0) return;
            try
            {
                await SendDictionary(WellKnownDiag.DiagSettingsValueName, _settingsDict);
                await SendDictionary(WellKnownDiag.DiagStringsValueName, _valuesStr);
                await SendDictionary(WellKnownDiag.DiagDigitValueName, _valuesDig);
            }
            catch (Exception e)
            {
                _logger.Error(e, $"Error to update settings:{e.Message}");
            }
            finally
            {
                Interlocked.Exchange(ref _isUpdateInProgress, 0);
            }
        }

        private Task<PayloadVoid> OnGetAll(DeviceIdentity devid, KeyValuePair<string, string> data)
        {
            foreach (var item in _valuesDig)
            {
                item.Value.LastSyncTime = DateTime.MinValue;
            }
            foreach (var item in _valuesStr)
            {
                item.Value.LastSyncTime = DateTime.MinValue;
            }
            foreach (var item in _settingsDict)
            {
                item.Value.LastSyncTime = DateTime.MinValue;
            }

            return Task.FromResult(PayloadVoid.Default);
        }

        private Task<PayloadVoid> OnValueSet(DeviceIdentity devid, KeyValuePair<string, string> data)
        {
            _logger.Info($"Settings changed[sys:{devid.SystemId}, com:{devid.ComponentId}]: {data.Key} = {data.Value}");
            Status.Log(MavSeverity.MavSeverityInfo, $"Write {data.Key}={data.Value}");
            return Task.Factory.StartNew(() =>
            {
                _settings.OnRemoteUpdate(data);
                return new PayloadVoid();
            });
        }

        private async Task SendDictionary<T>(string path, IReadOnlyDictionary<string,ValueWithFlag<T>> values)
        {
            if (values.Count == 0) return;
            var list = new List<KeyValuePair<string, T>>();
            foreach (var item in values.Where(_=>(DateTime.Now - _.Value.LastSyncTime) > _maxAgeToUpdateAll ))
            {
                list.Add(new KeyValuePair<string, T>(item.Key,item.Value.Value));
                item.Value.LastSyncTime = DateTime.Now;
            }

            if (list.Count == 0) return;
            var factor = 1;
            while (true)
            {
                try
                {
                    var temp = new Dictionary<string,T>();
                    var itemsPerOne = list.Count / factor;
                    for (var i = 0; i < list.Count; i++)
                    {
                        temp.Add(list[i].Key,list[i].Value);
                        if (temp.Count >= itemsPerOne)
                        {
                            await Send(new DeviceIdentity { ComponentId = 0, SystemId = 0 }, path, temp, CancellationToken.None);
                            temp.Clear();
                        }
                    }
                    break;
                }
                catch (PayloadOversizeException e)
                {
                    factor++;
                }
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
        private readonly ConcurrentDictionary<string, ValueWithFlag<string>> _values;
        private readonly Subject<KeyValuePair<string,string>> _localChanges = new Subject<KeyValuePair<string, string>>();
        private readonly Subject<KeyValuePair<string, string>> _remoteChanges = new Subject<KeyValuePair<string, string>>();
        private readonly Logger _logger = LogManager.GetCurrentClassLogger();

        public SettingsValues(ConcurrentDictionary<string, ValueWithFlag<string>> values)
        {
            _values = values;
        }

        public IObservable<KeyValuePair<string, string>> OnLocalChanged => _localChanges;
        public IObservable<KeyValuePair<string, string>> OnRemoteChanged => _remoteChanges;
        public bool IsEmpty => _values.IsEmpty;

        public void OnRemoteUpdate(KeyValuePair<string, string> value)
        {
            _values.AddOrUpdate(value.Key, _ => new ValueWithFlag<string>{Value = value.Value}, (k, v) =>
            {
                v.Value = value.Value;
                return v;
            });
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
            get => _values.TryGetValue(name, out var value) ? value.Value : null;
            set
            {
                _values.AddOrUpdate(name, _ => new ValueWithFlag<string>{LastSyncTime = DateTime.MinValue,Value = value}, (k, v) =>
                {
                    v.LastSyncTime = DateTime.MinValue;
                    v.Value = value;
                    return v;
                });
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
