using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;
using Newtonsoft.Json.Linq;
using NLog;

namespace Asv.Mavlink.Json
{
    
    public class JsonOneFileConfiguration : IConfiguration
    {
        private readonly string _fileName;
        private readonly Dictionary<string, JToken> _values;
        private readonly ReaderWriterLockSlim _rw = new ReaderWriterLockSlim(LockRecursionPolicy.SupportsRecursion);
        private static readonly Logger Logger = LogManager.GetCurrentClassLogger();


        public JsonOneFileConfiguration(string fileName)
        {
            if (string.IsNullOrEmpty(fileName))
                throw new ArgumentException($"File name {fileName} cannot be null or empty.", nameof(fileName));

            var dir = Path.GetDirectoryName(Path.GetFullPath(fileName));
            if (string.IsNullOrWhiteSpace(dir)) throw new InvalidOperationException("Directory path is null");

            if (!Directory.Exists(dir))
            {
                Logger.Warn($"Directory with config file not exist. Try to create it: {dir}");
                Directory.CreateDirectory(dir);
            }
            
            _fileName = fileName;
            if (File.Exists(fileName) == false)
            {
                Logger.Warn($"Config file not exist. Try to create it: {fileName}");
                InternalSaveChanges();
            }
            else
            {
                var text = File.ReadAllText(_fileName);
                try
                {
                    _values = JsonConvert.DeserializeObject<Dictionary<string, JToken>>(text, new StringEnumConverter()) ?? new Dictionary<string, JToken>();
                }
                catch (Exception e)
                {
                    Logger.Error(e,$"Error to load JSON configuration from file. File content: {text}");
                    throw;
                }
            }
        }

        private void InternalSaveChanges()
        {
            try
            {
                var content = JsonConvert.SerializeObject(_values ?? new Dictionary<string, JToken>(), Formatting.Indented, new StringEnumConverter());
                File.WriteAllText(_fileName, content);
            }
            catch (Exception e)
            {
                Logger.Error(e,$"Error to serialize configutation and save it to file ({_fileName}):{e.Message}");
                throw;
            }
        }

        public IEnumerable<string> AvalableParts => GetParts();

        private IEnumerable<string> GetParts()
        {
            try
            {
                _rw.EnterReadLock();
                return _values.Keys.ToArray();
            }
            finally
            {
                _rw.ExitReadLock();
            }
            
        }

        public bool Exist<TPocoType>(string key)
        {
            return _values.ContainsKey(key);
        }

        public TPocoType Get<TPocoType>(string key, TPocoType defaultValue)
        {
            try
            {
                _rw.EnterUpgradeableReadLock();
                JToken value;
                if (_values.TryGetValue(key, out value))
                {
                    var a = value.ToObject<TPocoType>();
                    return a;
                }
                else
                {
                    Set(key,defaultValue);
                    return defaultValue;
                }
            }
            finally
            {
                _rw.ExitUpgradeableReadLock();
            }
        }

        public void Set<TPocoType>(string key, TPocoType value)
        {
            try
            {
                _rw.EnterWriteLock();
                var jValue = JsonConvert.DeserializeObject<JToken>(JsonConvert.SerializeObject(value));
                if (_values.ContainsKey(key))
                {
                    Logger.Trace($"Update config part [{key}]");
                    _values[key] = jValue;
                }
                else
                {
                    Logger.Trace($"Add new config part [{key}]");
                    _values.Add(key,jValue);
                }
                InternalSaveChanges();
            }
            finally
            {
                _rw.ExitWriteLock();
            }
        }

        public void Remove(string key)
        {
            try
            {
                _rw.EnterWriteLock();
                if (_values.ContainsKey(key))
                {
                    Logger.Trace($"Remove config part [{key}]");
                    _values.Remove(key);
                    InternalSaveChanges();
                }
            }
            finally
            {
                _rw.ExitWriteLock();
            }
        }
    }
}
