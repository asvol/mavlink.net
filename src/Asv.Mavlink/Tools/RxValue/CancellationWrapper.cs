using System;
using System.Threading;

namespace Asv.Mavlink
{
    public class CancellationWrapper<T> : IDisposable
    {
        private readonly Action<T, CancellationToken> _onConnectedCallback;
        private readonly IDisposable _subscribe;
        private CancellationTokenSource _cancel;

        public CancellationWrapper(IRxValue<T> factory, Action<T, CancellationToken> onConnectedCallback)
        {
            _onConnectedCallback = onConnectedCallback;
            if (factory.Value != null)
            {
                Reconnect(factory.Value);
            }

            _subscribe = factory.Subscribe(Reconnect);
        }

        private void Reconnect(T value)
        {
            Disconnect();
            if (value != null)
            {
                Connect(value);
            }
        }

        private void Connect(T value)
        {
            _cancel = new CancellationTokenSource();
            _onConnectedCallback(value, _cancel.Token);
        }

        private void Disconnect()
        {
            _cancel?.Cancel(false);
            _cancel?.Dispose();
            _cancel = null;
        }

        public void Dispose()
        {
            Disconnect();
            _subscribe?.Dispose();
        }
    }

    public class CancellationWrapper<T1, T2> : IDisposable
    {
        private readonly Action<T1, T2, CancellationToken> _onConnectedCallback;
        private readonly IDisposable _subscribe1;
        private readonly IDisposable _subscribe2;
        private CancellationTokenSource _cancel;

        public CancellationWrapper(IRxValue<T1> factory1, IRxValue<T2> factory2, Action<T1,T2, CancellationToken> onConnectedCallback)
        {
            _onConnectedCallback = onConnectedCallback;
            if (factory1.Value != null && factory2.Value != null)
            {
                Reconnect(factory1.Value, factory2.Value);
            }

            _subscribe1 = factory1.Subscribe(_ => Reconnect(_,factory2.Value));
            _subscribe2 = factory2.Subscribe(_ => Reconnect(factory1.Value, _));
        }

        private void Reconnect(T1 value1, T2 value2)
        {
            Disconnect();
            if (value1 != null && value2 != null)
            {
                Connect(value1, value2);
            }
        }

        private void Connect(T1 value1, T2 value2)
        {
            _cancel = new CancellationTokenSource();
            _onConnectedCallback(value1,value2, _cancel.Token);
        }

        private void Disconnect()
        {
            _cancel?.Cancel(false);
            _cancel?.Dispose();
            _cancel = null;
        }

        public void Dispose()
        {
            Disconnect();
            _subscribe1?.Dispose();
            _subscribe2?.Dispose();
        }
    }
}