using System;
using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using Nito.AsyncEx;

namespace Asv.Mavlink.Streams
{
    public class JsonStreamBase : IJsonStream, IDisposable, IObservable<JObject>
    {
        private readonly CancellationTokenSource _disposeCancel = new CancellationTokenSource();
        private readonly Subject<Exception> _onErrorSubject = new Subject<Exception>();
        private readonly Subject<JObject> _onData = new Subject<JObject>();
        private readonly ITextStream _textStream;
        private readonly bool _disposeStream;
        private readonly TimeSpan _requestTimeout;

        public JsonStreamBase(ITextStream textStream, bool disposeStream, TimeSpan requestTimeout)
        {
            this._textStream = textStream;
            this._disposeStream = disposeStream;
            this._requestTimeout = requestTimeout;
            this._textStream.OnError.Subscribe((IObserver<Exception>)this._onErrorSubject);
            this._textStream.Select<string, JObject>(new Func<string, JObject>(this.SafeConvert)).Where<JObject>((Func<JObject, bool>)(_ => _ != null)).Subscribe<JObject>((IObserver<JObject>)this._onData, this._disposeCancel.Token);
        }

        private JObject SafeConvert(string s)
        {
            try
            {
                return JsonConvert.DeserializeObject<JObject>(s);
            }
            catch (Exception ex)
            {
                this._onErrorSubject.OnNext(ex);
                return (JObject)null;
            }
        }

        public IObservable<Exception> OnError
        {
            get
            {
                return (IObservable<Exception>)this._onErrorSubject;
            }
        }

        public async Task Send<T>(T data, CancellationToken cancel)
        {
            try
            {
                string str = JsonConvert.SerializeObject((object)(T)data, Formatting.None);
                await this._textStream.Send(str, cancel);
                str = (string)null;
            }
            catch (Exception ex)
            {
                this._onErrorSubject.OnNext(ex);
            }
        }

        public async Task<JObject> RequestText(string request, Func<JObject, bool> responseFilter, CancellationToken cancel)
        {
            JObject jobject;
            using (CancellationTokenSource timeoutCancel = new CancellationTokenSource(this._requestTimeout))
            {
                using (CancellationTokenSource linkedCancel = CancellationTokenSource.CreateLinkedTokenSource(cancel, timeoutCancel.Token))
                {
                    IDisposable subscribe = (IDisposable)null;
                    try
                    {
                        AsyncAutoResetEvent eve = new AsyncAutoResetEvent(false);
                        JObject result = (JObject)null;
                        subscribe = this.FirstAsync<JObject>(responseFilter).Subscribe<JObject>((Action<JObject>)(_ =>
                        {
                            result = _;
                            eve.Set();
                        }));
                        await SendText(request, linkedCancel.Token);
                        await eve.WaitAsync(linkedCancel.Token);
                        jobject = result;
                    }
                    finally
                    {
                        subscribe?.Dispose();
                    }
                }
            }
            return jobject;
        }

        public async Task SendText(string data, CancellationToken cancel)
        {
            try
            {
                await _textStream.Send(data, cancel);
            }
            catch (Exception ex)
            {
                this._onErrorSubject.OnNext(ex);
            }
        }

        public async Task<JObject> Request<TRequest>(TRequest request, Func<JObject, bool> responseFilter, CancellationToken cancel)
        {
            JObject jobject;
            using (CancellationTokenSource timeoutCancel = new CancellationTokenSource(this._requestTimeout))
            {
                using (CancellationTokenSource linkedCancel = CancellationTokenSource.CreateLinkedTokenSource(cancel, timeoutCancel.Token))
                {
                    IDisposable subscribe = (IDisposable)null;
                    try
                    {
                        AsyncAutoResetEvent eve = new AsyncAutoResetEvent(false);
                        JObject result = (JObject)null;
                        subscribe = this.FirstAsync<JObject>(responseFilter).Subscribe<JObject>((Action<JObject>)(_ =>
                        {
                            result = _;
                            eve.Set();
                        }));
                        await this.Send<TRequest>(request, linkedCancel.Token);
                        await eve.WaitAsync(linkedCancel.Token);
                        jobject = result;
                    }
                    finally
                    {
                        subscribe?.Dispose();
                    }
                }
            }
            return jobject;
        }

        public void Dispose()
        {
            this._disposeCancel.Cancel(false);
            this._disposeCancel.Dispose();
            if (!this._disposeStream)
                return;
            this._textStream?.Dispose();
        }

        public IDisposable Subscribe(IObserver<JObject> observer)
        {
            return this._onData.Subscribe(observer);
        }
    }

}
