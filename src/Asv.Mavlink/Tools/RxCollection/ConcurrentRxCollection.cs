using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reactive.Subjects;

namespace Asv.Avialab.Core
{
    public class ConcurrentRxCollection<TModel> : IRxCollection<TModel>, IDisposable
    {
        private readonly SynchronizedList<TModel> _items = new SynchronizedList<TModel>(new List<TModel>());
        private readonly Subject<TModel> _addSubject = new Subject<TModel>();
        private readonly Subject<TModel> _remSubject = new Subject<TModel>();

        public IEnumerator<TModel> GetEnumerator()
        {
            return _items.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public void Remove(TModel model)
        {
            if (_items.Remove(model))
            {
                _remSubject.OnNext(model);
            }
        }

        public void Clear()
        {
            var itemsToDelete = _items.ToArray();
            foreach (var item in itemsToDelete)
            {
                Remove(item);
            }
        }

        public void Add(TModel model)
        {
            _items.Add(model);
            _addSubject.OnNext(model);
        }

        public IObservable<TModel> OnAdd => _addSubject;
        public IObservable<TModel> OnRemove => _remSubject;

        public void Dispose()
        {
            _addSubject?.OnCompleted();
            _addSubject?.Dispose();

            _remSubject?.OnCompleted();
            _remSubject?.Dispose();
            
        }
    }
}
