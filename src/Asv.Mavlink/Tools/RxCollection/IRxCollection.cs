namespace Asv.Avialab.Core
{
    public interface IRxCollection<TModel> : IReadonlyRxCollection<TModel>
    {
        void Clear();
        void Add(TModel model);
        void Remove(TModel model);
    }
}
