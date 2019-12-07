namespace Asv.Avialab.Core
{
    public interface IRxCollection<TModel> : IReadonlyRxCollection<TModel>
    {
        void Add(TModel model);
        void Remove(TModel model);
    }
}