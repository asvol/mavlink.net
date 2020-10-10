namespace Asv.Mavlink
{
    public class MathEx
    {

        /// <summary>
        ///   Interpolates data using a piece-wise linear function.
        /// </summary>
        /// <param name="value">The value to be calculated.</param>
        /// <param name="x">The input data points <c>x</c>. Those values need to be sorted.</param>
        /// <param name="y">The output data points <c>y</c>.</param>
        /// <param name="lower">
        /// The value to be returned for values before the first point in <paramref name="x" />.</param>
        /// <param name="upper">
        /// The value to be returned for values after the last point in <paramref name="x" />.</param>
        /// <returns>Computes the output for f(value) by using a piecewise linear
        /// interpolation of the data points <paramref name="x" /> and <paramref name="y" />.</returns>
        public static double Interpolate1D(
            double value,
            double[] x,
            double[] y,
            double lower,
            double upper)
        {
            for (int index1 = 0; index1 < x.Length; ++index1)
            {
                if (value < x[index1])
                {
                    if (index1 == 0)
                        return lower;
                    int index2 = index1 - 1;
                    int index3 = index1;
                    double num = (value - x[index2]) / (x[index3] - x[index2]);
                    return y[index2] + (y[index3] - y[index2]) * num;
                }
            }
            return upper;
        }
    }
}
