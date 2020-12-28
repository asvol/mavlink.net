using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Windows;

namespace Asv.Mavlink
{
    

    public class PiecewiseLinearFunction:IEnumerable<KeyValuePair<double,double>>
    {
        private readonly double[,] _values;


        public PiecewiseLinearFunction(double[,] values)
        {
            _values = values;
        }

        public double this[double value]
        {
            get
            {
                if (_values.Length < 4) throw new Exception("Need 2 or more points");
                var first = true;
                double x2;
                double x3;
                double y2;
                double y3;
                double num;
                var prev = new KeyValuePair<double, double>();

                for (int i = 0; i < _values.Length/_values.Rank; i++)
                {
                    if (value < _values[i,0])
                    {
                        if (first)
                        {
                            x2 = _values[0,0];
                            x3 = _values[1,0];
                            y2 = _values[0,1];
                            y3 = _values[1,1];
                        }
                        else
                        {
                            x2 = prev.Key;
                            x3 = _values[i, 0];
                            y2 = prev.Value;
                            y3 = _values[i, 1];
                        }
                        num = (value - x2) / (x3 - x2);
                        return y2 + (y3 - y2) * num;
                    }

                    prev = new KeyValuePair<double, double>(_values[i, 0], _values[i, 1]);
                    first = false;
                }

                var index = _values.Length / _values.Rank;
                x2 = _values[index - 2,0];
                x3 = _values[index - 2,0];
                y2 = _values[index - 1,1];
                y3 = _values[index - 1,1];
                num = (value - x2) / (x3 - x2);
                return y2 + (y3 - y2) * num;
            }
            
        }


        public IEnumerator<KeyValuePair<double, double>> GetEnumerator()
        {
            return (IEnumerator<KeyValuePair<double, double>>) _values.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        
    }
}
