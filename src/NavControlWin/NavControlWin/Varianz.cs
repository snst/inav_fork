using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NavControlWin
{
    class Varianz
    {
        int count = 0;
        List<double> values = new List<double>();
        double theVar = 0.0;

        public Varianz(int c)
        {
            count = c;
        }

        public void reset()
        {
            values.Clear();
        }

        public void add(double val)
        {
            if(count>0)
            {
                values.Add(val);
                count--;
            }
            else if(count==0)
            {
                theVar = calc();
                count--;
            }
        }

        public double calc()
        {
            double var = 0.0;
            if(values.Count>0)
            {
                double avg = 0;
                foreach (double v in values)
                {
                    avg += v;
                }
                avg /= values.Count;

                foreach (double v in values)
                {
                    var += ((v - avg) * (v - avg));
                }
                var /= values.Count;
            }
            return var;
        }
    }
}
