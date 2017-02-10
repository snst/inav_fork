using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;

namespace NavControlWin
{
    class LogItem
    {
        public double ts;
        public double a;
        public double alt;
        public double a2;
        public int an;
        public LogItem(double _a, double _alt, double _ts, double _a2, int _an)
        {
            a = _a;
            alt = _alt;
            ts = _ts;
            a2 = _a2;
            an = _an;

        }
    }

    class Logger
    {
        DateTime start;
        List<LogItem> list = new List<LogItem>();
        public void add(double a, double alt, double a2, int an)
        {
            if (list.Count == 0)
                start = DateTime.Now;

            list.Add(new LogItem(a, alt, (DateTime.Now- start).TotalMilliseconds, a2, an ));
        }

        public void dump()
        {
            int last = 0;
            int n = 0;
            foreach( LogItem l in list)
            {
                int now = (int)l.ts;
                Debug.WriteLine(String.Format("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}", n, (int)l.ts, now-last, (int)l.alt, l.a, l.a2, l.an));
                last = now;
                n++;
            }
        }

        public void clear()
        {
            list.Clear();
        }
    }
}
