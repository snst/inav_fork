using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NavControlWin
{
    class Filter2
    {
        public double baroMittel = 0.0f;
        ArrayList baroList = new ArrayList();
        int BAROLEN = 50;
        public double baroMittel2 = 0.0f;
        public double baroSTD = 0.0f;

        public void add(double acc, double baro, double dt)
        {
            if (baroList.Count >= BAROLEN)
                baroList.RemoveAt(0);

            baroList.Add(baro);

            calcBaroStandardAbweichung();

            double gain = 0.01f;

            double diff = Math.Abs(baro - baroMittel);
            if (diff > baroSTD)
                gain += 0.05;
            else
                gain -= 0.001;

            if (gain > 0.3) gain = 0.3;
            else if (gain < 0.002) gain = 0.002;

            baroMittel = baroMittel + gain * (baro - baroMittel);

        }

        void calcBaroStandardAbweichung()
        {
            baroMittel2 = 0.0;
            foreach(double v in baroList)
            {
                baroMittel2 += v;
            }
            baroMittel2 /= baroList.Count;

            baroSTD = 0.0f;
            foreach (double v in baroList)
            {
                double k = v - baroMittel;
                baroSTD += k * k;
            }

            baroSTD /= baroList.Count;
            baroSTD = Math.Sqrt(baroSTD);

        }
    }
}
