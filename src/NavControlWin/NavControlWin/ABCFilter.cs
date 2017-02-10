using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NavControlWin
{
    class ABCFilter
    {
        Varianz varA = new Varianz(1000);
        Varianz varBaro = new Varianz(1000);

        double clamp(double val, double cl)
        {
            if (val > cl)
                return val - cl;
            else if (val < -cl) 
                return val + cl;
            else 
                return 0.0;
        }



        public double _v = 0.0f;
        public double _alt = 0.0f;
        public double gainF = 0.0f;

        public void update(double baroAlt, double accZ, double dt)
        {
            double T = dt / 1000.0f;

        //    gain = 0.03f;
            double fAcc = 0.003f;

            double ca = clamp(accZ, 10);

            _v += ca * T;
            _alt += _v * T;
      //      if(ca==0.0f)
     //       _v *= 0.95f;


       //     a += ca/3.0;
         //   if(ca == 0.0f)
         //   a *= 0.5f;

            double cAcc = fAcc * Math.Abs(accZ);
            gain = cAcc;
            //gain -= 0.05f;
            gain = Math.Min(gain, 0.4f);
            gain = Math.Max(gain, 0.003f);

            gainF = gainF + 0.2f * (gain - gainF);

            alt = alt + gainF * (baroAlt - alt);
            alt3 = alt3 + 0.01 * (baroAlt - alt3);

            double deltaAlt = clamp(baroAlt-alt2, 20);
            alt2 += deltaAlt;

        }

        public void reset()
        {
            a = 0.0f;
            v = 0.0f;
            alt = 0.0f;
        }

        public double gain = 0.003f;
        public double alpha = 0.0f;
        public double beta = 0.0f;
        public double gamma = 0.0f;
        public double alt = 0.0f;
        public double alt2 = 0.0f;
        public double alt3 = 0.0f;
        public double v = 0.0f;
        public double a = 0.0f;


    }
}