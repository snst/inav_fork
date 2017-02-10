using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NavControlWin
{
    public class EKF
    {
        public float estAlt = 0.0f;
        public float estVel = 0.0f;
        public float q = 0.03f; // process noise
        public float p = 0.0f; // estimated error
        public float r = 4.0f; // sensor noise
        public float g = 0.0f;
        public float controlFactor = 1.0f;
        public float f = 0;
        bool fInit = false;

        public float update(float baroAlt, float accZ, float dt)
        {
            if(!fInit)
            {
                estAlt = baroAlt;
                fInit = true;
            }

            // float newAlt = estAlt + (accZ * accZ * dt / 2);

            //a = newAlt / estAlt;

            // Predict
            f = controlFactor * (accZ * dt * dt) / 2000.0f;
            estAlt = estAlt + f;
            p = p + q;

            // Update
            g = p / (p + r); // g==0 => est, g==1 => baro
            estAlt = estAlt + g * (baroAlt - estAlt);
            p = (1 - g) * p;

            return estAlt;
        }

        public void reset()
        {
            fInit = false;
            p = 50.0f; 
            g = 0.0f; // kalmann gain
            estAlt = 0.0f;
            estVel = 0.0f;
            f = 0.0f;
        }
    }


}
