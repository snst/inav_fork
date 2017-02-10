using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NavControlWin
{
    class Filter4
    {
        System.Windows.Forms.DataVisualization.Charting.Chart chart;
        double ts = 0;

        public Vector<double> x = Vector<double>.Build.DenseOfArray(new double[]  // state estimate
        {     0 // heightAboveGround
            , 0 // vertical speed
//            , 0 // baroGroundHeight
  //          , 0 // gpsGroundHeight
        });

        public Matrix<double> F = null;
        public Matrix<double> B = null;
        public Matrix<double> P = null;
        public Matrix<double> H = getH();
        public Matrix<double> R = null;//getR(10000, 10000);
        public Matrix<double> Q = null;//getQ();
        public Vector<double> u = null;
        public Vector<double> Fx = null;
        public Vector<double> Bu = null;
        //        public Matrix<double> I = Matrix<double>.Build.DenseOfArray(new double[,] { { 1, 0 }, { 0, 1 } });

        public Vector<double> y;
        public Matrix<double> S;
        public Matrix<double> K;
        Form1 form;

        Varianz v = new Varianz(200);


        public static Matrix<double> getF(double dt)
        {
            return Matrix<double>.Build.DenseOfArray(new double[,] {
                  { 1, dt }
                , { 0, 1 }
                //  { 1,dt, 0, 0 }
                //, { 0, 1, 0, 0 }
//                , { 0, 0, 1, 0 }
//                , { 0, 0, 0, 1 }
            });
        }

        public static Matrix<double> getB(double dt)
        {
            return Matrix<double>.Build.DenseOfArray(new double[,] {
                  { dt*dt / 2.0 }
                , { dt }
                //  { dt* dt / 2.0 }
                //, { dt }
  //              , { 0 }
    //            , { 0 }
            });
        }

        public static Matrix<double> getH()
        {
            return Matrix<double>.Build.DenseOfArray(new double[,] {
                { 1, 0 }
                //  { 1, 0, 1, 0 }
                //, { 1, 0, 0, 0 }
                //, { 1, 0, 0, 1 }
            });
        }

        public static Matrix<double> getQ(double dt, double sig)
        {
            return Matrix<double>.Build.DenseOfArray(new double[,] {
                  { dt*dt*dt*dt*sig/4, dt*dt*dt*sig/2 }
                , { dt*dt*dt*sig/2, dt*dt*sig }

                //  { 0.3, 0, 0, 0 }
                //, { 0, 0.5, 0, 0 }
                //, { 0, 0, 0, 0 }
                //, { 0, 0, 0, 0 }
            });
        }

        public static Matrix<double> getP(double sigBaro, double sigAcc)
        {
            return Matrix<double>.Build.DenseOfArray(new double[,] {
                  { sigBaro, 0 }
                , { 0, sigAcc }

                //  { 0.1, 0, 0, 0 }
                //, { 0, 0.1, 0, 0 }
                //, { 0, 0, 10000, 0 }
                //, { 0, 0, 0, 10000 }
            });
        }

        public static Matrix<double> getR(double sigAcc, double sonar, double gps)
        {
            return Matrix<double>.Build.DenseOfArray(new double[,] {
                  { sigAcc }
                //  { 1, 0, 0 }
                //, { 0, sonar, 0 }
                //, { 0, 0, gps }
            });
        }

        public static Vector<double> get_u(double acc)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { acc });
        }

        public static Matrix<double> getMatrix2x2(double a, double b, double c, double d)
        {
            return Matrix<double>.Build.DenseOfArray(new double[,] { { a, b }, { c, d } });
        }

        public static Matrix<double> getMatrix1x2(double a, double b)
        {
            return Matrix<double>.Build.DenseOfArray(new double[,] { { a, b } });
        }

        public static Matrix<double> getMatrix1x1(double a)
        {
            return Matrix<double>.Build.DenseOfArray(new double[,] { { a } });
        }

        public void init(System.Windows.Forms.DataVisualization.Charting.Chart ch, Form1 f)
        {
            chart = ch;
            form = f;
            chart.ChartAreas[0].AxisX.ScaleView.Size = 400;
            chart.Series[0].Name = "BaroAlt";
            chart.Series[1].Name = "EstAlt";
            chart.Series[2].Name = "predictAlt";
            chart.Series[3].Name = "acc";
            chart.Series[4].Name = "BaroMedian";
            chart.Series[5].Name = "Bu";
            chart.Series[6].Name = "BaroSmooth";
            form.NuSigPos().Value = (decimal)(0.3);
            form.NuSigBaro().Value = (decimal)0.1;
            form.NuSigAcc().Value = (decimal)0.1;
        }

        public double lastBaroAlt = 0;

        public double[] baroMedian = new double[5] { 0, 0, 0, 0, 0 };

        public double getMedian(double val)
        {
            for (int i = 0; i < baroMedian.Length-1; i++)
                baroMedian[i] = baroMedian[i + 1];
            baroMedian[baroMedian.Length - 1] = val;

            double[] cA = (double[])baroMedian.Clone();
            Array.Sort(cA);
            double ret = cA[baroMedian.Length / 2];
            return ret;


        }

        public double _s = 0;
        public double _v = 0;

        public void reset()
        {
            _v = _s = 0;
        }

        public void update(double baroAlt, double accZ, double dt)
        {
            ts += dt * 80;

            double accF = accZ * (double)2;

            _v += accF * dt;
            _s += _v * dt;

            double median = getMedian(baroAlt);
            lastBaroAlt += (double)0.1 * (median - lastBaroAlt);

            this.chart.Series[4].Points.AddXY(ts, median);
            this.chart.Series[6].Points.AddXY(ts, lastBaroAlt);
            this.chart.Series[0].Points.AddXY(ts, baroAlt);
            this.chart.Series[3].Points.AddXY(ts, accZ / 20);

                       

            double z = lastBaroAlt;

            u = get_u(accF); // control vector

            F = getF(dt); // state transition model
            B = getB(dt); // control input model

            Fx = F * x;
            Bu = B * u;
            
            // predict
            x = Fx + Bu; // Predicted (a priori) state estimate

            this.chart.Series[2].Points.AddXY(ts, x[0]);

            Q = getQ(dt, (double)form.NuSigPos().Value);
            R = getR(0.3, 10000, 10000);
            P = getP((double)form.NuSigBaro().Value, (double)form.NuSigAcc().Value);

            P = F * P * F.Transpose() + Q; // Predicted (a priori) estimate covariance

            // update
            y = z - (H * x); // Innovation or measurement residual, Fehler altitude

            S = H * P * H.Transpose() + R; // Innovation (or residual) covariance
            K = P * H.Transpose() * S.Inverse(); // Optimal Kalman gain
            x = x + (K * y); // Updated (a posteriori) state estimate
            P = P - (K * H * P); // Updated (a posteriori) estimate covariance

            double estAlt = x[0];
            this.chart.Series[1].Points.AddXY(ts, estAlt);

            //            double estVel = x[1];

            //            this.chart.Series[1].Points.AddXY(ts, estAlt);
            //            this.chart.Series[2].Points.AddXY(ts, estVel);
            //            this.chart.Series[4].Points.AddXY(ts, y[0]);
        }

        public void draw()
        {
            if (chart.ChartAreas[0].AxisX.Maximum > chart.ChartAreas[0].AxisX.ScaleView.Size)
            {
                chart.ChartAreas[0].AxisX.ScaleView.Scroll(chart.ChartAreas[0].AxisX.Maximum);
            }

        }
    }
}
