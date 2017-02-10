using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NavControlWin
{
    class Filter3
    {
        System.Windows.Forms.DataVisualization.Charting.Chart chart;
        double ts = 0;

        public Vector<double> x = Vector<double>.Build.DenseOfArray(new double[] { 0, 0 });
        public Matrix<double> F = Matrix<double>.Build.DenseOfArray(new double[,] { { 1, 1 }, { 0, 1 } });
        public Matrix<double> B;
        public Matrix<double> P = getMatrix2x2( 1000, 0, 0, 1000);
        public Matrix<double> H = getMatrix1x2( 1, 0 );
        public Matrix<double> R = getMatrix1x1( 15 ); // measurement var
        public Matrix<double> Q = Matrix<double>.Build.DenseOfArray(new double[,] { { 0,0 }, { 0, 5 } }); // process var
        public Vector<double> u;
        public Matrix<double> I = Matrix<double>.Build.DenseOfArray(new double[,] { { 1, 0 }, { 0, 1 } });

        public Vector<double> y;
        public Matrix<double> S;
        public Matrix<double> K;
        Form1 form;

        Varianz v = new Varianz(200);

        public static Matrix<double> getMatrix2x2(double a, double b, double c, double d)
        {
           return  Matrix<double>.Build.DenseOfArray(new double[,] { { a, b }, { c, d } });
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
            chart.Series[2].Name = "EstVel";
            chart.Series[3].Name = "acc";
            chart.Series[4].Name = "err";
            //form.NuVar().Value = (decimal)(100);
            form.NuBaroSmooth().Value = (decimal)1.5;
            //form.NuAccFactor().Value = (decimal)1;
        }

        public void updateParam(double dt)
        {
            ts += dt * 50;
//            R = Matrix<double>.Build.DenseOfArray(new double[,] { { (double)form.NuVar().Value } });
            F = Matrix<double>.Build.DenseOfArray(new double[,] { { 1, dt }, { 0, 1 } });
            Q = Matrix<double>.Build.DenseOfArray(new double[,] { { 0, 0 }, { 0, (double)form.NuBaroSmooth().Value } }); // process var
            B = Matrix<double>.Build.DenseOfArray(new double[,] { { dt*dt/2.0 }, { dt } });
        }

        public void update(double baroAlt, double accZ, double dt)
        {
            updateParam(dt);

            double z = baroAlt;

            double accF = accZ * (double)form.NuAccFactor().Value;
            //v.add(accF);
          //  v.add(z);

            u = Vector<double>.Build.DenseOfArray(new double[] { accF });

            x = F * x;
            x = x + B * u;

            P = F * P * F.Transpose() + Q;

            y = z - (H * x);
            S = H * P * H.Transpose() + R;
            K = P * H.Transpose() * S.Inverse();
            x = x + (K * y);
            P = P - (K * H * P);

            double estAlt = x[0];
            double estVel = x[1];

            this.chart.Series[0].Points.AddXY(ts, baroAlt);
            this.chart.Series[1].Points.AddXY(ts, estAlt);
            this.chart.Series[2].Points.AddXY(ts, estVel);
            this.chart.Series[3].Points.AddXY(ts, accZ/20);
            this.chart.Series[4].Points.AddXY(ts, y[0]);
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
