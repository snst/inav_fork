using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;


namespace NavControlWin
{
    class EKF2
    {
        public float update(double baroAlt, double accZ, double dt)
        {
            dt /= 1000.0;

            z[0] = baroAlt;
            z[1] = v;

            u[1] = accZ;

            Q = Matrix<double>.Build.DenseIdentity(2) * (PNStd * PNStd);
            R = Matrix<double>.Build.DenseIdentity(2) * (MNStd * MNStd);


            A = DenseMatrix.OfArray(new double[,] {
                { 1, dt },
                { 0, 1 }});

            B = DenseMatrix.OfArray(new double[,] {
                { 1, 0 },
                { 0, dt }});


            x = (A * x) + (B * u);
            P = A * P * A.Transpose() + Q;
            K = (P * H.Transpose() * (H * P * H.Transpose() + R).Inverse());
            x = x + K * (z - (H * x));
            P = P - (K * H * P.Transpose());

            alt = x[0];
            v = x[1];

            return (float)alt;
        }

        public void reset()
        {
            x = DenseVector.OfArray(new double[] { 0.0, 0.0 });
            H = Matrix<double>.Build.DenseIdentity(2);
            P = Matrix<double>.Build.DenseIdentity(2) * (MNStd * MNStd);
            z = DenseVector.OfArray(new double[] { 0.0, 0.0 });
            u = DenseVector.OfArray(new double[] { 0.0, 1.0 });
            alt = 0.0;
            v = 0.0;
        }

        public double PNStd = 0.02;  // Process noise variance
        public double MNStd = 0.4;  // Measurement noise variance

        Matrix<double> A;
        Matrix<double> B;
        Matrix<double> H; // measurement function to return the state
        Matrix<double> R; // measurement error
        Matrix<double> P;
        Matrix<double> Q; // Process noise covariance matrix
        Matrix<double> K;
        Vector<double> z;
        Vector<double> x;
        Vector<double> u;
        public double v;
        public double alt;

    }
}