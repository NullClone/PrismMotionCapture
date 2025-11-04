using PMC.Utilities;
using UnityEngine;

namespace PMC
{
    public class KalmanFilter
    {
        // Fields

        private double[] state = new double[6] { 0, 0, 0, 0, 0, 0 };
        private double[,] ResidualCovariance;
        private double[,] ResidualCovarianceInv;
        private double[,] KalmanGain;
        private double[,] EstimateCovariance;
        private double[,] TransitionMatrix;
        private double[,] ProcessNoise;

        private readonly double[,] MeasurementNoise = MatrixUtils.Diagonal(3, 1.0);
        private readonly double[,] MeasurementMatrix = new double[3, 6]
        {
            { 1, 0, 0, 0, 0, 0, },
            { 0, 0, 1, 0, 0, 0, },
            { 0, 0, 0, 0, 1, 0, }
        };


        // Methods

        public void SetParameter(double timeInterval, double noise)
        {
            var numArray = new double[6, 3];
            numArray[0, 0] = timeInterval * timeInterval / 2.0;
            numArray[1, 0] = timeInterval;
            numArray[2, 1] = timeInterval * timeInterval / 2.0;
            numArray[3, 1] = timeInterval;
            numArray[4, 2] = timeInterval * timeInterval / 2.0;
            numArray[5, 2] = timeInterval;

            ProcessNoise = numArray.Multiply(MatrixUtils.Diagonal(numArray.GetLength(1), noise)).Multiply(numArray.Transpose());

            EstimateCovariance = ProcessNoise;

            TransitionMatrix = new double[6, 6]
            {
                { 1, timeInterval, 0, 0, 0, 0 },
                { 0, 1, 0, 0, 0, 0 },
                { 0, 0, 1, timeInterval, 0, 0 },
                { 0, 0, 0, 1, 0, 0 },
                { 0, 0, 0, 0, 1, timeInterval },
                { 0, 0, 0, 0, 0, 1 }
            };
        }

        public void Predict()
        {
            state = TransitionMatrix.Multiply(state);

            EstimateCovariance = MatrixUtils.Add(TransitionMatrix.Multiply(EstimateCovariance).Multiply(TransitionMatrix.Transpose()), ProcessNoise);

            var b = MeasurementMatrix.Transpose();

            ResidualCovariance = MatrixUtils.Add(MeasurementMatrix.Multiply(EstimateCovariance).Multiply(b), MeasurementNoise);
            ResidualCovarianceInv = ResidualCovariance.Inverse();
            KalmanGain = EstimateCovariance.Multiply(b).Multiply(ResidualCovarianceInv);
        }

        public void Correct(Vector3 value)
        {
            var measurement = new double[3] { value.x, value.y, value.z };

            var innovation = MatrixUtils.Subtract(measurement, MeasurementMatrix.Multiply(state));

            state = MatrixUtils.Add(state, KalmanGain.Multiply(innovation));

            EstimateCovariance = MatrixUtils.Identity(state.Length).Subtract(KalmanGain.Multiply(MeasurementMatrix)).Multiply(EstimateCovariance);
        }

        public Vector3 Update(Vector3 value)
        {
            Predict();

            Correct(value);

            return new Vector3((float)state[0], (float)state[2], (float)state[4]);
        }
    }
}