using System;

namespace PMC.Utilities
{
    public static class MatrixUtils
    {
        public static void Multiply(this double[,] a, double[,] b, double[,] result)
        {
            int length1 = a.GetLength(1);
            int length2 = result.GetLength(0);
            int length3 = result.GetLength(1);

            var numArray = new double[length1];

            for (int index1 = 0; index1 < length3; ++index1)
            {
                for (int index2 = 0; index2 < numArray.Length; ++index2)
                {
                    numArray[index2] = b[index2, index1];
                }

                for (int index3 = 0; index3 < length2; ++index3)
                {
                    double num = 0.0;

                    for (int index4 = 0; index4 < numArray.Length; ++index4)
                    {
                        num += a[index3, index4] * numArray[index4];
                    }

                    result[index3, index1] = num;
                }
            }
        }

        public static double[] Multiply(this double[,] matrix, double[] columnVector)
        {
            int length = matrix.GetLength(0);

            if (matrix.GetLength(1) != columnVector.Length)
            {
                throw new Exception("columnVector Vector must have the same length as columns in the matrix.");
            }

            var numArray = new double[length];

            for (int index1 = 0; index1 < length; ++index1)
            {
                for (int index2 = 0; index2 < columnVector.Length; ++index2)
                {
                    numArray[index1] += matrix[index1, index2] * columnVector[index2];
                }
            }

            return numArray;
        }

        public static double[,] Multiply(this double[,] a, double[,] b)
        {
            var result = new double[a.GetLength(0), b.GetLength(1)];

            a.Multiply(b, result);

            return result;
        }


        public static double[,] Add(this double[,] a, double[,] b)
        {
            int length1 = a.GetLength(0) == b.GetLength(0) && a.GetLength(1) == b.GetLength(1) ? a.GetLength(0) : throw new ArgumentException("Matrix dimensions must match", nameof(b));
            int length2 = a.GetLength(1);

            var numArray = new double[length1, length2];

            for (int index1 = 0; index1 < length2; ++index1)
            {
                for (int index2 = 0; index2 < length1; ++index2)
                {
                    numArray[index1, index2] = a[index1, index2] + b[index1, index2];
                }
            }

            return numArray;
        }

        public static double[] Add(this double[] a, double[] b)
        {
            if (a == null)
            {
                throw new ArgumentNullException(nameof(a));
            }

            if (b == null)
            {
                throw new ArgumentNullException(nameof(b));
            }

            if (a.Length != b.Length)
            {
                throw new ArgumentException("Vector lengths must match", nameof(b));
            }

            var numArray = new double[a.Length];

            for (int index = 0; index < a.Length; ++index)
            {
                numArray[index] = a[index] + b[index];
            }

            return numArray;
        }


        public static double[,] Subtract(this double[,] a, double[,] b, bool inPlace = false)
        {
            if (a == null)
            {
                throw new ArgumentNullException(nameof(a));
            }

            if (b == null)
            {
                throw new ArgumentNullException(nameof(b));
            }

            int length1 = a.GetLength(0) == b.GetLength(0) && a.GetLength(1) == b.GetLength(1) ? a.GetLength(0) : throw new ArgumentException("Matrix dimensions must match", nameof(b));
            int length2 = b.GetLength(1);

            var numArray = inPlace ? a : new double[length1, length2];

            for (int index1 = 0; index1 < length1; ++index1)
            {
                for (int index2 = 0; index2 < length2; ++index2)
                {
                    numArray[index1, index2] = a[index1, index2] - b[index1, index2];
                }
            }

            return numArray;
        }

        public static double[] Subtract(this double[] a, double[] b, bool inPlace = false)
        {
            if (a.Length != b.Length)
            {
                throw new ArgumentException("Vector length must match", nameof(b));
            }

            var numArray = inPlace ? a : new double[a.Length];

            for (int index = 0; index < a.Length; ++index)
            {
                numArray[index] = a[index] - b[index];
            }

            return numArray;
        }


        public static T[,] Diagonal<T>(int size, T value)
        {
            var objArray = new T[size, size];

            for (int index = 0; index < size; ++index)
            {
                objArray[index, index] = value;
            }

            return objArray;
        }


        public static T[,] Transpose<T>(this T[,] matrix) => matrix.Transpose(false);

        public static T[,] Transpose<T>(this T[,] matrix, bool inPlace)
        {
            int length1 = matrix.GetLength(0);
            int length2 = matrix.GetLength(1);

            if (inPlace)
            {
                if (length1 != length2)
                {
                    throw new ArgumentException("Only square matrices can be transposed in place.", nameof(matrix));
                }

                for (int index1 = 0; index1 < length1; ++index1)
                {
                    for (int index2 = index1; index2 < length2; ++index2)
                    {
                        matrix[index2, index1] = matrix[index1, index2];
                        matrix[index1, index2] = matrix[index2, index1];
                    }
                }

                return matrix;
            }

            var objArray = new T[length2, length1];

            for (int index3 = 0; index3 < length1; ++index3)
            {
                for (int index4 = 0; index4 < length2; ++index4)
                {
                    objArray[index4, index3] = matrix[index3, index4];
                }
            }

            return objArray;
        }


        public static double[,] Inverse(this double[,] matrix) => matrix.Inverse(false);

        public static double[,] Inverse(this double[,] matrix, bool inPlace)
        {
            int length1 = matrix.GetLength(0);
            int length2 = matrix.GetLength(1);

            if (length1 != length2)
            {
                throw new ArgumentException("Matrix must be square", nameof(matrix));
            }

            if (length1 == 3)
            {
                double num1 = matrix[0, 0];
                double num2 = matrix[0, 1];
                double num3 = matrix[0, 2];
                double num4 = matrix[1, 0];
                double num5 = matrix[1, 1];
                double num6 = matrix[1, 2];
                double num7 = matrix[2, 0];
                double num8 = matrix[2, 1];
                double num9 = matrix[2, 2];
                double num10 = (num1 * ((num5 * num9) - (num6 * num8))) - (num2 * ((num4 * num9) - (num6 * num7))) + (num3 * ((num4 * num8) - (num5 * num7)));

                if (num10 == 0.0)
                {
                    throw new Exception();
                }

                double num11 = 1.0 / num10;

                var numArray = inPlace ? matrix : new double[3, 3];

                numArray[0, 0] = num11 * ((num5 * num9) - (num6 * num8));
                numArray[0, 1] = num11 * ((num3 * num8) - (num2 * num9));
                numArray[0, 2] = num11 * ((num2 * num6) - (num3 * num5));
                numArray[1, 0] = num11 * ((num6 * num7) - (num4 * num9));
                numArray[1, 1] = num11 * ((num1 * num9) - (num3 * num7));
                numArray[1, 2] = num11 * ((num3 * num4) - (num1 * num6));
                numArray[2, 0] = num11 * ((num4 * num8) - (num5 * num7));
                numArray[2, 1] = num11 * ((num2 * num7) - (num1 * num8));
                numArray[2, 2] = num11 * ((num1 * num5) - (num2 * num4));

                return numArray;
            }

            if (length1 != 2)
            {
                throw new ArgumentException("Matrix not Support size", nameof(matrix));
            }

            double num12 = matrix[0, 0];
            double num13 = matrix[0, 1];
            double num14 = matrix[1, 0];
            double num15 = matrix[1, 1];
            double num16 = (num12 * num15) - (num13 * num14);

            if (num16 == 0.0)
            {
                throw new Exception();
            }

            double num17 = 1.0 / num16;
            var numArray1 = inPlace ? matrix : new double[2, 2];

            numArray1[0, 0] = num17 * num15;
            numArray1[0, 1] = -num17 * num13;
            numArray1[1, 0] = -num17 * num14;
            numArray1[1, 1] = num17 * num12;

            return numArray1;
        }


        public static double[,] Identity(int size)
        {
            var numArray = new double[size, size];

            for (int index = 0; index < size; ++index)
            {
                numArray[index, index] = 1.0;
            }

            return numArray;
        }
    }
}