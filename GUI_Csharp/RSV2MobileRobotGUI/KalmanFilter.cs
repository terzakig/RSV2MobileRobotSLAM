using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class KalmanFilter
    {
        // state vector dimension
        public int Dim;
        // a priori state estimates
        public double[][] X_est;
        // a posteriori state estimates
        public double[][] Xest;
        // measurement vector at step-k
        public double[][] Z;
        // step
        public int step;

        // **************** State Space Model matrices ***********************
        // state transition matrix A (for Xk+1 = A*Xk + B*Uk + Wk + W0)
        public double[][] A;
        // input matrix
        public double[][] B;
        // constant white noise in the process
        public double[][] W0;
        
        // ************ measurement relation matrices ********************
        // state-to-measurement relation matrix: Zk = H*Xk + Vk + V0

        public double[][] H;
        // constant noise in the measurement V0
        public double[][] V0;

        // The Kalman Gain (Blending Factor) at step-k
        public double[][] K;

        // The a priori error covariance estimate
        public double[][] P_est;

        // The a posteriori error covariance estimate
        public double[][] Pest;

        // Wk process noise - Measurement Noise Vk covariance matrices Q and R
        public double[][] Q, R;

        
        // constructor
        public KalmanFilter(int dim, double[][] Amatrix, double[] Bmatrix, double[] W0matrix,
                            double[][] Hmatrix, double[] V0matrix, double[][] Qmatrix, double[][] Rmatrix,
                            double[] Z0)
        {
            int i, j;

            Dim = dim;

            // creating A
            A = new double[Dim][];
            for (i = 0; i < Dim; i++)
            {
                A[i] = new double[Dim];
                for (j = 0; j < Dim; j++)
                    A[i][j] = Amatrix[i][j];
            }
            // creating B
            B = new double[Dim][];
            for (i = 0; i < Dim; i++)
            {
                B[i] = new double[1];
                B[i][0] = Bmatrix[i];
            }

            // creating constant process noise matrix
            W0 = new double[Dim][];
            for (i = 0; i < Dim; i++)
            {
                W0[i] = new double[1];
                W0[i][0] = W0matrix[i];
            }

            // creating measurement relation matrix
            H = new double[Dim][];
            for (i = 0; i < Dim; i++)
            {
                H[i] = new double[Dim];
                for (j = 0; j < Dim; j++)
                    H[i][j] = Hmatrix[i][j];
            }
            // creating measurement constant noise matrix
            V0 = new double[Dim][];
            for (i = 0; i < Dim; i++)
            {
                V0[i] = new double[1];
                V0[i][0] = V0matrix[i];
            }

            // creating covariance matrixes
            Q = new double[Dim][];
            R = new double[Dim][];
            for (i = 0; i < Dim; i++)
            {
                Q[i] = new double[Dim];
                R[i] = new double[Dim];
                for (j = 0; j < Dim; j++)
                {
                    Q[i][j] = Qmatrix[i][j];
                    R[i][j] = Rmatrix[i][j];
                }
            }

            // assigning initial measurements to the state estimate matrix
            Xest = new double[Dim][];
            for (i = 0; i < Dim; i++)
            {
                Xest[i] = new double[Dim];
                Xest[i][0] = Z0[i];
            }

            // setting step-0 a posteriori error covariance to 0
            Pest = new double[Dim][];
            for (i = 0; i < Dim; i++)
            {
                Pest[i] = new double[Dim];
                for (j = 0; j < Dim; j++)
                    Pest[i][j] = 0;
            }

            // simply creating the apriori state estimate matrix
            X_est = new double[Dim][];
            for (i = 0; i < Dim; i++)
                X_est[i] = new double[1];

            // Simply creating the a priori error covariance estimate matrix
            P_est = new double[Dim][];
            for (i = 0; i < Dim; i++)
                P_est[i] = new double[Dim];

            // Simply creating the Kalman Gain matrix
            K = new double[Dim][];
            for (i = 0; i < Dim; i++)
                K[i] = new double[Dim];

            step = 0;
        }

        // a static method to add matrices
        public static double[][] addMatrices(double[][] A, double[][] B, int rows, int cols)
        {
            int i, j;
            double[][] C = new double[rows][];

            for (i = 0; i < rows; i++)
            {
                C[i] = new double[cols];
                for (j = 0; j < cols; j++)
                    C[i][j] = A[i][j] + B[i][j];
            }

            return C;
        }

        // a static method to subtract matrices
        public static double[][] subtractMatrices(double[][] A, double[][] B, int rows, int cols)
        {
            int i, j;
            double[][] C = new double[rows][];

            for (i = 0; i < rows; i++)
            {
                C[i] = new double[cols];
                for (j = 0; j < cols; j++)
                    C[i][j] = A[i][j] - B[i][j];
            }

            return C;
        }

        // a static method to mutiply matrices
        public static double[][] mulMatrices(double[][] A, double[][] B, int rowsA, int colsA, 
                                             int colsB, ref int rowsC, ref int colsC)
        {
            int i, j;
            int k;
            double linecolprod;
            rowsC = rowsA;
            colsC = colsB;
            double[][] C = new double[rowsA][];
            for (i = 0; i < rowsC; i++)
                C[i] = new double[colsC];

            for (i = 0; i < rowsC; i++)
                for (j = 0; j < colsC; j++)
                {
                    linecolprod = 0;
                    for (k = 0; k < colsA; k++)
                        linecolprod += A[i][k] * B[k][j];
                    C[i][j] = linecolprod;
                }
            return C;
        }
        
        // a static method that computes the inverse of a matrix
        public static double[][] transposedMatrix(double[][] A, int rows, int cols)
        {
            int i, j;
            double[][] inverse = new double[cols][];

            for (i = 0; i < cols; i++)
            {
                inverse[i] = new double[rows];
                for (j = 0; j < rows; j++)
                    inverse[i][j] = A[j][i];
            }

            return inverse;
        }

        // a static method that reduces an nxn square matrix to a
        // complementary minor matrix n-1xn-1 given an element row, col
        public static double[][] reduceMatrix(double[][] A, int dim, int row, int col)
        {
            int ri, rj , i, j;
            double[][] reduced = new double[dim - 1][];

            for (ri = 0; ri < dim - 1; ri++)
                reduced[ri] = new double[dim - 1];
            
            ri = rj = 0;
            for (i = 0; i < dim; i++)
                for (j = 0; j < dim; j++)
                    if ((i != row) && (j != col))
                    {
                        reduced[ri][rj] = A[i][j];
                        rj++;
                        if (rj == dim - 1)
                        {
                            rj = 0;
                            ri++;
                        }
                    }
            return reduced;
        }


        // a static method that computes the determinant of a matrix
        public static double getDeterminant(double[][] A, int dim) {
            int i, j;
            double det = 0;
            if (dim == 1)
                det = A[0][0];
            else
            if (dim == 2) 
                det = A[0][0]*A[1][1] - A[1][0]*A[0][1];
            else {
                double sum = 0;
                
                    for (j=0; j<dim; j++) {
                        // reducing the dterminant's order using algebraic complements
                        double[][] minorMatrix = reduceMatrix(A, dim, 0, j);
                        // summing
                        sum += Math.Pow(-1, 1+j+1)*A[0][j]*getDeterminant(minorMatrix, dim-1);
                    }
                det = sum;
            }

            return det;
        }

        public static double[][] inverseMatrix(double[][] A, int dim)
        {
            int i, j;
            
            double[][] adjoint = new double[dim][];
            double[][] inverse = new double[dim][];

            if (dim == 1)
            {
                inverse[0] = new double[1];
                inverse[0][0] = 1 / A[0][0];
            }
            else
            {
                double determinant = getDeterminant(A, dim); ;

                for (i = 0; i < dim; i++)
                {
                    adjoint[i] = new double[dim];
                    inverse[i] = new double[dim];
                }
                // creating the adjoint
                for (i = 0; i < dim; i++)
                    for (j = 0; j < dim; j++)
                        adjoint[i][j] = Math.Pow(-1, i + j) * getDeterminant(reduceMatrix(A, dim, i, j), dim - 1);
                // transposing the adjoint
                double[][] adjointT = transposedMatrix(adjoint, dim, dim);
                // finding the inverse now
                for (i = 0; i < dim; i++)
                    for (j = 0; j < dim; j++)
                        inverse[i][j] = adjointT[i][j] / determinant;
            }
            return inverse;
        }



        public double[] getNewEstimate(double[] Zmatrix, double[] Umatrix)
        {
            int i, j;
            double[][] U = new double[1][];
            U[0] = new double[Dim];

            // copying input matrix values
            for (i = 0; i < Dim; i++)
                U[0][i] = Umatrix[i];

            // copying measured values to Z
            Z = new double[Dim][];
            for (i = 0; i < Dim; i++)
            {
                Z[i] = new double[1];
                Z[i][0] = Zmatrix[i];
            }

            // ***************** Computations *************************
            int r = 0, c = 0;
            // increasing step
            step++;


            // ***************** A. Prediction phase **************************
            // A1. A priori error estimate X_
            X_est = addMatrices(addMatrices(mulMatrices(A, Xest, Dim, Dim, 1, ref r, ref c),
                                mulMatrices(B, U, Dim, 1, Dim, ref r, ref c),
                                Dim, 1),
                                W0,
                                Dim, 1);
            // A2. A priori covariance Pk_
            P_est = addMatrices(mulMatrices(mulMatrices(A, Pest, Dim, Dim, Dim, ref r, ref c),
                                             transposedMatrix(A, Dim, Dim), Dim, Dim, Dim, ref r, ref c),
                                 Q, Dim, Dim);

            // **************** B. Measurement Update phase ********************
            // B1. Kalman Gain
            K = mulMatrices(mulMatrices(P_est, transposedMatrix(H, Dim, Dim), Dim, Dim, Dim, ref r, ref c),
                             inverseMatrix(addMatrices(mulMatrices(mulMatrices(H, P_est, Dim, Dim, Dim, ref r, ref c),
                                                                               transposedMatrix(H, Dim, Dim), Dim,
                                                                               Dim, Dim, ref r, ref c),
                                                                               R, Dim, Dim), Dim),
                                                                               Dim, Dim, Dim, ref r, ref c);
            // B2. A posteriori estimate update
            Xest = addMatrices(X_est, mulMatrices(K, subtractMatrices(subtractMatrices(Z, V0, Dim, 1),
                                                                      mulMatrices(H, X_est, Dim, Dim, 1, ref r, ref c), Dim, 1),
                                                                      Dim,Dim, 1, ref r, ref c), Dim, 1);
            // B3. A posteriori covariance Pk
            // creating a unity matrix
            double[][] I = new double[Dim][];
            for (i = 0; i < Dim; i++)
            {
                I[i] = new double[Dim];
                for (j = 0; j < Dim; j++)
                    I[i][j] = (i == j) ? 1 : 0;
            }

            Pest = mulMatrices(subtractMatrices(I, mulMatrices(K, H, Dim, Dim, Dim, ref r, ref c), Dim, Dim),
                               P_est, Dim, Dim, Dim, ref r, ref c);

            // creating a regular array to store the prediction
            double[] retval = new double[Dim];
            for (i = 0; i < Dim; i++)
                retval[i] = Xest[i][0];

            return retval;

        }

                            
        

    }
}
