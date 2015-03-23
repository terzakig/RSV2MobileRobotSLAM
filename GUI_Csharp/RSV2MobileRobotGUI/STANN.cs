using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class STANN : ANN
    {
        public double[][] ClusterCentroids;
        public double[][] ClusterSums;
        public int[] ClusterVectorCount;
        public int ClusterNum;
        public int InputPermutationsNum;
        public int InputRange;
        public Boolean ForcedSelfTrain;

        public HistogramEntry InputHisto;


        // a constantly populated histogram (aiding the ongoing clustering process)
        //public int[] InputHistogram;

        public STANN(int inputnum, int inputrange, int layernum, 
                     int neuronnum, int outputnum, double threshold, 
                     double lr, Random rand, Boolean forcedSelfTrain)
            : base(inputnum, layernum, neuronnum, outputnum, threshold, lr, rand)
        {
            ForcedSelfTrain = forcedSelfTrain;
            InputRange = inputrange;
            if (ForcedSelfTrain)
            {
                // computing number of clusters
                ClusterNum = (int)Math.Pow(2, OutputNum);
                // initializing the centroids array and the input sums per cluster
                resetClusterCentroids();
                // retrieving the number of possible input permutations
                InputPermutationsNum = getInputPermsNumber();
                // resetting the Input Histogram

                InputHisto = null;
                //resetInputHistogram();
            }


        }


        public void resetClusterVectorCount()
        {
            int i;
            ClusterVectorCount = new int[ClusterNum];
            for (i = 0; i < ClusterNum; i++)
                ClusterVectorCount[i] = 0;
        }

      /*  public void resetInputHistogram()
        {
            int i;
            InputHistogram = new int[InputPermutationsNum];
            for (i = 0; i < InputPermutationsNum; i++)
                InputHistogram[i] = 0;
        }*/

        public void copyCentroids(ref double[][] vec1, double[][] vec2)
        {
            int i, j;
            for (i = 0; i < ClusterNum; i++)
                for (j = 0; j < InputNum; j++)
                    vec1[i][j] = vec2[i][j];
        }

        public int getInputVectorIndex(double[] inputvec)
        {
            int i;
            int index = 0;
            for (i = 0; i < InputNum; i++)
                index += (int)(Math.Pow(InputRange, i) * inputvec[i]);
            return index;
        }


        public double[] getHistogramVector(int index)
        {
            int i, num;
            double[] histvec = new double[InputNum];
            for (i = 0; i < InputNum; i++)
                histvec[i] = 0;

            num = index;
            i = 0;

            do
            {
                int quot = num % InputRange;
                histvec[i] = (double)quot;
                i++;
                num = num / InputRange;
                if (num < InputRange) histvec[i] = (double)num;
            } while (num >= InputRange);

            return histvec;
        }


        /*public int histoElementNumber()
        {
            int i, count = 0;
            for (i = 0; i < InputPermutationsNum; i++)
                count += InputHistogram[i];
            return count;
        }*/

       /* public void autoClustering1(double[] inputvec)
        {
            int i, j;
            int inputHistoIndex;
            Boolean equal;

            // updating the histogram
            inputHistoIndex = getInputVectorIndex(inputvec);
            InputHistogram[inputHistoIndex]++;
            int vectorPoolLength = histoElementNumber();
            // histogram updated

            // creating a temporary array to store centroids calculated in the previous iteration step
            double[][] prevCentroids = new double[ClusterNum][];

            for (i = 0; i < ClusterNum; i++)
                prevCentroids[i] = new double[InputNum];
            // array created and initialized

            // iterating
            do
            {
                // copying centroids to previous centrodis array
                copyCentroids(ref prevCentroids, ClusterCentroids);
                // resetting the sums
                resetClusterSums();
                // reseting the vector count for each cluster
                resetClusterVectorCount();
                // creating sums using the histogram
                for (i = 0; i < InputPermutationsNum; i++)
                {
                    if (InputHistogram[i] > 0)
                    {
                        double[] thevec = getHistogramVector(i);
                        // now finding the closest centroid to the vector at hand
                        double minDistance = getEuclideanDistance(ClusterCentroids[0], thevec, InputNum);
                        int minDistanceCluster = 0;
                        for (j = 0; j < ClusterNum; j++)
                        {
                            double dist = getEuclideanDistance(ClusterCentroids[j], thevec, InputNum);
                            if (dist <= minDistance)
                            {
                                minDistance = dist;
                                minDistanceCluster = j;
                            }
                        }
                        // closest centroid found.
                        // Adding the vector to the sum weighted by the histogram entry
                        int k;
                        for (k = 0; k < InputNum; k++)
                            ClusterSums[minDistanceCluster][k] += InputHistogram[i] * thevec[k];
                        // vector added (f-times, f is the histogram entry for this particular vector)
                        // increasing the vector count of the cluster
                        ClusterVectorCount[minDistanceCluster] += InputHistogram[i];
                    }
                }

                // finding new centroids (computing averages)
                for (i = 0; i < ClusterNum; i++)
                    if (ClusterVectorCount[i] > 0)
                        for (j = 0; j < InputNum; j++)
                            ClusterCentroids[i][j] = ClusterSums[i][j] / ClusterVectorCount[i];
                // new centroids stored
                // Now checking if previous centroids are equal to the new centroids
                equal = true;
                for (i = 0; i < ClusterNum; i++)
                {
                    if (!equal) break;
                    for (j = 0; j < InputNum; j++)
                        if (prevCentroids[i][j] != ClusterCentroids[i][j])
                        {
                            equal = false;
                            break;
                        }
                }
            } while (!equal);
        }
        */

        public void autoClustering(double[] inputvec)
        {
            int i, j;
            int inputHistoIndex;
            Boolean equal;
            

            // updating the histogram
            inputHistoIndex = mapVector2Int(inputvec, InputRange, InputNum);
            HistogramEntry.increaseFrequency (ref InputHisto, inputHistoIndex);
            
            int vectorPoolLength = HistogramEntry.HistogramLength(InputHisto);
            // histogram updated

            // creating a temporary array to store centroids calculated in the previous iteration step
            double[][] prevCentroids = new double[ClusterNum][];

            for (i = 0; i < ClusterNum; i++)
                prevCentroids[i] = new double[InputNum];
            // array created and initialized

            // iterating
            do
            {
                // copying centroids to previous centrodis array
                copyCentroids(ref prevCentroids, ClusterCentroids);
                // resetting the sums
                resetClusterSums();
                // reseting the vector count for each cluster
                resetClusterVectorCount();
                // creating sums using the histogram
                HistogramEntry temp;
                for (temp = InputHisto; temp!=null ; temp = temp.next)
                {
                    
                        int[] intvec = mapInt2Vector(temp.Input, InputRange, InputNum);
                        double[] thevec = new double[InputNum];
                        for (int c = 0; c < InputNum; c++)
                            thevec[c] = (double)intvec[c];
                        // now finding the closest centroid to the vector at hand
                        double minDistance = getEuclideanDistance(ClusterCentroids[0], thevec, InputNum);
                        int minDistanceCluster = 0;
                        for (j = 0; j < ClusterNum; j++)
                        {
                            double dist = getEuclideanDistance(ClusterCentroids[j], thevec, InputNum);
                            if (dist <= minDistance)
                            {
                                minDistance = dist;
                                minDistanceCluster = j;
                            }
                        }
                        // closest centroid found.
                        // Adding the vector to the sum weighted by the histogram entry
                        int k;
                        for (k = 0; k < InputNum; k++)
                            ClusterSums[minDistanceCluster][k] += temp.Frequency  * thevec[k];
                            
                        // vector added (f-times, f is the histogram entry for this particular vector)
                        // increasing the vector count of the cluster
                        ClusterVectorCount[minDistanceCluster] += temp.Frequency;
                    
                }

                // finding new centroids (computing averages)
                for (i = 0; i < ClusterNum; i++)
                    if (ClusterVectorCount[i] > 0)
                        for (j = 0; j < InputNum; j++)
                            ClusterCentroids[i][j] = ClusterSums[i][j] / ClusterVectorCount[i];
                // new centroids stored
                // Now checking if previous centroids are equal to the new centroids
                equal = true;
                for (i = 0; i < ClusterNum; i++)
                {
                    if (!equal) break;
                    for (j = 0; j < InputNum; j++)
                        if (prevCentroids[i][j] != ClusterCentroids[i][j])
                        {
                            equal = false;
                            break;
                        }
                }
            } while (!equal);
        }


        public void resetClusterCentroids()
        {
            int i;
            ClusterCentroids = new double[ClusterNum][];
            for (i = 0; i < ClusterNum; i++)
            {
                ClusterCentroids[i] = new double[InputNum];
                ClusterCentroids[i] = getHistogramVector(i);
            }
        }

        public void resetClusterSums()
        {
            ClusterSums = new double[ClusterNum][];
            int i, j;
            for (i = 0; i < ClusterNum; i++)
            {
                ClusterSums[i] = new double[InputNum];
                for (j = 0; j < InputNum; j++)
                    ClusterSums[i][j] = 0;
            }
        }

        public int getInputPermsNumber()
        {
            int retval = (int)Math.Pow(InputRange, InputNum);
            return retval;
        }





        public static double getEuclideanDistance(double[] vec1, double[] vec2, int dim)
        {
            int i;
            double sum = 0;
            for (i = 0; i < dim; i++)
                sum += Math.Pow(vec1[i] - vec2[i], 2);
            sum = Math.Sqrt(sum);

            return sum;
        }

        public double[] int2BinaryVector(int number)
        {
            int num = number, i;
            double[] binary = new double[OutputNum];

            i = 0;
            do
            {
                int quot = num % 2;
                binary[i] = (double)quot;
                i++;
                num = num / 2;
                if (num < 2) binary[i] = num;
            } while (num >= 2);

            return binary;
        }

        public void selfTrain(double[] inputvec)
        {

            // rearranging centroids
            autoClustering(inputvec);
            
            int i;
            double minDistance = getEuclideanDistance(ClusterCentroids[0], inputvec, InputNum);
            int minDistanceCentroidIndex = 0;

            for (i = 0; i < ClusterNum; i++)
            {
                double dist = getEuclideanDistance(ClusterCentroids[i], inputvec, InputNum);
                if ( minDistance > dist)
                {
                    minDistance = dist;
                    minDistanceCentroidIndex = i;
                }
            }
            // converting the minimum distance cluster index to a vector of binary digits
            double[] DesiredOutputVec = int2BinaryVector(minDistanceCentroidIndex);
            // back propagating 5 times...
            for (i=0; i<5; i++)
                this.backPropagate(inputvec, DesiredOutputVec);
        }


        // the following static methods will map a vector of integers to a unique integer
        // and vice versa
        // 1. mapping integer to a vector
        public static int[] mapInt2Vector(int number, int range, int vectorlength)
        {
            int num = number, i, j;
            int[] tempvec = new int[vectorlength]; // max vector length is 200

            i = 0;
            do
            {
                int quot = num % range;
                tempvec[i] = quot;
                i++;
                num = num / range;
                if (num < range) tempvec[i] = num;
            } while ((num >= range)&&(i<vectorlength-1));
            //vectorlength = i + 1;
            
            for (j = i+1; j < vectorlength; j++)
                tempvec[j] = 0;

            return tempvec;
        }

        public static double[] mapInt2VectorDouble(int number, int range, int vectorlength)
        {
            int num = number, i, j;
            double[] tempvec = new double[vectorlength]; // max vector length is 200

            i = 0;
            do
            {
                int quot = num % range;
                tempvec[i] = quot;
                i++;
                num = num / range;
                if (num < range) tempvec[i] = num;
            } while ((num >= range)&&(i<vectorlength-1));
            //vectorlength = i + 1;

            for (j = i + 1; j < vectorlength; j++)
                tempvec[j] = 0;

            return tempvec;
        }


        // 2. mapping vector to an integer
        public static int mapVector2Int(int[] vector, int range, int vectorlength)
        {
            int i, num = 0;
            for (i = 0; i < vectorlength; i++)
                num += (int)(vector[i] * Math.Pow(range, i));

            return num;
        }
        // overloaded for vectors of double types
        public static int mapVector2Int(double[] vector, int range, int vectorlength)
        {
            int i, num = 0;
            for (i = 0; i < vectorlength; i++) 
                num += (int)(vector[i] * Math.Pow(range, i));

            return num;
        }


        public static int sigmoids2Int(double[] sigmoids, int veclength)
        {
            int theoutput = 0;
            int i;
            for (i = 0; i < veclength; i++)
            {
                int bit = (sigmoids[i] < 0.5) ? 0 : 1;
                theoutput += (int)Math.Pow(2, i) * bit;
            }
            return theoutput;
        }

    }
}
