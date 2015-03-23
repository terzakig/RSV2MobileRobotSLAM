using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class Neuron
    {
        public double[] Weights;
        public int InputNum;
        public double Threshold;
        public double LR; // Learning Rate

        public Neuron(int inputnum, double threshold, double lr)
        {
            InputNum = inputnum;
            Threshold = threshold;
            LR = lr;
            // creating the weights array
            Weights = new double[InputNum];
        }

        // thresholded output as integer
        public int binOutputAsInt(double[] inputvec)
        {
            int i;
            double sum = 0;
            for (i = 0; i < InputNum; i++)
                sum += Weights[i] * inputvec[i];
            return (sum < Threshold) ? 0 : 1;
        }

        // thresholded output as double
        public double binOutputAsDouble(double[] inputvec)
        {
            int i;
            double sum = 0;
            for (i = 0; i < InputNum; i++)
                sum += Weights[i] * inputvec[i];
            return (sum < Threshold) ? 0.0 : 1.0;
        }
        // sigmoid output as double
        public double sigmOutput(double[] inputvec)
        {
            int i;
            double sum = 0;
            for (i = 0; i < InputNum; i++)
                sum += Weights[i] * inputvec[i];
            return 1 / (1 + Math.Exp(-sum));
        }

        // Weight update in back-propagation using error and the coresponding previous input at windex
        public void updateWeight(int windex, double error, double prevInput)
        {
            Weights[windex] += LR * error * prevInput;
        }

    }
}
