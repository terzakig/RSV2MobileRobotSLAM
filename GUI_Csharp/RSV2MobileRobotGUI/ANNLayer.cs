using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class ANNLayer
    {
        public Neuron[] Neurons;
        public int NeuronNum;
        public int InputNum;
        public double Threshold;
        public double LR;

        public ANNLayer(int neuronnum, int inputnum, double threshold, double lr)
        {
            NeuronNum = neuronnum;
            InputNum = inputnum;
            Threshold = threshold;
            LR = lr;

            Neurons = new Neuron[NeuronNum];
            int i;
            for (i = 0; i < NeuronNum; i++)
                Neurons[i] = new Neuron(InputNum, Threshold, LR);
        }

        public void setLearningRate(double lr)
        {
            int i;
            LR = lr;
            for (i = 0; i < NeuronNum; i++)
                Neurons[i].LR = LR;
        }
    }
}
