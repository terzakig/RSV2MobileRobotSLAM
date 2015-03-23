using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class ANN
    {
        public ANNLayer[] Network;
        public int InputNum, LayerNum, OutputNum;
        public double Threshold, LR;
        public int NeuronNum;
        public Random rnd;

        // ANN constructor. Input and Hidden layers contain the same number of neurons.
        public ANN(int inputnum, int layernum, int neuronnum, int outputnum,
                    double threshold, double lr, Random rand)
        {
            InputNum = inputnum;
            LayerNum = layernum;
            NeuronNum = neuronnum;
            OutputNum = outputnum;
            LR = lr;
            Threshold = threshold;
            rnd = rand;
            // creating the network
            Network = new ANNLayer[LayerNum];
            // creating the input layer
            Network[0] = new ANNLayer(NeuronNum, InputNum, Threshold, LR);
            // assigning random weights
            for (int j = 0; j < NeuronNum; j++)
                for (int k = 0; k < InputNum; k++)
                {
                    int sign = (rnd.Next(101) < 50) ? -1 : 1;
                    Network[0].Neurons[j].Weights[k] = sign*rnd.NextDouble();
                }
            // creating hidden layers
            int i;
            for (i = 1; i < LayerNum - 1; i++)
            {
                Network[i] = new ANNLayer(NeuronNum, NeuronNum, Threshold, LR);
                // assigning random weights
                for (int j = 0; j < NeuronNum; j++)
                    for (int k = 0; k < NeuronNum; k++)
                    {
                        int sign = (rnd.Next(101) < 50) ? -1 : 1;
                        Network[i].Neurons[j].Weights[k] = sign * rnd.NextDouble();
                    }
            }
            // creating the output layer
            Network[LayerNum - 1] = new ANNLayer(OutputNum, NeuronNum, Threshold, LR);
            // assigning random weights
            for (int j = 0; j < OutputNum; j++)
                for (int k = 0; k < NeuronNum; k++)
                    Network[LayerNum - 1].Neurons[j].Weights[k] = rnd.NextDouble();

        }



        // layer outputs at layerindex
        public double[] sigmoidLayerOutputs(double[] inputvec, int LayerIndex)
        {
            double[] curoutputs, prevoutputs;
            curoutputs = new double[NeuronNum];
            prevoutputs = new double[NeuronNum];
            // calculating input layer outputs
            int i, j, k;
            for (i = 0; i < NeuronNum; i++)
                curoutputs[i] = Network[0].Neurons[i].sigmOutput(inputvec);
            // examining if the requested layer concerns the output layer
            int lastlayer = (LayerIndex < LayerNum - 1) ? LayerIndex : LayerNum - 2;
            // now feedforwarding outputs to the requested layer
            for (j = 1; j <= lastlayer; j++)
            {
                // copying curoutputs to prevoutputs
                for (k = 0; k < NeuronNum; k++)
                    prevoutputs[k] = curoutputs[k];
                // done
                for (i = 0; i < NeuronNum; i++)
                    // Now using prevoutputs as inputs to the current layer
                    // in order to calculate its outputs
                    curoutputs[i] = Network[j].Neurons[i].sigmOutput(prevoutputs);
                // current layer outputs calculated
            }

            if (LayerIndex == LayerNum - 1)
            { // requested layer is the output layer
                // copying curoutputs to prevoutputs
                for (k = 0; k < NeuronNum; k++)
                    prevoutputs[k] = curoutputs[k];
                // outputs copied
                // now re-creating curoutputs using outputnum
                curoutputs = new double[OutputNum];
                // calculating outputs
                for (i = 0; i < OutputNum; i++)
                    curoutputs[i] = Network[LayerNum - 1].Neurons[i].sigmOutput(prevoutputs);
            }

            return curoutputs;
        }

        // back propagate the error and update the weights of the network given the desired output vector
        public void backPropagate(double[] inputvec, double[] desiredoutputvec)
        {
            double[][] outputerrors;
            double[] actualoutputvec;
            double[] layerinputvec;
            double sum;
            int i, j, k, h;
            outputerrors = new double[LayerNum][];
            // parsing the network backwards
            for (j = LayerNum - 1; j >= 0; j--)
            {
                // creating the current layer error line according to the number of neurons in it
                outputerrors[j] = new double[Network[j].NeuronNum];
                // now retrieving the actual outputs of the layer
                actualoutputvec = sigmoidLayerOutputs(inputvec, j);
                // now calculating the error. If current layer is the output, we use the desiredoutputs vector
                if (j == LayerNum - 1)
                { // current layer is the output layer
                    for (k = 0; k < OutputNum; k++)
                        outputerrors[j][k] = actualoutputvec[k] * (1 - actualoutputvec[k]) * (desiredoutputvec[k] - actualoutputvec[k]);
                }
                else
                { // hidden layer or input layer
                    // calulcating error for each neuron in the layer
                    for (h = 0; h < NeuronNum; h++)
                    {
                        // calculating the sum of errors for each weight in the next layer
                        sum = 0;
                        for (k = 0; k < Network[j + 1].NeuronNum; k++)
                            sum += Network[j + 1].Neurons[k].Weights[h] * outputerrors[j + 1][k];
                        // sum calculated for neuron-h in layer-j
                        // now calculating error
                        outputerrors[j][h] = actualoutputvec[h] * (1 - actualoutputvec[h]) * sum;
                    }
                }
            }
            // network errors stored
            // updating network weights
            for (j = LayerNum - 1; j >= 0; j--)
            {
                // computing the input vector for the current layer
                if (j == 0) // input layer. Input is the network input 
                    layerinputvec = inputvec;
                else  // hidden or output layer. Input should be the previous layer outputs
                    layerinputvec = sigmoidLayerOutputs(inputvec, j - 1);
                // updating layer weights
                for (k = 0; k < Network[j].NeuronNum; k++)
                {// neuron enumeration
                    int weightnum = (j == 0) ? InputNum : Network[j - 1].NeuronNum;
                    for (i = 0; i < weightnum; i++) // weight enumeration in neuron 
                        Network[j].Neurons[k].updateWeight(i, outputerrors[j][k], layerinputvec[i]);
                }
            }


        }

        public void setLearningRate(double lr)
        {
            int i;
            LR = lr;
            for (i = 0; i < LayerNum; i++)
                Network[i].setLearningRate(LR);
        }

    }
}
