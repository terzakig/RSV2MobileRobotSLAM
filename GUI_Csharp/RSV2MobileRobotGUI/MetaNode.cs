using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class MetaNode
    {
        // The node's Neural Network
        public STANN stann;

        // parent nodes receiving input from this treenode
        public MetaNode[] Parents;
        public int ParentNum;

        // node children in the tree-structure
        public MetaNode[] Children;
        public int ChildrenNum;
        // number of raw numeric inputs (aside the children inputs)
        public int RawInputNum;

        // stann attributes
        public int InputNum, LayerNum, NeuronNum, InputRange;
        public double Threshold, LR;
        // global random number generator
        public Random rnd;
        // previous step output and output. Stored to be used for Reinforcement Learning purposes
        public double PreviousOutput; // previous decimal thresholded output eqivalent of the metanode 
        public double CurrentOutput;  // if a node is accessed for a second, third, etc. than once in a network output calculation
                                      // this value is accessed instead of recalculating the output
        public double NoUpdatePassOutput; // holds the current output of the node
                                          // when parsing for output without doing updates
                                          // to the nodes - getOutputNoUpdate()
        public int OutputPass;        // identifies the last access of the node for output production
        public double[] PreviousInputVec; // previous input vector to the metnode
        
        // self-train attribute
        public Boolean ForcedSelfTrain;
        public Boolean ForcedQLearning;
        // node level in the overall tree-like network
        public int NodeLevel;

        // Q-learning variables 
        //public double[] RewardTable;
        //public double[][] QTable;
        RewardTableEntry RewardTable;
        QTableEntry QTable;


        public NodeIOLogEntry[] IOLog; // used to update rewards for several last steps
        public int IOLogLength;        // length of the IO log
        public static int MAX_IO_LOG_LENGTH = 50; // maximum length of the IO log

        public double Qalpha, Qgamma; // Q-learning reward update parameters

        // the leaf's index (used to apply proper input vector)
        public int LeafIndex;


        public MetaNode(int rawinputnum, MetaNode[] children, int childrennum,
                            MetaNode[] parents, int parentnum, int nodeoutputnum,
                            int rawinputrange, int layernum, int neuronnum,
                            double threshold, double lr, Random rand,
                            Boolean forceSelfTrain, Boolean forcedQLearning,
                            int nodelevel, double alpha, double gamma, int leafindex)
        {
            int i;
            RawInputNum = rawinputnum;
            ChildrenNum = childrennum;
            LayerNum = layernum;
            NeuronNum = neuronnum;
            Threshold = threshold;
            LR = lr;
            rnd = rand;
            ParentNum = parentnum;
            ForcedSelfTrain = forceSelfTrain;
            ForcedQLearning = forcedQLearning;
            NodeLevel = nodelevel;
            Qalpha = alpha;
            Qgamma = gamma;
            LeafIndex = leafindex;
            // copying children array. also figuring out the input range for the stann
            int maxrange = rawinputrange;
            Children = new MetaNode[ChildrenNum];
            for (i = 0; i < ChildrenNum; i++)
            {
                Children[i] = children[i];
                Children[i].addParent(this);
                
                maxrange = (maxrange < Math.Pow(2, Children[i].stann.OutputNum)) ? (int)Math.Pow(2, Children[i].stann.OutputNum) : maxrange;
            }
            InputRange = maxrange;
            // copying parent array
            for (i = 0; i < ParentNum; i++)
                Parents[i] = parents[i];

            InputNum = getInputNum();
            // now creating the STANN or the ANN of the node
                stann = new STANN(InputNum, InputRange, LayerNum,
                                  NeuronNum, nodeoutputnum, Threshold, LR, rnd, ForcedSelfTrain);
            // initializing previous input vector and previous output properties to zero
            CurrentOutput = PreviousOutput = 0;
            OutputPass = 0;
            PreviousInputVec = new double[InputNum];
            for (i = 0; i < InputNum; i++)
                PreviousInputVec[i] = 0;
            
            
            // initializing the Reward Table and table of Q-values for possible Q-learning use (or abuse :))
            // if the ForcedQLearning Property is set
            if (ForcedQLearning)
            {/*
                int possibleinputs = (int)Math.Pow(InputRange, InputNum);
                int possibleoutputs = (int)Math.Pow(2, stann.OutputNum);

                QTable = new double[possibleinputs][];
                RewardTable = new double[possibleinputs];
                for (i = 0; i < possibleinputs; i++)
                {
                    QTable[i] = new double[possibleoutputs];
                    RewardTable[i] = 0;
                    for (j = 0; j < possibleoutputs; j++)
                        QTable[i][j] = 0;
                } */

                RewardTable = null;
                QTable = null;
                // initializing the IO log
                IOLog = new NodeIOLogEntry[MAX_IO_LOG_LENGTH];
                IOLogLength = 0;
            }

        }
        



        // return total number of inputs to the node
        public int getInputNum()
        {
            return RawInputNum + ChildrenNum;
        }

        // returns the number of binary outputs of the node
        public int getOutputNum()
        {
            return stann.OutputNum;
        }
        // adds a parent tp the Parents list. No change to the STANN
        public void addParent(MetaNode parent)
        {
            int i;
            MetaNode[] temp;
            if (ParentNum == 0)
            {
                ParentNum++;
                Parents = new MetaNode[ParentNum];
            }
            else
            {
                temp = new MetaNode[ParentNum];
                for (i = 0; i < ParentNum; i++)
                    temp[i] = Parents[i];
                ParentNum++;
                Parents = new MetaNode[ParentNum];
                for (i = 0; i < ParentNum - 1; i++)
                    Parents[i] = temp[i];
            }
            Parents[ParentNum - 1] = parent;
        }

        // adds a child to the node. the STANN MUST be recreated (due to change on the inputs)
        public void addChild(MetaNode child)
        {
            int i;
            MetaNode[] temp;

            if (ChildrenNum == 0)
            {
                // increase the number of children
                ChildrenNum++;
                // increase the number of inputs as well!
                InputNum++;
                Children = new MetaNode[ChildrenNum];
            }
            else
            {
                temp = new MetaNode[ChildrenNum];
                for (i = 0; i < ChildrenNum; i++)
                    temp[i] = Children[i];
                // increase number of children
                ChildrenNum++;
                // increase number of inputs as well!
                InputNum++;
                Children = new MetaNode[ChildrenNum];
                for (i = 0; i < ChildrenNum - 1; i++)
                    Children[i] = temp[i];
            }
            Children[ChildrenNum - 1] = child;

            int newinputrange, curoutputnum = stann.OutputNum;
            if (stann.InputRange < (int)Math.Pow(2, child.stann.OutputNum))
            {
                newinputrange = (int)Math.Pow(2, child.stann.OutputNum);
                InputRange = newinputrange;
            }
            else
            {
                newinputrange = stann.InputRange;
                InputRange = stann.InputRange;
            }

            // recreating the STANN
            stann = new STANN(getInputNum(), newinputrange, LayerNum, 
                              NeuronNum, curoutputnum, Threshold, LR, rnd, ForcedSelfTrain);

            if (ForcedQLearning)
            {
                // Re-initializing the Reward Table and table of Q-values for possible Q-learning use (or abuse :))
               /*
                int possibleinputs = (int)Math.Pow(InputRange, InputNum);
                int possibleoutputs = (int)Math.Pow(2, stann.OutputNum);

                QTable = new double[possibleinputs][];
                RewardTable = new double[possibleinputs];
                for (i = 0; i < possibleinputs; i++)
                {
                    QTable[i] = new double[possibleoutputs];
                    RewardTable[i] = 0;
                    for (j = 0; j < possibleoutputs; j++)
                        QTable[i][j] = 0;
                }
                */
                RewardTable = null;
                QTable = null;
            }
            // Re-creating the previous input vector 
            PreviousInputVec = new double[InputNum];
            for (i = 0; i < InputNum; i++)
                PreviousInputVec[i] = 0;

        }

        // static methods that create a tree or a branch

        // Leaf level creation (MUST be connected to the raw/processed sensory input)
        public static MetaNode[] createTreeLeaves(int leavesnum, int leafinputnum, int inputrange,
                                                   int stannlayernum, int stannneuronnum, int leafoutputnum,
                                                   double stannthreshold, double stannlr,
                                                   MetaNode[] parents, Random rand, int startindex)
        {

            MetaNode[] leaves = new MetaNode[leavesnum];

            // creating each node individually
            int i;
            for (i = 0; i < leavesnum; i++)
            {
                leaves[i] = new MetaNode(leafinputnum, null, 0, parents, 0,
                                            leafoutputnum, inputrange, stannlayernum,
                                            stannneuronnum, stannthreshold, stannlr, 
                                            rand, true, false, 0, 0, 0, startindex+i);
            }

            return leaves;
        }


        // Single Leaf level creation (MUST be connected to the raw/processed sensory input)
        public static MetaNode createTreeLeaf(int leafinputnum, int inputrange,
                                                   int stannlayernum, int stannneuronnum, int leafoutputnum,
                                                   double stannthreshold, double stannlr,
                                                   MetaNode[] parents, Random rand, int leafindex)
        {

            MetaNode leaf;

            // creating node 
            
            
           leaf = new MetaNode(leafinputnum, null, 0, parents, 0,
                                            leafoutputnum, inputrange, stannlayernum,
                                            stannneuronnum, stannthreshold, stannlr, 
                                            rand, true, false, 0, 0, 0, leafindex);
            

            return leaf;
        }


        // Higher or intermediate level nodes creation
        public static MetaNode[] createHigherLevelNodes(int nodenum, MetaNode[] children, int childrennum,
                                               int stannlayernum, int stannneuronnum, int nodeoutputnum,
                                               double stannthreshold, double stannlr,
                                               MetaNode[] parents, int parentnum,
                                               Random rand, Boolean selftrain,
                                               Boolean supervised, double alpha, double gamma, int level)
        {

            MetaNode[] nodes = new MetaNode[nodenum];
            int i;
            // creating each node individually
            for (i = 0; i < nodenum; i++)
                nodes[i] = new MetaNode(0, children, childrennum, parents,
                                           parentnum, nodeoutputnum, 0,
                                           stannlayernum, stannneuronnum, stannthreshold,
                                           stannlr, rand, selftrain, supervised, level, alpha, gamma, 0);

            return nodes;
        }

        // Higher or intermediate level single node creation
        public static MetaNode createHigherLevelNode(MetaNode[] children, int childrennum,
                                               int stannlayernum, int stannneuronnum, int nodeoutputnum,
                                               double stannthreshold, double stannlr,
                                               MetaNode[] parents, int parentnum,
                                               Random rand, Boolean selftrain,
                                               Boolean supervised, double alpha, double gamma, int level)
        {

            MetaNode node;
            
            // creating node 
            
            node = new MetaNode(0, children, childrennum, parents,
                                           parentnum, nodeoutputnum, 0,
                                           stannlayernum, stannneuronnum, stannthreshold,
                                           stannlr, rand, selftrain, supervised, level, alpha, gamma, 0);

            return node;
        }


        // return the output of a MetaNode branch given the input vector to the leaves
        /*
        public static double getOutput(MetaNode mnet, double[][] inputvec, int pass)
        {
            int i;
            double[] sigmoids;
            double theoutput;

            if (mnet.ChildrenNum == 0)
            {
                if (mnet.OutputPass != pass)
                {
                    mnet.OutputPass = pass;
                    // self training if the node has its ForceSelfTrain attribute set to true
                    if (mnet.ForcedSelfTrain)
                        // self training 
                        mnet.stann.selfTrain(inputvec[mnet.LeafIndex]);
                   
                    // retrieving the sigmoids of the node
                    sigmoids = mnet.stann.sigmoidLayerOutputs(inputvec[mnet.LeafIndex], mnet.stann.LayerNum - 1);
                    // calculating the decimal equivalent to the ordered thresholded sigmoid outputs
                    theoutput = STANN.mapVector2Int(sigmoids, 2, mnet.stann.OutputNum);
                    
                    mnet.PreviousOutput = mnet.CurrentOutput;
                    mnet.CurrentOutput = theoutput;
                }
                else
                    theoutput = mnet.CurrentOutput;

            }
            else
            {
                if (mnet.OutputPass != pass)
                {
                    mnet.OutputPass = pass;
                    double[] levelinput = new double[mnet.InputNum];

                    for (i = 0; i < mnet.InputNum; i++)
                        if (mnet.Children[i].NodeLevel >= mnet.NodeLevel) // requesting output from a higher (or equal ) level node
                            levelinput[i] = mnet.Children[i].PreviousOutput; // avoiding circular reference in recursion
                        else
                            levelinput[i] = getOutput(mnet.Children[i], inputvec, pass);

                    // self training if the aselfrain attribute is on
                    if (mnet.ForcedSelfTrain) mnet.stann.selfTrain(levelinput);
                    // retrieving sigmoids
                    sigmoids = mnet.stann.sigmoidLayerOutputs(levelinput, mnet.stann.LayerNum - 1);
                    // calculating the decimal equivalent to the thresholded outputs
                    theoutput = 0;
                    for (i = 0; i < mnet.stann.OutputNum; i++)
                    {
                        int bit = (sigmoids[i] < 0.5) ? 0 : 1;
                        theoutput += (int)Math.Pow(2, i) * bit;
                    }
                    // updating previous Input Vector and Previous Output properties of the metanode
                    int t;
                    mnet.PreviousInputVec = new double[mnet.InputNum];
                    for (t = 0; t < mnet.InputNum; t++)
                        mnet.PreviousInputVec[t] = levelinput[t];
                    mnet.PreviousOutput = mnet.CurrentOutput;
                    mnet.CurrentOutput = theoutput;
                    // previous input vector and output updated with the new values

                    // Must now train the network!!!! (in case the qlearning property is on)
                    if (mnet.ForcedQLearning)
                    {
                        // mapping the input to the proper index in the reward table
                        int inputindex = 0;
                        for (t = 0; t < mnet.InputNum; t++)
                            inputindex += (int)(Math.Pow(mnet.InputRange, t) * levelinput[t]);
                        // finding the output that corresponds to the maximum Qvaluefor the given input
                        double maxQvalue = mnet.getMaximumQValue(inputindex);
                        int maxQvalueOutputindex = 0;
                        while (mnet.QTable[inputindex][maxQvalueOutputindex] != maxQvalue)
                            maxQvalueOutputindex++;

                        // converting the maximum Q value output to a vector of binary digits
                        double[] desiredOutput = mnet.stann.int2BinaryVector(maxQvalueOutputindex);
                        // now training...
                        mnet.stann.backPropagate(levelinput, desiredOutput);
                        // updating the IO log 
                        if (mnet.IOLogLength == MAX_IO_LOG_LENGTH)
                        { // IO Log is full
                            // clearing the log and starting all over again
                            mnet.IOLogLength = 1;
                        }
                        else
                            mnet.IOLogLength++;
                        // updating the IO log entries
                        mnet.IOLog[mnet.IOLogLength - 1].input = inputindex;
                        mnet.IOLog[mnet.IOLogLength - 1].output = (int)theoutput;

                    }
                }
                else
                    theoutput = mnet.CurrentOutput;

            }

            return theoutput;
        }
        */


        

        public static double getOutput(MetaNode mnet, double[][] inputvec, int pass)
        {
            int i;
            double[] sigmoids;
            double theoutput;

            if (mnet.ChildrenNum == 0)
            {
                if (mnet.OutputPass != pass)
                {
                    mnet.OutputPass = pass;
                    // self training if the node has its ForceSelfTrain attribute set to true
                    if (mnet.ForcedSelfTrain)
                        // self training 
                        mnet.stann.selfTrain(inputvec[mnet.LeafIndex]);

                    // retrieving the sigmoids of the node
                    sigmoids = mnet.stann.sigmoidLayerOutputs(inputvec[mnet.LeafIndex], mnet.stann.LayerNum - 1);
                    // calculating the decimal equivalent to the ordered thresholded sigmoid outputs
                    theoutput = STANN.sigmoids2Int(sigmoids, mnet.stann.OutputNum);

                    mnet.PreviousOutput = mnet.CurrentOutput;
                    mnet.CurrentOutput = theoutput;
                }
                else
                    theoutput = mnet.CurrentOutput;

            }
            else
            {
                if (mnet.OutputPass != pass)
                {
                    mnet.OutputPass = pass;
                    double[] levelinput = new double[mnet.InputNum];

                    for (i = 0; i < mnet.InputNum; i++)
                        if (mnet.Children[i].NodeLevel >= mnet.NodeLevel) // requesting output from a higher (or equal ) level node
                            levelinput[i] = mnet.Children[i].PreviousOutput; // avoiding circular reference in recursion
                        else
                            levelinput[i] = getOutput(mnet.Children[i], inputvec, pass);

                    // self training if the aselfrain attribute is on
                    if (mnet.ForcedSelfTrain) mnet.stann.selfTrain(levelinput);
                    // retrieving sigmoids
                    sigmoids = mnet.stann.sigmoidLayerOutputs(levelinput, mnet.stann.LayerNum - 1);
                    
                    // calculating the decimal equivalent to the thresholded outputs
                    
                    theoutput = STANN.mapVector2Int(sigmoids, 2, mnet.stann.OutputNum);
                    // updating previous Input Vector and Previous Output properties of the metanode
                    int t;
                    mnet.PreviousInputVec = new double[mnet.InputNum];
                    for (t = 0; t < mnet.InputNum; t++)
                        mnet.PreviousInputVec[t] = levelinput[t];
                    

                    mnet.PreviousOutput = mnet.CurrentOutput;
                    mnet.CurrentOutput = theoutput;
                    // previous input vector and output updated with the new values

                    // Must now train the network!!!! (in case the qlearning property is on)
                    if (mnet.ForcedQLearning)
                    {
                        // mapping the input to the proper index in the reward table
                        int inputindex = STANN.mapVector2Int(levelinput, mnet.InputRange, mnet.InputNum);

                        // finding the output that corresponds to the maximum Qvaluefor the given input

                        QTableEntry maxQvalueEntry = QTableEntry.getMaxQValue(mnet.QTable, inputindex);


                        if (maxQvalueEntry != null)
                        {
                            // converting the maximum Q value output to a vector of binary digits
                            double[] desiredOutput = STANN.mapInt2VectorDouble(maxQvalueEntry.Output, 2, mnet.stann.OutputNum);
                            // now training...
                            mnet.stann.backPropagate(levelinput, desiredOutput);
                        }
                        // updating the IO log 
                        if (mnet.IOLogLength == MAX_IO_LOG_LENGTH)
                        { // IO Log is full
                            // clearing the log and starting all over again
                            mnet.IOLogLength = 1;
                        }
                        else
                            mnet.IOLogLength++;

                        // updating the IO log entries
                        mnet.IOLog[mnet.IOLogLength - 1].input = inputindex;
                        mnet.IOLog[mnet.IOLogLength - 1].output = (int)theoutput;
                    }

                    
                }
                else
                    theoutput = mnet.CurrentOutput;

            }

            return theoutput;
        }


        public static double getOutputNoUpdate(MetaNode mnet, double[][] inputvec, int pass)
        {
            int i;
            double[] sigmoids;
            double theoutput;

            if (mnet.ChildrenNum == 0)
            {
                if (mnet.OutputPass != pass)
                {
                    mnet.OutputPass = pass;
                    // self training if the node has its ForceSelfTrain attribute set to true
                    

                    // retrieving the sigmoids of the node
                    sigmoids = mnet.stann.sigmoidLayerOutputs(inputvec[mnet.LeafIndex], mnet.stann.LayerNum - 1);
                    // calculating the decimal equivalent to the ordered thresholded sigmoid outputs
                    theoutput = STANN.mapVector2Int(sigmoids, 2, mnet.stann.OutputNum);

                   
                }
                else
                    theoutput = mnet.NoUpdatePassOutput;

            }
            else
            {
                if (mnet.OutputPass != pass)
                {
                    mnet.OutputPass = pass;
                    double[] levelinput = new double[mnet.InputNum];

                    for (i = 0; i < mnet.InputNum; i++)
                        if (mnet.Children[i].NodeLevel >= mnet.NodeLevel) // requesting output from a higher (or equal ) level node
                            levelinput[i] = mnet.Children[i].PreviousOutput; // avoiding circular reference in recursion
                        else
                            levelinput[i] = getOutputNoUpdate (mnet.Children[i], inputvec, pass);

                    // retrieving sigmoids
                    sigmoids = mnet.stann.sigmoidLayerOutputs(levelinput, mnet.stann.LayerNum - 1);

                    // calculating the decimal equivalent to the thresholded outputs

                    theoutput = STANN.mapVector2Int(sigmoids, 2, mnet.stann.OutputNum);
                    

                }
                else
                    theoutput = mnet.CurrentOutput;

            }

            return theoutput;
        }


        public static double[] getNodeInputNoUpdate(MetaNode mnet, double[][] inputvec, int pass)
        {
            int i;
            double[] sigmoids;
            double[] theinput;

            if (mnet.ChildrenNum == 0)
                // the nodes input should be the input vector corresponding to the leaf
                theinput = inputvec[mnet.LeafIndex];
            else
            {
                theinput = new double[mnet.InputNum];

                for (i=0; i<mnet.InputNum; i++) 
                    theinput[i] = getOutputNoUpdate(mnet.Children[i], inputvec, pass);
            }

            return theinput;
        }

        // return the maximum reward in the Reward Table for a given input
       /* public double getMaximumQValue(int inputindex)
        {
            double maxQ = QTable[inputindex][0];
            int maxQindex = 0;

            int i;
            int possibleoutputs = (int)Math.Pow(2, stann.OutputNum);

            for (i = 0; i < possibleoutputs; i++)
                if (maxQ < QTable[inputindex][i])
                {
                    maxQ = QTable[inputindex][i];
                    maxQindex = i;
                }
            return QTable[inputindex][maxQindex];
        }*/



        // given the last input(previousInputVec) caused PreviousOutput(which led to a satisfactory result),
        //we may assign a reward to the new state that came up, thus backtracking and updating the rewards for
        // a finite number of steps (input-output) that superceeded this succesfull ending
        /*
        public void assignReward(double[] currentinputvec, double reward)
        {
            int previousinput = 0, currentinput = 0, i;
            for (i = 0; i < InputNum; i++) {
                previousinput += (int)(Math.Pow(InputRange, i) * PreviousInputVec[i]);
                currentinput += (int)(Math.Pow(InputRange, i) * currentinputvec[i]);
            }

            RewardTable[currentinput] = reward;
        
            // Updating the rewards in the Reward table using the log entry
            double currentStepReward = reward;
            int tempinput = currentinput;

            for (i = IOLogLength - 1; i >= 0; i--)
            {
                // updating the q-value for the input-output log entry (in three lines)
                QTable[IOLog[i].input][IOLog[i].output] = (1 - Qalpha) * QTable[IOLog[i].input][IOLog[i].output];
                QTable[IOLog[i].input][IOLog[i].output] += Qalpha * RewardTable[tempinput];
                QTable[IOLog[i].input][IOLog[i].output] += Qalpha * Qgamma * getMaximumQValue(tempinput);
                // Q-value of the entry updated
                tempinput = IOLog[i].input;
            }

            // clearing the IO Log to avoid re-assigning Q values on the same chain of actions when a new reward appears
            clearIOLog();
        }
        */

        public void assignReward(double[] currentinputvec, double reward)
        {
            int i;
            // mapping previous and current input vectors to integers
            int previousinput = STANN.mapVector2Int(PreviousInputVec, InputRange, InputNum);
            int currentinput = STANN.mapVector2Int(currentinputvec, InputRange, InputNum);

            // adding a reward entry for the current input
            
            RewardTableEntry.updateRewardEntry(ref RewardTable, currentinput, reward);
            
            
            // Updating the Q - values in the Q table using the existing log entries
            double currentStepReward = reward;
            int tempinput = currentinput;

            for (i = IOLogLength - 1; i >= 0; i--)
            {
                // retrieving the Q -table entry for the current input in the log
                QTableEntry entry = QTableEntry.findQTableEntry(QTable, IOLog[i].input, IOLog[i].output);
                if (entry == null)
                {
                    QTableEntry.assignQValue(ref QTable, IOLog[i].input, IOLog[i].output, 0);
                    entry = QTableEntry.findQTableEntry(QTable, IOLog[i].input, IOLog[i].output);
                }
                else
                    entry.Frequency++;
                // The Non-Deterministic MDP coefficient
                double NDPCoefficient = 1.0 / (1.0 + 1.0*entry.Frequency);

                double qvalue = entry.QValue;
                // updating the q-value for the input-output log entry (in three lines)
                qvalue = NDPCoefficient*(1 - Qalpha) * qvalue;
                qvalue += NDPCoefficient * Qalpha * RewardTableEntry.getReward(RewardTable, tempinput);
                qvalue += NDPCoefficient * Qalpha * Qgamma * QTableEntry.getMaxQValue(QTable, tempinput).QValue;

                entry.QValue = qvalue;
                // Q-value of the entry updated
                tempinput = IOLog[i].input;
            }

            
        }


        public void clearIOLog()
        {
            IOLogLength = 0;
        }


         
    }
}
