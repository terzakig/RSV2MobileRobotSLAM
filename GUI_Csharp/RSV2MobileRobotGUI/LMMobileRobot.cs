using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;

namespace RobosapienRFControl
{
    enum t_CartAbility
    {
        abGO_FORWARD = 0,
        abGO_BACKWARD,
        abGO_FORWARD10,
        abGO_BACKWARD10,
        abTURN_LEFT45,
        abTURN_LEFT90,
        abTURN_RIGHT45,
        abTURN_RIGHT90,
        abTURN_LEFT10,
        abTURN_RIGHT10,
        ABILITIES_NUM
    };

    

    class LMMobileRobot : RoboEntity
    {
        // Cognitive array MetaNodes
        public MetaNode CogTop; // top metanode
        public MetaNode CogSonarNode; // Sonar Transducers leave node
        public MetaNode[] CogCamLinesNodes; // 8 camera lines nodes
        
        public const int ABSTRACTION_FRAME_WIDTH = 8;
        public const int ABSTRACTION_FRAME_HEIGHT = 8;

        // **********************    Mobile Robot Sonar sensors  *****************************
        public ushort[] SonarArray;
        public ushort[] PrevSonarArray; // used in gradient calculations

        // ability execution flags
        public Boolean flagAbilityExecuting, flagAbilityDone;

        // sensor readings acquisition flag
        public Boolean flagSensorDataAcquired;

        // sonar firing flags
        public Boolean flagSonarArrayFiring, flagSonarArrayFiringDone;

        // state constants
        public const int stIdle = 0;
        public const int stWaitingAbstractionData = 1;

        // class members
        public Boolean flagFrameReady;
        public Boolean flagAbstractionReady;
        public Boolean flagAbstractionDataReady;
        CMUCAMFrame FrameAbstraction;
        public byte[] RawAbstraction;
        public int AbstractionBytes;

        


        int state;


        // constructor
        public LMMobileRobot(EZRoboNetDevice netdevice, byte avatarnodeid, Random randgen)
            : base(netdevice, avatarnodeid, randgen)
        {
            flagAbstractionDataReady = false;
            flagAbstractionReady = false;

            flagSensorDataAcquired = false;

            AbstractionBytes = 0;

            // creating the sonar array
            SonarArray = new ushort[8];
            PrevSonarArray = new ushort[8];

            // creating the cognitive array
            CogTop = createLMCartCognitiveArray(ref CogSonarNode, ref CogCamLinesNodes);


            state = stIdle;

            flagSonarArrayFiring = flagSonarArrayFiringDone = false;
            flagAbilityDone = flagAbilityExecuting = false;

            RawAbstraction = new byte[ABSTRACTION_FRAME_HEIGHT * ABSTRACTION_FRAME_WIDTH * 3];

        }

        
        // this method downloads a command to the avatar
        // based on the available abilities
        public void useAbility(t_CartAbility ability)
        {
            CommandMsg msg = new CommandMsg();
            msg.robot = t_Robot.t_Station;
            msg.Cmd = RobotCmd.rc_ManualCommand;
            msg.ParamsLength = 1; // one byte - ability index
            msg.CmdParams = new byte[msg.ParamsLength];
            msg.CmdParams[0] = (byte)ability;
            
            flagAbilityExecuting = true;
            flagAbilityDone = false;

            this.sendCommandMessage(msg);
        }


        

        // the following method initiates an abstracted frame retrieval
        public void retrieveAbstraction()
        {
            AbstractionBytes = 0;
            flagAbstractionReady = false;

            requestCMUCAMAbstractionData();
        }

        // this method requests a CMUCAM Frame
        public void requestCMUCAMAbstractionData()
        {
            CommandMsg msg = new CommandMsg();
            msg.robot = t_Robot.t_Station;
            msg.Cmd = RobotCmd.rc_SendCMUCAMAbstractionData;
            msg.ParamsLength = 1;
            msg.CmdParams = new byte[msg.ParamsLength];
            msg.CmdParams[0] = 0;
            flagAbstractionDataReady = false;

            this.sendCommandMessage(msg);
        }

        // this method requests sensory data from the robosapien V2 avatar
        public void requestSensorData()
        {
            CommandMsg msg = new CommandMsg();
            msg.robot = t_Robot.t_Station;
            msg.Cmd = RobotCmd.rc_SendSensors;
            msg.ParamsLength = 1;
            msg.CmdParams = new byte[msg.ParamsLength];
            msg.CmdParams[0] = 0;
            flagSensorDataAcquired = false;
            this.sendCommandMessage(msg);
        }

        // the following method commands the mobile robot to fire the sonar array
        public void fireSonarArray() {
            CommandMsg msg = new CommandMsg();
            msg.robot = t_Robot.t_Station;
            msg.Cmd = RobotCmd.rc_FireSonarArray;
            msg.ParamsLength = 1; // one byte - just zero
            msg.CmdParams = new byte[msg.ParamsLength];
            msg.CmdParams[0] = 0;
            // raising flags
            flagSonarArrayFiring = true;
            flagSonarArrayFiringDone = false;
            
            this.sendCommandMessage(msg);
        }


        public override void handleMessages(object sender, EventArgs e)
        {
            int packetsavailable = packetsAvailable();
            EZPacket pack;
            // 1. checking pakcest available and acting

            if (packetsavailable > 0)
            {
                pack = getNextAvailablePacket();

                // unwrapping the packet
                CommandMsg msg = RobotMessenger.convertBytes2CommandMsg(pack.Message);
                // handling the message
                switch (msg.Cmd)
                {
                    case RobotCmd.rc_CommandDone: // handling the flags
                        flagAbilityExecuting = false;
                        flagAbilityDone = true;
                        break;
                    case RobotCmd.rc_ManualCommand: // Nothing...
                        break;
                    case RobotCmd.rc_SendCMUCAMAbstractionData: // nothing...
                        break;

                    case RobotCmd.rc_SendSensors: // nothing
                        break;
                    case RobotCmd.rc_SensorData: // retrieving sensory data from the avatar
                        acquireSensorData(msg);
                        flagSensorDataAcquired = true;
                        break;
                    case RobotCmd.rc_CMUCAMAbstractionData: // retrieving a CMUCAM frame
                        acquireCMUCAMAbstractionData(msg);
                        break;
                    case RobotCmd.rc_SonarFired: // handling flags
                        flagSonarArrayFiring = false;
                        flagSonarArrayFiringDone = true;
                        break;
                    
                }


            }

            // 2. Cheking state and flags
            if (flagAbstractionDataReady)
                requestCMUCAMAbstractionData();
        }

        // this method copies sensor data from the frame into the entity's sensor member variables
        public void acquireSensorData(CommandMsg msg)
        {
            byte[] sensorData = msg.CmdParams;
            // movingcurrent sensor readings to previous sensor readings array
            // assigning sensor readings
            int i;
            for (i = 0; i < 8; i++)
            {
                PrevSonarArray[i] = SonarArray[i];
                SonarArray[i] = (ushort)(sensorData[2 * i] + sensorData[2 * i + 1] * 256);
            }
        }


        public void acquireCMUCAMAbstractionData(CommandMsg msg)
        {
            int parmslength = msg.ParamsLength;
            byte[] parms = msg.CmdParams;
            int i, j;
            int startindex = AbstractionBytes;
            for (i = 0; i < parmslength; i++)
            {
                RawAbstraction[startindex + i] = parms[i];
                AbstractionBytes++;
            }
            flagAbstractionDataReady = true;

            if (AbstractionBytes == ABSTRACTION_FRAME_HEIGHT * ABSTRACTION_FRAME_WIDTH * 3)
            {
                FrameAbstraction = new CMUCAMFrame();
                FrameAbstraction.RedChannel = new byte[ABSTRACTION_FRAME_HEIGHT][];
                FrameAbstraction.GreenChannel = new byte[ABSTRACTION_FRAME_HEIGHT][];
                FrameAbstraction.BlueChannel = new byte[ABSTRACTION_FRAME_HEIGHT][];

                for (i = 0; i < ABSTRACTION_FRAME_HEIGHT; i++)
                {
                    FrameAbstraction.RedChannel[i] = new byte[ABSTRACTION_FRAME_WIDTH];
                    FrameAbstraction.GreenChannel[i] = new byte[ABSTRACTION_FRAME_WIDTH];
                    FrameAbstraction.BlueChannel[i] = new byte[ABSTRACTION_FRAME_WIDTH];

                    for (j = 0; j < ABSTRACTION_FRAME_WIDTH; j++)
                    {
                        FrameAbstraction.RedChannel[i][j] = RawAbstraction[(i * ABSTRACTION_FRAME_WIDTH + j) * 3];
                        FrameAbstraction.GreenChannel[i][j] = RawAbstraction[(i * ABSTRACTION_FRAME_WIDTH + j) * 3 + 1];
                        FrameAbstraction.BlueChannel[i][j] = RawAbstraction[(i * ABSTRACTION_FRAME_WIDTH + j) * 3 + 2];
                    }
                }
                flagAbstractionReady = true;
                flagAbstractionDataReady = false; // preventing a new chunk request

            }



        }

        // draw the 16x16 abstraction
        public void drawAbstraction(System.Windows.Forms.Panel panel)
        {

            int AbstractedFrameWidth = panel.Width;
            int AbstractedFrameHeight = panel.Height;
            Graphics gr = panel.CreateGraphics();
            gr.Clear(Color.White);
            int line, col, i, j;
            int resolutionX = AbstractedFrameWidth / ABSTRACTION_FRAME_WIDTH;
            int resolutionY = AbstractedFrameHeight / ABSTRACTION_FRAME_HEIGHT;
            for (line = 0; line < ABSTRACTION_FRAME_HEIGHT; line++)
                for (i = 0; i < resolutionY; i++)
                    for (col = 0; col < ABSTRACTION_FRAME_WIDTH; col++)
                        for (j = 0; j < resolutionX; j++)
                        {
                            System.Drawing.Pen pen = new Pen(Color.FromArgb(RawAbstraction[(line * ABSTRACTION_FRAME_WIDTH + col) * 3],
                                                                            RawAbstraction[(line * ABSTRACTION_FRAME_WIDTH + col) * 3 + 1],
                                                                            RawAbstraction[(line * ABSTRACTION_FRAME_WIDTH + col) * 3 + 2]
                                                                            )
                                                                            );
                            int y = line * resolutionY + i;
                            int x = col * resolutionX + j;

                            gr.DrawLine(pen, x, y, x + 1, y + 1);
                        }
            flagAbstractionReady = false;
        }


        // fill 8 text boxes with the sonar readings
        public void fillSonarTextBoxes(System.Windows.Forms.TextBox[] texts)
        {
            int i;
            for (i = 0; i < 8; i++)
                texts[i].Text = Convert.ToString(SonarArray[i]);

            // clearing the flag
            flagSensorDataAcquired = false;
        }


        public double[][] makeInputVector() {
            // creating teh input vectors for all leaves
            // 1 input vectorfor the sonar transducers, 8 input vectors for each frame line (9 metanodes)
            double[][] inputVector = new double[9][];

            // **************** transducer data *******************
            // all readings are divided by 10

            inputVector[0] = new double[8];

            inputVector[0][0] = (double)((SonarArray[0] > 30 ? 3 : SonarArray[0])/10);
            inputVector[0][1] = (double)((SonarArray[1] > 30 ? 3 : SonarArray[1]) / 10);

            inputVector[0][2] = (double)((SonarArray[2] > 30 ? 3 : SonarArray[2]) / 10);
            inputVector[0][3] = (double)((SonarArray[3] > 30 ? 3 : SonarArray[3]) / 10);

            inputVector[0][4] = (double)((SonarArray[4] > 30 ? 3 : SonarArray[4]) / 10);

            inputVector[0][5] = (double)((SonarArray[5] > 30 ? 3 : SonarArray[5]) / 10);

            inputVector[0][6] = (double)((SonarArray[6] > 30 ? 3 : SonarArray[6]) / 10);
            inputVector[0][7] = (double)((SonarArray[7] > 30 ? 3 : SonarArray[7]) / 10);

            
            // ******************** Done with Sonar Array  ********************

          
            // ********************** Adding the thresholded frame lines **********

            // threholding first...
            byte[] threshImage = new byte[ABSTRACTION_FRAME_HEIGHT * ABSTRACTION_FRAME_WIDTH];
            byte threshold = 80;
            int i;
            for (i = 0; i < ABSTRACTION_FRAME_WIDTH * ABSTRACTION_FRAME_HEIGHT; i++)
            {
                double bwpixel = 0.3 * RawAbstraction[3 * i] + 0.59 * RawAbstraction[i * 3 + 1] + 0.11 * RawAbstraction[3 * i + 2];
                threshImage[i] = (bwpixel < threshold) ? (byte)0 : (byte)1;
            }
            // Line 1
            inputVector[1] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[1][i] = threshImage[i];

            // Line 2
            inputVector[2] = new double[8];

            i = 0;
            for (i = 0; i < 8; i++)
                inputVector[2][i] = threshImage[8 + i];
            // Line 3
            inputVector[3] = new double[8];

            i = 0;
            for (i = 0; i < 8; i++)
                inputVector[3][i] = threshImage[16 + i];
            i = 0;
            // Line 4
            inputVector[4] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[4][i] = threshImage[24 + i];

            // Line 5
            inputVector[5] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[5][i] = threshImage[32 + i];


            // Line 6
            inputVector[6] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[6][i] = threshImage[40 + i];

            // Line 7
            inputVector[7] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[7][i] = threshImage[48 + i];


            // Line 8
            inputVector[8] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[8][i] = threshImage[56 + i];

            return inputVector;
        }


         // This method creates a Metanework cognitive array
        public static MetaNode createLMCartCognitiveArray(ref MetaNode sonarnode, ref MetaNode[] camlinesnodes)
        {
            MetaNode[] children;

            // creating Leaves
            // 1. Creating 1st level (0) STANN (1 MetaNode) for Sonar Sensors
            MetaNode TransducersNode = MetaNode.createTreeLeaf( 8,   // number of inputs 
                                                      4,   // range of input (0-3)
                                                      3,   // 3 STANN Layers
                                                      20,  // Number of Neurons in each layer (except the output layer)
                                                       5,   // Number of binary outputs of the node
                                                     0.5,  // STANN Threshold
                                                     0.6,  // STANN Learning Rate (Quick)
                                                     null, // parents should be linked here
                                                     RandGen,   // random number generator
                                                     0          // leaf index is 0
                                                     );
            

            // 4. Creating 1st level STANN MetaNode for the Camera Input (8x8 frame abstraction)
            MetaNode[] CamFrameLinesNodes = MetaNode.createTreeLeaves(8, // 8 metanodes, 1 for each line
                                                                 8,  //  8 line inputs
                                                                 2, // thresholded input range (0 black, 1 -white)
                                                                 3,  // 3 STANN Layers
                                                                20,  // 20 neurons per layer (except output)
                                                                 3,  // 3 STANN binary ouputs
                                                               0.5, // STANN threshold
                                                               0.5, // STANN Learning Rate (Quick)
                                                              null, // Parents are null for now...
                                                            RandGen, // Random Number Generator
                                                            2        // the starting index for these leaves is 2
                                                            );

            // Creating TOP node (although if things go nice, we may create a level before the top node)
            children = new MetaNode[10];
            children[0] = TransducersNode;
            int i;
            for (i = 0; i < 8; i++)
                children[1 + i] = CamFrameLinesNodes[i];

            // ATTENTION! this node will using its output as input, therefore should include
            // itself in the children array following creation

            MetaNode Top = MetaNode.createHigherLevelNode(children, //  children array
                                                          3,        // 3 children
                                                          3,        // 3 STANN Layers
                                                          25,       // 25 neurons per layer
                                                          4,        // 4 binary outputs 
                                                          0.5,      // STANN threshold
                                                          0.5,      // fast learning rate
                                                          null,     // NO Parents. we're at the top
                                                          0,        // 0 number of parents
                                                          RandGen,  // Random Number Generator
                                                          false,    // node is NOT self trained
                                                          true,     // Q-Learning enabled
                                                          0.3,      // Q -learning a param is 0.3
                                                          0.6,      // Q - learning γ param is 0.6
                                                          1         // Level 2
                                                          );
            // *** ADDING self into the children list
            Top.addChild(Top);




            sonarnode = TransducersNode;
            camlinesnodes = CamFrameLinesNodes;

            return Top;

        }

    }
}
