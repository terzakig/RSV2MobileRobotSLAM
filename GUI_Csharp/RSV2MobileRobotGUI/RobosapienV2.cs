using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;

namespace RobosapienRFControl
{
    enum t_RSV2Ability
    {
        abWALK_FORWARD = 0,
        abWALK_BACKWARD,         // walk BACKWARD
        abTURN_RIGHT,            // turn RIGHT
        abTURN_LEFT,             // turn LEFT
        abWALK_FORWARDRIGHT,    // walk FORWARD- RIGHT
        abWALK_FORWARDLEFT,     // walk FORWARD-LEFT
        abWALK_BACKWARDRIGHT,   // walk BACKWARD-RIGHT
        abWALK_BACKWARDLEFT,    // walk BACKWARD-LEFT

        // Buldozer walk
        abBULLDOZER_FORWARD,      // Bulldozer FORWARD
        abBULLDOZER_BACKWARD,     // Bulldozer BACKWARD
        // walk/turn abilities ends

        // Upper Body+Head motion
        abLEAN_BACKWARD,       //  LEAN FORWARD
        abLEAN_FORWARD,      //  LEAN BACKWARD
        abLEAN_RIGHT,         //  LEAN RIGHT
        abLEAN_LEFT,          //  LEAN LEFT
        abLEAN_BACKWARDRIGHT,  // LEAN FORWARD-RIGHT
        abLEAN_BACKWARDLEFT,   // LEAN FORWARD-LEFT
        abLEAN_FORWARDRIGHT, // LEAN BACKWARD-RIGHT
        abLEAN_FORWARDLEFT,  // LEAN BACKWARD-LEFT
        // Upper Body+Head motion commands ends


        abFREE_ROAM,     // 
        abSTOP,          // STOP

        // right arm motion commands
        abRIGHT_ARM_THROW,        // Right Arm THROW
        abRIGHT_ARM_LOW_PICKUP,   // Right arm LOW PICKUP
        abRIGHT_ARM_HIGH_PICKUP,  // Right Arm HIGH PICKUP
        abRIGHT_ARM_GRAB,         // Right Arm GRAB (from the floor)
        abRIGHT_ARM_GIVE,         // Right Arm GIVE
        abRIGHT_ARM_ROLL,         // Right Arm ROLL
        // directional right arm motion
        abRIGHT_ARM_UP,           // Right Arm UP
        abRIGHT_ARM_DOWN,         // Right Arm DOWN
        abRIGHT_ARM_RIGHT,        // Right Arm RIGHT
        abRIGHT_ARM_LEFT,         // Right Arm LEFT
        abRIGHT_ARM_UPRIGHT,      // Right Arm UP AND RIGHT
        abRIGHT_ARM_UPLEFT,       // Right Arm UP and LEFT
        abRIGHT_ARM_DOWNRIGHT,    // Right Arm DOWN and RIGHT
        abRIGHT_ARM_DOWNLEFT,     // Right Arm DOWN and LEFT
        // right arm motion commands ends

        // left arm motion commands
        abLEFT_ARM_THROW,         // Left Arm THROW
        abLEFT_ARM_LOW_PICKUP,    // Left Arm LOW PICKUP
        abLEFT_ARM_HIGH_PICKUP,   // Left Arm HIGH PICKUP
        abLEFT_ARM_GRAB,          // Left Arm GRAB
        abLEFT_ARM_GIVE,          // Left Arm GIVE
        abLEFT_ARM_ROLL,          // Left Arm ROLL
        // directional left arm motion
        abLEFT_ARM_UP,            // Left Arm UP
        abLEFT_ARM_DOWN,          // Left Arm DOWN
        abLEFT_ARM_RIGHT,         // Left Arm RIGHT
        abLEFT_ARM_LEFT,          // Left Arm LEFT
        abLEFT_ARM_UPRIGHT,       // Left Arm UP and RIGHT
        abLEFT_ARM_UPLEFT,        // Left Arm UP and LEFT
        abLEFT_ARM_DOWNRIGHT,     // Left Arm DOWN and RIGHT
        abLEFT_ARM_DOWNLEFT,      // Left Arm DOWN and LEFT
  
        abLIE_DOWN,         // LIE DOWN->SIT UP->LIE DOWN->STAND UP
        abGET_UP,           // GET UP

        // special moves 
        abRIGHT_KICK,       // RIGHT KICK 
        abRIGHT_PUSH,       // RIGHT PUSH
        abRIGHT_CHOP,       // RIGHT CHOP
        abLEFT_CHOP,        // LEFT CHOP
        abLEFT_PUSH,        // LEFT PUSH
        abLEFT_KICK,        // LEFT KICK
        abOOPS,             // OOPS (whatever that means)

        // Hip and Waist Tilt Motion Commands
        abWAIST_FORWARD,        // Waist-Hip tilt FORWARD
        abWAIST_BACKWARD,       // Waist-Hip tilt BCKWARDS
        abWAIST_RIGHT,          // Waist-Hip tilt RIGHT
        abWAIST_LEFT,           // Waist-Hip tilt LEFT
        abWAIST_FORWARDRIGHT,   // Waist-Hip tilt FORWARD and RIGHT
        abWAIST_FORWARDLEFT,   // Waist-Hip tilt FORWARD and LEFT
        abWAIST_BACKWARDRIGHT, // Waist-Hip tilt BACKWARD and RIGHT
        abWAIST_BACKWARDLEFT,  // Waist-Hip tilt BEACKWARDS and LEFT
        // Hip and Waist tilt motion ends

        // Misc motion
        abHIGH_FIVE,         // HIGH FIVE

        // Both Arms Motion Commands
        abBOTHARMS_UP,         // Both Arms UP
        abBOTHARMS_DOWN,       // Both Arms DOWN
        abBOTHARMS_RIGHT,      // Both Arms RIGHT
        abBOTHARMS_LEFT,       // Both Arms LEFT
        abBOTHARMS_UPRIGHT,    // Both Arms UP and RIGHT
        abBOTHARMS_UPLEFT,     // Both Arms UP and LEFT
        abBOTHARMS_DOWNRIGHT,  // Both Arms DOWN and RIGHT
        abBOTHARMS_DOWNLEFT,   // Both Arms DOWN and LEFT
        // Both Arms motion comands ends

        // Behavioral expressions etc. and other moves
        abLAUGH,            // LAUGH
        abINSULT,           // INSULT
        abRIGHT_ARM_DROP,   // Right Arm DROP
        abLEFT_ARM_DROP,    // Left Arm DROP
        abPLAN,             // PLAN (very curious about this...)
        abSPARE_CHANGE,     // SPARE CHANGE (again very curious...)
        abHEY_BABY,         // HEY BABY (!)
        // more expressions...
        abROAR,             // ROARRRRRR
        abDIODE,            // DIODE (? - should try that out...)
        abFETCH,            // FETCH (? again curious...)
        abDANGER,           // DANGER (? - curious...)
        abCALM_DOWN,        // CALM DOWN
        abHUG,              // HUG!
        abBURP,             // BURP 
        // behavior and misc move commands ends 

        // Head only Motion Commands
        abHEAD_UP,         // HEAD UP
        abHEAD_DOWN,       // HEAD DOWN
        abHEAD_RIGHT,      // HEAD RIGHT
        abHEAD_LEFT,       // HEAD LEFT
        abHEAD_UPRIGHT,    // HEAD UP-RIGHT 
        abHEAD_UPLEFT,     // HEAD UP-LEFT
        abHEAD_DOWNRIGHT,  // HEAD DOWN-RIGHT
        abHEAD_DOWNLEFT,   // HEAD DOWN-LEFT
        // Head Motion Comands ends

        // Upper Body ONLY motion commands
        abBODY_BACKWARD,    // Body lean BACKWARD
        abBODY_FORWARD,     // Body lean FORWARD 
        abBODY_RIGHT,        // Body lean RIGHT
        abBODY_LEFT,         // Body lean LEFT
        abBODY_BACKWARDRIGHT,  // Body lean BACKWARD-RIGHT
        abBODY_BACKWARDLEFT,  // Body lean BACKWARD-LEFT
        abBODY_FORWARDRIGHT,  // Body lean FORWARD-RIGHT
        abBODY_FORWARDLEFT,   // Body lean FORWARD-LEFT
        abDANCE_DEMO,         // Dance Demo (pretty cool)
        abMIC_SENSORS_ONOFF,  // Mics ON/OFF
        abRESET,              // RESET
        ABILITIES_NUM
    };

    class RobosapienV2 : RoboEntity
    {

        public const int ABSTRACTION_FRAME_WIDTH = 8;
        public const int ABSTRACTION_FRAME_HEIGHT = 8;

        // **********************    RobosapienV2 sensors *****************************
        public byte sensorLeft_Foot_Front_Bumper;
        public byte sensorLeft_Foot_Rear_Bumper;
        public byte sensorRight_Foot_Front_Bumper;
        public byte sensorRight_Foot_Rear_Bumper;
        public byte sensorLeft_Hand_Bumper;
        public byte sensorRight_Hand_Bumper;
        public byte sensorLeft_Hand_Pickup;
        public byte sensorRight_Hand_Pickup;
        // the wrist encoders can have values from 0 - 3.
        public byte sensorLeft_Wrist_Encoder;
        public byte sensorRight_Wrist_Encoder;

        // analog sensors
        public ushort sensorCamera_HSD;
        public ushort sensorCamera_RST;
        public ushort sensorCamera_RDY;
        public ushort sensorCamera_SK;

        public ushort sensorLeft_Shoulder_Pot;
        public ushort sensorRight_Shoulder_Pot;

        public ushort sensorLeft_Mic;
        public ushort sensorRight_Mic;

        public ushort sensorLeft_Head_IR;
        public ushort sensorCenter_Head_IR;
        public ushort sensorRight_Head_IR;

        // the following members contain recent triggers of the mic and the bumper sensors
        public byte sensorLeft_Mic_Triggered;
        public byte sensorRight_Mic_Triggered;
        public byte sensorLeft_Foot_Rear_Triggered;
        public byte sensorLeft_Foot_Front_Triggered;
        public byte sensorRight_Foot_Rear_Triggered;
        public byte sensorRight_Foot_Front_Triggered;
        public byte sensorLeft_Hand_Triggered;
        public byte sensorRight_Hand_Triggered;
        // ******************************* Sensor List ends ************************

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

        // ability execution flags
        public Boolean flagAbilityExecuting, flagAbilityDone;

        // sensor readings acquisition flag
        public Boolean flagSensorDataAcquired;


        // Cognitive array MetaNodes
        public MetaNode CogTop;
        public MetaNode CogLimbs;
        
        public MetaNode[] CogCamFrameLines;


        int state;



        public RobosapienV2(EZRoboNetDevice netdevice, byte avatarnodeid, Random randgen)
            : base(netdevice, avatarnodeid, randgen)
        {


            flagAbstractionDataReady = false;
            flagAbstractionReady = false;
            AbstractionBytes = 0;

            // creating the cognitive array
            CogTop = createRSV2CognitiveArray(ref CogLimbs, ref CogCamFrameLines);


            state = stIdle;

            RawAbstraction = new byte[ABSTRACTION_FRAME_HEIGHT * ABSTRACTION_FRAME_WIDTH * 3];



        }

        // this method downloads a command to the avatar
        // based on the available abilities
        public void useAbility(t_RSV2Ability ability)
        {
            CommandMsg msg = new CommandMsg();
            msg.robot = t_Robot.t_Station;
            msg.Cmd = RobotCmd.rc_ManualCommand;
            msg.ParamsLength = 1; // one byte - ability index
            msg.CmdParams = new byte[msg.ParamsLength];
            msg.CmdParams[0] = (byte)ability;
            // raising flag
            flagAbilityDone = false;
            flagAbilityExecuting = true;
            
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
                    case RobotCmd.rc_CommandDone: // must release the pending ability here
                        //clearing execution flag
                        flagAbilityExecuting = false;
                        // raising execution completion flag
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
                        break;
                    case RobotCmd.rc_CMUCAMAbstractionData: // retrieving a CMUCAM frame
                        acquireCMUCAMAbstractionData(msg);
                        break;
                }


            }

            // 2. Cheking state and flags
            if (flagAbstractionDataReady)
                requestCMUCAMAbstractionData();
        }


        // this method fill text boex with sensor readings
        public void fillSensorTexts(System.Windows.Forms.TextBox[] texts)
        {
            // 2. assigning sensor readings
            texts[0].Text = sensorLeft_Foot_Front_Bumper == 0 ? "0" : "1";
            texts[1].Text = sensorLeft_Foot_Rear_Bumper == 0 ? "0" : "1";
            texts[2].Text = sensorRight_Foot_Front_Bumper == 0 ? "0" : "1";
            texts[3].Text = sensorRight_Foot_Rear_Bumper == 0 ? "0" : "1";
            texts[4].Text = sensorLeft_Hand_Bumper == 0 ? "0" : "1";
            texts[5].Text = sensorRight_Hand_Bumper == 0 ? "0" : "1";
            texts[6].Text = sensorLeft_Hand_Pickup == 0 ? "0" : "1";
            texts[7].Text = sensorRight_Hand_Pickup == 0 ? "0" : "1";

            texts[8].Text = Convert.ToString(sensorLeft_Wrist_Encoder);
            texts[9].Text = Convert.ToString(sensorRight_Wrist_Encoder);

            // analog sensors
            texts[10].Text = Convert.ToString(sensorCamera_HSD);
            texts[11].Text = Convert.ToString(sensorCamera_RST);
            texts[12].Text = Convert.ToString(sensorCamera_RDY);
            texts[13].Text = Convert.ToString(sensorCamera_SK);

            texts[14].Text = Convert.ToString(sensorLeft_Shoulder_Pot);
            texts[15].Text = Convert.ToString(sensorRight_Shoulder_Pot);

            texts[16].Text = Convert.ToString(sensorLeft_Mic);
            texts[17].Text = Convert.ToString(sensorRight_Mic);

            texts[18].Text = Convert.ToString(sensorLeft_Head_IR);
            texts[19].Text = Convert.ToString(sensorCenter_Head_IR);
            texts[20].Text = Convert.ToString(sensorRight_Head_IR);

            // the mic and bumper sensor triggers
            texts[21].Text = Convert.ToString(sensorLeft_Mic_Triggered);
            texts[22].Text = Convert.ToString(sensorRight_Mic_Triggered);
            texts[23].Text = Convert.ToString(sensorLeft_Foot_Rear_Triggered);
            texts[24].Text = Convert.ToString(sensorLeft_Foot_Front_Triggered);
            texts[25].Text = Convert.ToString(sensorRight_Foot_Rear_Triggered);
            texts[26].Text = Convert.ToString(sensorRight_Foot_Front_Triggered);
            texts[27].Text = Convert.ToString(sensorLeft_Hand_Triggered);
            texts[28].Text = Convert.ToString(sensorRight_Hand_Triggered);

            // clearing the flag
            flagSensorDataAcquired = false;
        }

        // copying sensor data from the frame into the entity's sensor member variables
        public void acquireSensorData(CommandMsg msg)
        {
            byte[] sensorData = msg.CmdParams;
            // assigning sensor readings
            sensorLeft_Foot_Front_Bumper = sensorData[0];
            sensorLeft_Foot_Rear_Bumper = sensorData[1];
            sensorRight_Foot_Front_Bumper = sensorData[2];
            sensorRight_Foot_Rear_Bumper = sensorData[3];
            sensorLeft_Hand_Bumper = sensorData[4];
            sensorRight_Hand_Bumper = sensorData[5];
            sensorLeft_Hand_Pickup = sensorData[6];
            sensorRight_Hand_Pickup = sensorData[7];

            sensorLeft_Wrist_Encoder = sensorData[8];
            sensorRight_Wrist_Encoder = sensorData[9];

            // analog sensors
            sensorCamera_HSD = (ushort)(sensorData[10] + sensorData[11] * 256);
            sensorCamera_RST = (ushort)(sensorData[12] + sensorData[13] * 256);
            sensorCamera_RDY = (ushort)(sensorData[14] + sensorData[15] * 256);
            sensorCamera_SK = (ushort)(sensorData[16] + sensorData[17] * 256);

            sensorLeft_Shoulder_Pot = (ushort)(sensorData[18] + sensorData[19] * 256);
            sensorRight_Shoulder_Pot = (ushort)(sensorData[20] + sensorData[21] * 256);

            sensorLeft_Mic = (ushort)(sensorData[22] + sensorData[23] * 256);
            sensorRight_Mic = (ushort)(sensorData[24] + sensorData[25] * 256);

            sensorLeft_Head_IR = (ushort)(sensorData[26] + sensorData[27] * 256);
            sensorCenter_Head_IR = (ushort)(sensorData[28] + sensorData[29] * 256);
            sensorRight_Head_IR = (ushort)(sensorData[30] + sensorData[31] * 256);

            // the mic and bumper sensor triggers
            sensorLeft_Mic_Triggered = sensorData[32];
            sensorRight_Mic_Triggered = sensorData[33];
            sensorLeft_Foot_Rear_Triggered = sensorData[34];
            sensorLeft_Foot_Front_Triggered = sensorData[35];
            sensorRight_Foot_Rear_Triggered = sensorData[36];
            sensorRight_Foot_Front_Triggered = sensorData[37];
            sensorLeft_Hand_Triggered = sensorData[38];
            sensorRight_Hand_Triggered = sensorData[39];

            flagSensorDataAcquired = true;
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
                flagAbstractionDataReady = false; // rpeventing a new chuynk request

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


        // This method creates a Metanework cognitive array
        public static MetaNode createRSV2CognitiveArray(ref MetaNode limbs, ref MetaNode[] camlines)
        {

            MetaNode[] children;

            // creating Leaves
            // 1. Creating 1st level (0) STANN (1 MetaNode) for Right/Left Arm/Foot Sensors, including the pickups (Limbs)
            MetaNode Limbs = MetaNode.createTreeLeaf(12,   // number of inputs for each leaf 
                                                      4,   // range of input (0-3)
                                                      3,   // 3 STANN Layers
                                                      20,  // Number of Neurons in each layer (except the output layer)
                                                       4,   // Number of binary outputs of the node
                                                     0.5,  // STANN Threshold
                                                     0.6,  // STANN Learning Rate (Quick)
                                                     null, // parents should be linked here
                                                     RandGen,   // random number generator
                                                     0          // leaf index is 0
                                                     );
            

            // 4. Creating 1st level STANN MetaNode for the Camera Input (8x8 frame abstraction)
            MetaNode[] CamFrameLines = MetaNode.createTreeLeaves(8, // 8 metanodes, 1 for each line
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
            children = new MetaNode[9];
            children[0] = Limbs;
            int i;
            for (i=0; i<8; i++)
                children[1+i] = CamFrameLines[i];

            // ATTENTION! this node will using its output as input, therefore should include
            // itself in the children array following creation

            MetaNode Top = MetaNode.createHigherLevelNode(children, //  children array
                                                          2,        // 2 children
                                                          3,        // 3 STANN Layers
                                                          30,       // 30 neurons per layer
                                                          6,        // 6 binary outputs (may have to reduce/increase it)
                                                          0.5,      // STANN threshold
                                                          0.7,      // fast learning rate
                                                          null,     // NO Parents. we're at the top
                                                          0,        // 0 number of parents
                                                          RandGen,  // Random Number Generator
                                                          false,    // node is NOT self trained
                                                          false,     // Q-Learning disabled (for now...)
                                                          0.3,      // Q -learning a param is 0.3
                                                          0.6,      // Q - learning γ param is 0.6
                                                          1         // Level 2
                                                          );
            // *** ADDING self into the children list
            Top.addChild(Top);


            // *************** Updating the parents entries of the MetaNodes in a bottom-up fashion *************

            


            limbs = Limbs;
            camlines = CamFrameLines;

            return Top;

        }


        // the following method assembles a sensory input vector for the cognitive array
        public double[][] makeInputVector()
        {
            // creating an input vector for all leaves
            // 12 inputs for limbs, 5 for IRs and Mics, 64 for the 8 line frame metanodes
            double[][] inputVector = new double[9][];

            // **************** Adding Limb sensor data *******************

            inputVector[0] = new double[12];

            inputVector[0][0] = sensorLeft_Foot_Front_Triggered;
            inputVector[0][1] = sensorLeft_Foot_Rear_Triggered;

            inputVector[0][2] = sensorRight_Foot_Front_Triggered;
            inputVector[0][3] = sensorRight_Foot_Rear_Triggered;

            inputVector[0][4] = sensorLeft_Hand_Triggered;

            inputVector[0][5] = sensorRight_Hand_Triggered;

            inputVector[0][6] = sensorLeft_Hand_Pickup;
            inputVector[0][7] = sensorRight_Hand_Pickup;

            inputVector[0][8] = sensorLeft_Wrist_Encoder;
            inputVector[0][9] = sensorRight_Wrist_Encoder;

            // normalizing the shoulder pots to a 0-3 scale
            inputVector[0][10] = (sensorLeft_Shoulder_Pot * 4) / 1023;
            inputVector[0][11] = (sensorRight_Shoulder_Pot * 4) / 1023;

            // ******************** Done with Limbs sensors ********************


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


        // the following method applies the sensor readings to the metanetwork and returns the result
        public int runCognitiveArray(int pass)
        {
            double[][] inputvector = makeInputVector();

            //double[][] inputvector = makeRandomInputVector();

            return (int)MetaNode.getOutput(CogTop, inputvector, pass);





        }


        // the following method assembles a sensory input vector for the cognitive array
        public double[][] makeRandomInputVector()
        {
            // creating an input vector for all leaves
            // 12 inputs for limbs, 5 for IRs and Mics, 64 for the 8 line frame metanodes
            double[][] inputVector = new double[10][];

            // **************** Adding Limb sensor data *******************

            inputVector[0] = new double[12];

            inputVector[0][0] = RandGen.Next(2);
            inputVector[0][1] = RandGen.Next(2);

            inputVector[0][2] = RandGen.Next(2);
            inputVector[0][3] = RandGen.Next(2);

            inputVector[0][4] = RandGen.Next(2);

            inputVector[0][5] = RandGen.Next(2);

            inputVector[0][6] = RandGen.Next(2);
            inputVector[0][7] = RandGen.Next(2);

            inputVector[0][8] = RandGen.Next(3);
            inputVector[0][9] = RandGen.Next(3);

            // normalizing the shoulder pots to a 0-3 scale
            inputVector[0][10] = (RandGen.Next(1023) * 4) / 1023;
            inputVector[0][11] = (RandGen.Next(1023) * 4) / 1023;

            // ******************** Done with Limbs sensors ********************

            // ******************************** Adding mics and IRs **************
            inputVector[1] = new double[5];

            inputVector[1][0] = RandGen.Next(1023) *4 / 1023;
            inputVector[1][1] = RandGen.Next(1023) * 4 / 1023;
            inputVector[1][2] = RandGen.Next(1023) * 4 / 1023;
            inputVector[1][3] = RandGen.Next(1023) * 4 / 1023;
            inputVector[1][4] = RandGen.Next(1023) * 4 / 1023;

            // ********************** Adding the thresholded frame lines **********

            // threholding first...
            byte[] threshImage = new byte[ABSTRACTION_FRAME_HEIGHT * ABSTRACTION_FRAME_WIDTH];
           
            int i;
            for (i = 0; i < ABSTRACTION_FRAME_WIDTH * ABSTRACTION_FRAME_HEIGHT; i++)
            {
                 
                threshImage[i] = (RandGen.Next(101) < 50) ? (byte)0 : (byte)1;
            }
            // Line 1
            inputVector[2] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[2][i] = threshImage[i];

            // Line 2
            inputVector[3] = new double[8];

            i = 0;
            for (i = 0; i < 8; i++)
                inputVector[3][i] = threshImage[8 + i];
            // Line 3
            inputVector[4] = new double[8];

            i = 0;
            for (i = 0; i < 8; i++)
                inputVector[4][i] = threshImage[16 + i];
            i = 0;
            // Line 4
            inputVector[5] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[5][i] = threshImage[24 + i];

            // Line 5
            inputVector[6] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[6][i] = threshImage[32 + i];


            // Line 6
            inputVector[7] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[7][i] = threshImage[40 + i];

            // Line 7
            inputVector[8] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[8][i] = threshImage[48 + i];


            // Line 8
            inputVector[9] = new double[8];

            for (i = 0; i < 8; i++)
                inputVector[9][i] = threshImage[56 + i];

            return inputVector;
        }


    }
}
