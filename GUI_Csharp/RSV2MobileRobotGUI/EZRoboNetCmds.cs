
    // list of possible commands to/from the robot 
    public enum RobotCmd
    {
        rc_ManualCommand = 0,
        rc_CommandDone,
        rc_SendSensors,
        rc_SendCMUCAMAbstractionData,
        rc_CMUCAMAbstractionData,
        rc_SensorData,
        rc_FireSonarArray,
        rc_SonarFired
    };


    // types of robots
    public enum t_Robot
    {
        t_RobosapienV2,
        t_HandmadeCart,
        t_Station
    };


    // A command Message. Should be contained in the payload of the packet
    struct CommandMsg
    {
        public t_Robot robot;
        public RobotCmd Cmd;
        public ushort ParamsLength;
        public byte[] CmdParams;
    };

    // Manual Command Parameters (contains the low and high byte of the command)
    struct ManualCmdparams
    {
        public ushort Command;

    };

    // Robosapien V2 sensor parameters (30 bytes lenth)
    struct RSV2Sensorparams
    {
        // ON/OFF sensors
        public byte P_Left_Foot_Front_Bumper;
        public byte P_Left_Foot_Rear_Bumper;
        public byte P_Right_Foot_Front_Bumper;
        public byte P_Right_Foot_Rear_Bumper;
        public byte P_Left_Hand_Bumper;
        public byte P_Right_Hand_Bumper;
        public byte P_Left_Hand_Pickup;
        public byte P_Right_Hand_Pickup;
        // Analog sensors
        public ushort P_Camera_HSD;
        public ushort P_Camera_RST;
        public ushort P_Camera_RDY;
        public ushort P_Camera_SK;

        public ushort P_Left_Shoulder_Pot;
        public ushort P_Right_Shoulder_Pot;

        public ushort P_Left_Mic;
        public ushort P_Right_Mic;

        public ushort P_Left_Head_IR;
        public ushort P_Center_Head_IR;
        public ushort P_Right_Head_IR;
    };

    // CMUCAM scanned line parameters in RGB
    struct CMUCAMLineparams
    {

        public byte[] RedChannel;
        public byte[] GreenChannel;
        public byte[] BlueChannel;
    };


    class RobotMessenger
    {

        // convert Manual Command Paramaters to bytes
        public static byte[] convertManualCmdparams2Bytes(ManualCmdparams parms)
        {
            byte[] rawparms = new byte[2];
            //command low byte
            rawparms[0] = (byte)(parms.Command & 0x0FF);
            // command high byte
            rawparms[1] = (byte)((parms.Command >> 8) & 0x0FF);

            return rawparms;
        }


        // convert RobosapienV2 sensor parameters to bytes
        public static byte[] convertRSV2Sensorparams2Bytes(RSV2Sensorparams parms)
        {
            byte[] rawparms = new byte[30];

            rawparms[0] = parms.P_Left_Foot_Front_Bumper;
            rawparms[1] = parms.P_Left_Foot_Rear_Bumper;
            rawparms[2] = parms.P_Right_Foot_Front_Bumper;
            rawparms[3] = parms.P_Right_Foot_Rear_Bumper;
            rawparms[4] = parms.P_Left_Hand_Bumper;
            rawparms[5] = parms.P_Right_Hand_Bumper;
            rawparms[6] = parms.P_Left_Hand_Pickup;
            rawparms[7] = parms.P_Right_Hand_Pickup;
            // Analog sensors
            rawparms[8] = (byte)(parms.P_Camera_HSD & 0x0FF);
            rawparms[9] = (byte)((parms.P_Camera_HSD >> 8) & 0x0FF);

            rawparms[10] = (byte)(parms.P_Camera_RST & 0x0FF);
            rawparms[11] = (byte)((parms.P_Camera_RST >> 8) & 0x0FF);

            rawparms[12] = (byte)(parms.P_Camera_RDY & 0x0FF);
            rawparms[13] = (byte)((parms.P_Camera_RDY >> 8) & 0x0FF);

            rawparms[14] = (byte)(parms.P_Camera_SK & 0x0FF);
            rawparms[15] = (byte)((parms.P_Camera_SK >> 8) & 0x0FF);

            rawparms[16] = (byte)(parms.P_Left_Shoulder_Pot & 0x0FF);
            rawparms[17] = (byte)((parms.P_Left_Shoulder_Pot >> 8) & 0x0FF);

            rawparms[18] = (byte)(parms.P_Right_Shoulder_Pot & 0x0FF);
            rawparms[19] = (byte)((parms.P_Right_Shoulder_Pot >> 8) & 0x0FF);

            rawparms[20] = (byte)(parms.P_Left_Mic & 0x0FF);
            rawparms[21] = (byte)((parms.P_Left_Mic >> 8) & 0x0FF);

            rawparms[22] = (byte)(parms.P_Right_Mic & 0x0FF);
            rawparms[23] = (byte)((parms.P_Right_Mic >> 8) & 0x0FF);

            rawparms[24] = (byte)(parms.P_Left_Head_IR & 0x0FF);
            rawparms[25] = (byte)((parms.P_Left_Head_IR >> 8) & 0x0FF);

            rawparms[26] = (byte)(parms.P_Center_Head_IR & 0x0FF);
            rawparms[27] = (byte)((parms.P_Center_Head_IR >> 8) & 0x0FF);

            rawparms[28] = (byte)(parms.P_Right_Head_IR & 0x0FF);
            rawparms[29] = (byte)((parms.P_Right_Head_IR >> 8) & 0x0FF);

            return rawparms;
        }

        

        

        // convert bytes to an Robosapien V2 Swensor readings structure
        public static RSV2Sensorparams convertBytes2RSV2Sensorparams(byte[] rawparms)
        {
            RSV2Sensorparams parms = new RSV2Sensorparams();

            parms.P_Left_Foot_Front_Bumper = rawparms[0];
            parms.P_Left_Foot_Rear_Bumper = rawparms[1];
            parms.P_Right_Foot_Front_Bumper = rawparms[2];
            parms.P_Right_Foot_Rear_Bumper = rawparms[3];
            parms.P_Left_Hand_Bumper = rawparms[4];
            parms.P_Right_Hand_Bumper = rawparms[5];
            parms.P_Left_Hand_Pickup = rawparms[6];
            parms.P_Right_Hand_Pickup = rawparms[7];
            // Analog sensors
            parms.P_Camera_HSD = (ushort)(rawparms[8] + rawparms[9] * 256);

            parms.P_Camera_RST = (ushort)(rawparms[10] + rawparms[11] * 256);

            parms.P_Camera_RDY = (ushort)(rawparms[12] + rawparms[13] * 256);

            parms.P_Camera_SK = (ushort)(rawparms[14] + rawparms[15] * 256);

            parms.P_Left_Shoulder_Pot = (ushort)(rawparms[16] + rawparms[17] * 256);

            parms.P_Right_Shoulder_Pot = (ushort)(rawparms[18] + rawparms[19] * 256);

            parms.P_Left_Mic = (ushort)(rawparms[20] + rawparms[21] * 256);

            parms.P_Right_Mic = (ushort)(rawparms[22] + rawparms[23] * 256);

            parms.P_Left_Head_IR = (ushort)(rawparms[24] + rawparms[25] * 256);

            parms.P_Center_Head_IR = (ushort)(rawparms[26] + rawparms[27] * 256);

            parms.P_Right_Head_IR = (ushort)(rawparms[28] + rawparms[29] * 256);

            return parms;
        }


        // convert manual Command Parameters (ability id)
        public static ManualCmdparams convertBytes2ManualCmdparams(byte[] rawparms)
        {
            ManualCmdparams parms = new ManualCmdparams();
            parms.Command = (ushort)(rawparms[0] + rawparms[1] * 256);

            return parms;
        }

        // convert bytes to a command message structure
        public static CommandMsg convertBytes2CommandMsg(byte[] rawmsg)
        {
            CommandMsg msg = new CommandMsg();
            msg.robot = (t_Robot)rawmsg[0];
            msg.Cmd = (RobotCmd)rawmsg[1];
            msg.ParamsLength = (ushort)(rawmsg[2] + rawmsg[3] * 256);

            int i;
            msg.CmdParams = new byte[msg.ParamsLength];
            for (i = 0; i < msg.ParamsLength; i++)
                msg.CmdParams[i] = rawmsg[4 + i];

            return msg;
        }
    }


