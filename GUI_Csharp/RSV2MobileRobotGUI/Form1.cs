using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace RobosapienRFControl
{
    public partial class Form1 : Form
    {
  
        public int rsv2pass, cartpass;
        public static Random rnd;
        public Boolean SensorDataRequested;
        public Boolean CameraFrameRequested;
        

        EZRoboNetDevice server;
        RobosapienV2 robosapien;
        LMMobileRobot cart;
        CartTrainingFSM cartTrainingFSM;
        RSV2TrainingFSM rsv2TrainingFSM;
        CartMetaNetworkFSM cartMNExecutingFSM;
        RSV2MetanetworkFSM rsv2MNExecutingFSM;
        MappingFSM mappingFSM;
        RSV2BrowsingFSM rsv2BrowsingFSM;

        public Boolean ModeCartTraining, ModeRSV2Training;
        
        public Boolean ModeCartExecutingMetaNetwork;
        public Boolean ModeRsv2ExecutingMetaNetwork;

        public Boolean ModeCartMapping;
        public Boolean ModeRSV2Browsing;


        public Form1()
        {
           

            ModeCartTraining = ModeRSV2Training = false;
            ModeCartExecutingMetaNetwork = false;
            ModeCartMapping = ModeRSV2Browsing = false;
            ModeRsv2ExecutingMetaNetwork = false;
            
            
            InitializeComponent();
            server = new EZRoboNetDevice(1, EZRoboNetDevice.t_Node_Station);

            rnd = new Random((int)DateTime.Now.Ticks);
            SensorDataRequested = CameraFrameRequested = false;
            rsv2pass = cartpass = 0;

            robosapien = new RobosapienV2(server, 2, rnd);
            cart = new LMMobileRobot(server, 3, rnd);

            cartTrainingFSM = new CartTrainingFSM(cart);
            cartMNExecutingFSM = new CartMetaNetworkFSM(cart);
            rsv2MNExecutingFSM = new RSV2MetanetworkFSM(robosapien);
            
            // creating mapping and browsing FSMs
            mappingFSM = new MappingFSM(cart, 4, MappingFSM.orSOUTH, 1, 1);
            rsv2BrowsingFSM = new RSV2BrowsingFSM(mappingFSM, robosapien, 0, 1, RSV2BrowsingFSM.orSOUTH);


            rsv2TrainingFSM = new RSV2TrainingFSM(robosapien);

            
            timer1.Start();
            timer2.Start();

            comboBox1.SelectedIndex = 2;
            comboBox2.SelectedIndex = 2;
        }

        private void button1_Click(object sender, EventArgs e)
        {
            
            robosapien.useAbility(t_RSV2Ability.abSTOP);
        }

        

        
        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void button2_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abWALK_FORWARD;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
   
        }

        private void button8_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abWALK_BACKWARD;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button6_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abTURN_RIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);

        }

        private void button7_Click(object sender, EventArgs e)
        {

            t_RSV2Ability ab = t_RSV2Ability.abTURN_LEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button3_Click(object sender, EventArgs e)
        {

            t_RSV2Ability ab = t_RSV2Ability.abWALK_FORWARDRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button4_Click(object sender, EventArgs e)
        {
            
            t_RSV2Ability ab = t_RSV2Ability.abWALK_FORWARDLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button10_Click(object sender, EventArgs e)
        {

            t_RSV2Ability ab = t_RSV2Ability.abWALK_BACKWARDRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button11_Click(object sender, EventArgs e)
        {

            t_RSV2Ability ab = t_RSV2Ability.abWALK_BACKWARDLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button5_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBULLDOZER_FORWARD;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button9_Click(object sender, EventArgs e)
        {

            t_RSV2Ability ab = t_RSV2Ability.abBULLDOZER_BACKWARD;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button12_Click(object sender, EventArgs e)
        {

            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_THROW;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button13_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_LOW_PICKUP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button14_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_HIGH_PICKUP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button15_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_GRAB;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button16_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_GIVE;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button17_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_ROLL;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button18_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBURP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button19_Click(object sender, EventArgs e)
        {
 
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_UP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button20_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_DOWN;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button21_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_RIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button22_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_LEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button23_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_UPRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button24_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_UPLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button25_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_DOWNRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button26_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_DOWNLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button27_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_CHOP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button28_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_KICK;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button39_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_PUSH;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button29_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_THROW;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button40_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_LOW_PICKUP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button41_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_HIGH_PICKUP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button30_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_GRAB;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button31_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_GIVE;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button32_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_ROLL;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button42_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_UP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button33_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_DOWN;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button43_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_LEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button34_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_RIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button44_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_UPRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button35_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_UPLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button36_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_DOWNRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button37_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_DOWNLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button38_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_KICK;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button45_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_CHOP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button46_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_PUSH;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button55_Click(object sender, EventArgs e)
        {
            robosapien.useAbility(t_RSV2Ability.abLEFT_ARM_DROP);
            t_RSV2Ability ab = t_RSV2Ability.abLEFT_ARM_DROP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button47_Click(object sender, EventArgs e)
        {
            robosapien.useAbility(t_RSV2Ability.abRIGHT_ARM_DROP);
            t_RSV2Ability ab = t_RSV2Ability.abRIGHT_ARM_DROP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button49_Click(object sender, EventArgs e)
        {
            robosapien.useAbility(t_RSV2Ability.abGET_UP);
            t_RSV2Ability ab = t_RSV2Ability.abGET_UP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button54_Click(object sender, EventArgs e)
        {
            robosapien.useAbility(t_RSV2Ability.abBOTHARMS_UP);
            t_RSV2Ability ab = t_RSV2Ability.abBOTHARMS_UP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button50_Click(object sender, EventArgs e)
        {
            robosapien.useAbility(t_RSV2Ability.abBOTHARMS_DOWN);
            t_RSV2Ability ab = t_RSV2Ability.abBOTHARMS_DOWN;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button51_Click(object sender, EventArgs e)
        {
            robosapien.useAbility(t_RSV2Ability.abBOTHARMS_LEFT);
            t_RSV2Ability ab = t_RSV2Ability.abBOTHARMS_LEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button53_Click(object sender, EventArgs e)
        {
            robosapien.useAbility(t_RSV2Ability.abBOTHARMS_RIGHT);
            t_RSV2Ability ab = t_RSV2Ability.abBOTHARMS_RIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button56_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBOTHARMS_DOWNLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button52_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBOTHARMS_DOWNRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button65_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBOTHARMS_UPLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button57_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBOTHARMS_UPRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button58_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abWAIST_FORWARD;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button59_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abWAIST_BACKWARD;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button60_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abWAIST_LEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button61_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abWAIST_RIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button62_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abWALK_FORWARDRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button63_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abWALK_FORWARDLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button64_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abWALK_BACKWARDRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button66_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abWALK_BACKWARDLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button67_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abHEAD_UP;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button68_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abHEAD_DOWN;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button69_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abHEAD_RIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button70_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abHEAD_LEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button71_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abHEAD_UPRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button72_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abHEAD_UPLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button73_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abHEAD_DOWNRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button74_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abHEAD_DOWNLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button75_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBODY_FORWARD;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button76_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBODY_BACKWARD;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button77_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBODY_LEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button78_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBODY_RIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button79_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBODY_FORWARDRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button80_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBODY_FORWARDLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);

        }

        private void button81_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBODY_BACKWARDRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button82_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abBODY_BACKWARDLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button83_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLAUGH;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button84_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abINSULT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button85_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abPLAN;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button86_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abSPARE_CHANGE;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button87_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abHEY_BABY;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button88_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abROAR;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button89_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abDIODE;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button90_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abFETCH;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button91_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abDANGER;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button92_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abCALM_DOWN;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button93_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abHUG;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button94_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abOOPS;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button95_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abHIGH_FIVE;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button48_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLIE_DOWN;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button96_Click(object sender, EventArgs e) {
        
            t_RSV2Ability ab = t_RSV2Ability.abLEAN_FORWARD;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button97_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEAN_BACKWARD;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button98_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEAN_LEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button99_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEAN_RIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button100_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEAN_FORWARDRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button101_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEAN_FORWARDLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button102_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEAN_BACKWARDRIGHT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button104_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abLEAN_BACKWARDLEFT;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button103_Click(object sender, EventArgs e)
        {
            t_RSV2Ability ab = t_RSV2Ability.abFREE_ROAM;
            if (ModeRSV2Training) rsv2TrainingFSM.trainingStep((int)ab, ref cartpass);
            else robosapien.useAbility(ab);
        }

        private void button105_Click(object sender, EventArgs e)
        {
            robosapien.requestSensorData();
        }


        private void printCartSensorData()
        {
            textBoxt0.Text = Convert.ToString(cart.SonarArray[0]);
            textBoxt1.Text = Convert.ToString(cart.SonarArray[1]);
            textBoxt2.Text = Convert.ToString(cart.SonarArray[2]);
            textBoxt3.Text = Convert.ToString(cart.SonarArray[3]);
            textBoxt4.Text = Convert.ToString(cart.SonarArray[4]);
            textBoxt5.Text = Convert.ToString(cart.SonarArray[5]);
            textBoxt6.Text = Convert.ToString(cart.SonarArray[6]);
            textBoxt7.Text = Convert.ToString(cart.SonarArray[7]);
        }


        private void timer1_Tick(object sender, EventArgs e)
        {

            

            System.Windows.Forms.TextBox[] texts = new TextBox[29];


            // 2. assigning sensor readings
            texts[0] = textBox2;
            texts[1] = textBox3;
            texts[2] = textBox4;
            texts[3] = textBox5;
            texts[4] = textBox6;
            texts[5] = textBox7;
            texts[6] = textBox8;
            texts[7] = textBox9;

            texts[8] = textBox10;
            texts[9] = textBox11;

            // analog sensors
            texts[10] = textBox12;
            texts[11] = textBox13;
            texts[12] = textBox14;
            texts[13] = textBox15;

            texts[14] = textBox16;
            texts[15] = textBox17;

            texts[16] = textBox18;
            texts[17] = textBox19;

            texts[18] = textBox20;
            texts[19] = textBox21;
            texts[20] = textBox22;

            // the mic and bumper sensor triggers
            texts[21] = textBox23;
            texts[22] = textBox24;
            texts[23] = textBox25;
            texts[24] = textBox26;
            texts[25] = textBox27;
            texts[26] = textBox28;
            texts[27] = textBox29;
            texts[28] = textBox30;
            
            
            if (ModeRSV2Browsing) rsv2BrowsingFSM.browsingStep(ref texts);
            else
            if (ModeRSV2Training)
            {

                rsv2TrainingFSM.transitionAction(panel2, texts);
            }
            else if (ModeRsv2ExecutingMetaNetwork)
            {
                if (rsv2MNExecutingFSM.state == CartMetaNetworkFSM.stIdle)
                    rsv2MNExecutingFSM.executionStep();
                else
                    rsv2MNExecutingFSM.transitionAction(ref panel2, texts, ref rsv2pass);
            }
            else
            {
                if (robosapien.flagAbstractionReady)
                    robosapien.drawAbstraction(panel2);
                if (robosapien.flagSensorDataAcquired)
                    robosapien.fillSensorTexts(texts);
            }
            
            

        }

        private void button106_Click(object sender, EventArgs e)
        {
            robosapien.retrieveAbstraction();
        }

        private void button107_Click(object sender, EventArgs e)
        {
            
        }

        private void button108_Click(object sender, EventArgs e)
        {
            
        }

        private void button109_Click(object sender, EventArgs e)
        {
            ModeRSV2Training = false;            
        }

        private void button110_Click(object sender, EventArgs e)
        {
            
        }

        private void button113_Click(object sender, EventArgs e)
        {
            rsv2MNExecutingFSM = new RSV2MetanetworkFSM(robosapien);
            ModeRSV2Training = false;
            ModeRsv2ExecutingMetaNetwork = true;
        }

        private void button112_Click(object sender, EventArgs e)
        {
            ModeRsv2ExecutingMetaNetwork = false;
        }

        private void button111_Click(object sender, EventArgs e)
        {
            
        }

        private void button114_Click(object sender, EventArgs e)
        {
            rsv2TrainingFSM = new RSV2TrainingFSM(robosapien);
            ModeRsv2ExecutingMetaNetwork = false;
            ModeRSV2Training = true;
            
        }

        private void button115_Click(object sender, EventArgs e)
        {
            robosapien.useAbility(t_RSV2Ability.abDANCE_DEMO);
        }

        private void button116_Click(object sender, EventArgs e)
        {
            t_CartAbility ab = t_CartAbility.abGO_FORWARD;
            if (ModeCartTraining) cartTrainingFSM.trainingStep((int)ab, ref cartpass);
            else cart.useAbility(ab);
        }

        private void button117_Click(object sender, EventArgs e)
        {
            t_CartAbility ab =  t_CartAbility.abTURN_RIGHT90;
            if (ModeCartTraining) cartTrainingFSM.trainingStep((int)ab, ref cartpass);
            else cart.useAbility(ab);
        }

        private void button118_Click(object sender, EventArgs e)
        {
            t_CartAbility ab =  t_CartAbility.abGO_BACKWARD;
            if (ModeCartTraining) cartTrainingFSM.trainingStep((int)ab, ref cartpass);
            else cart.useAbility(ab);
        }

        private void button119_Click(object sender, EventArgs e)
        {
            t_CartAbility ab =  t_CartAbility.abTURN_LEFT90;
            if (ModeCartTraining) cartTrainingFSM.trainingStep((int)ab, ref cartpass);
            else cart.useAbility(ab);
        }

        private void button120_Click(object sender, EventArgs e)
        {
            t_CartAbility ab = t_CartAbility.abTURN_LEFT45;
            if (ModeCartTraining) cartTrainingFSM.trainingStep((int)ab, ref cartpass);
            else cart.useAbility(ab);
        }

        private void button121_Click(object sender, EventArgs e)
        {
            t_CartAbility ab = t_CartAbility.abTURN_RIGHT45;
            if (ModeCartTraining) cartTrainingFSM.trainingStep((int)ab, ref cartpass);
            else cart.useAbility(ab);
        }

        private void button123_Click(object sender, EventArgs e)
        {
            cart.fireSonarArray();
        }

        private void button122_Click(object sender, EventArgs e)
        {
            cart.requestSensorData();
        }

        private void button127_Click(object sender, EventArgs e)
        {
            
            t_CartAbility ab = t_CartAbility.abTURN_LEFT10;
            if (ModeCartTraining) cartTrainingFSM.trainingStep((int)ab, ref cartpass);
            else cart.useAbility(ab);
        }

        private void button126_Click(object sender, EventArgs e)
        {
            
            t_CartAbility ab = t_CartAbility.abTURN_RIGHT10;
            if (ModeCartTraining) cartTrainingFSM.trainingStep((int)ab, ref cartpass);
            else cart.useAbility(ab);
        }

        private void button124_Click(object sender, EventArgs e)
        {
            t_CartAbility ab = t_CartAbility.abGO_FORWARD10;
            if (ModeCartTraining) cartTrainingFSM.trainingStep((int)ab, ref cartpass);
            else cart.useAbility(ab);

        }

        private void button125_Click(object sender, EventArgs e)
        {
            
            t_CartAbility ab = t_CartAbility.abGO_BACKWARD10;
            if (ModeCartTraining) cartTrainingFSM.trainingStep((int)ab, ref cartpass);
            else cart.useAbility(ab);
        }

        private void button128_Click(object sender, EventArgs e)
        {
            cart.retrieveAbstraction();
        }

        private void timer2_Tick(object sender, EventArgs e)
        {
            System.Windows.Forms.TextBox[] texts = new TextBox[8];

            texts[0] = textBoxt0;
            texts[1] = textBoxt1;
            texts[2] = textBoxt2;
            texts[3] = textBoxt3;
            texts[4] = textBoxt4;
            texts[5] = textBoxt5;
            texts[6] = textBoxt6;
            texts[7] = textBoxt7;

            if (ModeCartMapping)
            {
                mappingFSM.mappingStep(ref texts);
                // updating displacement and angular error estimates
                textBox31.Text = Convert.ToString(mappingFSM.DisplacementX);
                textBox32.Text = Convert.ToString(mappingFSM.DisplacementY);
                textBox33.Text = Convert.ToString(mappingFSM.AngularError);
            }
            else if (ModeCartTraining)
            {

                cartTrainingFSM.transitionAction(panel3, texts);
            }
            else if (ModeCartExecutingMetaNetwork)
            {
                if (cartMNExecutingFSM.state == CartMetaNetworkFSM.stIdle)
                    cartMNExecutingFSM.executionStep();
                else
                    cartMNExecutingFSM.transitionAction(ref panel3, texts, ref cartpass);
            }
            else
            {
                if (cart.flagAbstractionReady)
                    cart.drawAbstraction(panel3);
                if (cart.flagSensorDataAcquired)
                    printCartSensorData();
            }
        }

        private void button129_Click(object sender, EventArgs e)
        {
            cartTrainingFSM = new CartTrainingFSM(cart);
            ModeCartTraining = true;
            ModeCartMapping = false;
        }

        private void button130_Click(object sender, EventArgs e)
        {
            ModeCartTraining = false;
        }

        private void button131_Click(object sender, EventArgs e)
        {
            cartMNExecutingFSM = new CartMetaNetworkFSM(cart);
            ModeCartTraining = false;
            ModeCartMapping = false;
            ModeCartExecutingMetaNetwork = true;
        }

        private void button132_Click(object sender, EventArgs e)
        {
            ModeCartExecutingMetaNetwork = false;
        }

        private void button133_Click(object sender, EventArgs e)
        {
            int ipos = (int)numericIpos.Value;
            int jpos = (int)numericJpos.Value;
            int orient = comboBox1.SelectedIndex;
            int dim = (int)numericDim.Value;
            
            mappingFSM.resetFSM(ipos, jpos, orient, dim);

            ModeCartMapping = true;

        }

        private void button134_Click(object sender, EventArgs e)
        {
            ModeCartMapping = false;
            ModeCartTraining = false;
            ModeCartExecutingMetaNetwork = false;
        }

        private void button135_Click(object sender, EventArgs e)
        {
            
            double[][] A = new double[][] {new double[] {1, 0},
                                           new double[] {0, 1}
                                            };
            double[] B = new double[] { 0, 0};
            double[] V0 = new double[] { 2, 2};
            double[] W0 = new double[] { 0, 0};
            double[][] H = new double[][] {new double[] {1, 0},
                                           new double[] {0, 1}
                                            };
            double[][] Q = new double[][] { new double[] {5, 0},
                                            new double[] {0, 5}
                                          };
            double[][] R = new double[][] { new double[] {2, 0},
                                            new double[] {0, 2}
                                          };
            double[] Z = new double[] {5, 5};
            double[] Z1 = new double[] {6, 6};
            double[] U = new double[] {0, 0};
            KalmanFilter kalman = new KalmanFilter(2, A, B, W0, H, V0, Q, R, Z);
            double[] est = kalman.getNewEstimate(Z1, U);
            double[] Z2 = new double[] { 5, 5 };
            kalman.getNewEstimate(Z2, U);

            double[][] matrix = new double[][] { new double[] {1, 0, 0},
                                                 new double[] {2, 1, 0},
                                                 new double[] {1, 0, 1}};
            double d = KalmanFilter.getDeterminant(matrix, 3);

            double[][] inv = KalmanFilter.inverseMatrix(matrix, 3);
            double[] Gxy = MappingFSM.getGradient(cart, MappingFSM.orEAST, MappingFSM.orNORTH);
            double acos = MappingFSM.getAngleOffset(cart, MappingFSM.orEAST, MappingFSM.orNORTH);
            double angle = 180*Math.Acos(acos)/Math.PI;
            
            int test = 1;
        }

        private void button108_Click_1(object sender, EventArgs e)
        {
            robosapien.useAbility(t_RSV2Ability.abMIC_SENSORS_ONOFF);
        }

        private void button110_Click_1(object sender, EventArgs e)
        {
            robosapien.useAbility(t_RSV2Ability.abRESET);
        }

        private void button111_Click_1(object sender, EventArgs e)
        {
            int rsv2Ipos = (int)numericRsv2Ipos.Value;
            int rsv2Jpos = (int)numericRsv2Jpos.Value;
            int rsv2Orientation = comboBox2.SelectedIndex;

            rsv2BrowsingFSM.resetBrowsingFSM(rsv2Ipos, rsv2Jpos, rsv2Orientation);

            ModeRSV2Training = false;
            ModeRsv2ExecutingMetaNetwork = false;
            ModeRSV2Browsing = true;

        }

        private void button136_Click(object sender, EventArgs e)
        {
            ModeRSV2Browsing = false;
            ModeRsv2ExecutingMetaNetwork = false;
            ModeRSV2Training = false;
        }

        private void button107_Click_1(object sender, EventArgs e)
        {
            MappingFSM.drawMap(mappingFSM, panel4);
        }

        private void numericDim_ValueChanged(object sender, EventArgs e)
        {
            numericIpos.Maximum = numericDim.Value - 1;
            numericJpos.Maximum = numericDim.Value - 1;
            numericRsv2Ipos.Maximum = numericDim.Value - 1;
            numericRsv2Jpos.Maximum = numericDim.Value - 1;

        }


    }
}
