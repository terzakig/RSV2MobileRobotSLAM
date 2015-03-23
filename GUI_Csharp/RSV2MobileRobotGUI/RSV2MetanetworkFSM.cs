using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class RSV2MetanetworkFSM
    {
        // the RSV2 entity to execute the Cognitive array 
        public RobosapienV2 Robosapien;
        // FSM's internal state
        public int state;

        // state constants
        public const int stIdle = 0;
        public const int stSensorDataTransmission = 1;
        public const int stFrameTransmission = 2;
        public const int stAbilityExecuting = 3;
        
        public int LastUsedAbility;
        public double[][] LastInputVecs;
        public double[] TopNodeInput;

        //constructor
        public RSV2MetanetworkFSM(RobosapienV2 robo) {
            Robosapien = robo;

            state = stIdle;
        }

        public void executionStep()
        {
            // changing state
            state = stSensorDataTransmission;
            // requesting sensor data
            Robosapien.requestSensorData();
        }


        public void transitionAction(ref System.Windows.Forms.Panel panel,
                                     System.Windows.Forms.TextBox[] texts, ref int pass)
        {
            switch (state)
            {
                case stIdle: // nothing
                    break;
                case stSensorDataTransmission:
                    if (Robosapien.flagSensorDataAcquired)
                    {
                        Robosapien.fillSensorTexts(texts);

                        state = stFrameTransmission;
                        Robosapien.retrieveAbstraction();
                    }
                    break;
                case stFrameTransmission:
                    if (Robosapien.flagAbstractionReady)
                    {
                        Robosapien.drawAbstraction(panel);
                        state = stAbilityExecuting;
                        // running the cognitive array now
                        double[][] inputVecs = Robosapien.makeInputVector();
                        pass++;
                        int output = (int)MetaNode.getOutput(Robosapien.CogTop, inputVecs, pass);
                        
                        Robosapien.useAbility((t_RSV2Ability)output);
                       
                    }
                    break;
                case stAbilityExecuting:
                    if (Robosapien.flagAbilityDone)
                    {
                        Robosapien.flagAbilityDone = false;
                        state = stIdle;
                    }
                    break;

            }
        }




    }
}
