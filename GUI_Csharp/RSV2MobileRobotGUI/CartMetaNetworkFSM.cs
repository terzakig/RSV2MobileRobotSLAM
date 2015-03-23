using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class CartMetaNetworkFSM
    {

        // the Mobile robot to execute the Cognitive array 
        public LMMobileRobot Cart;
        // FSM's internal state
        public int state;

        // state constants
        public const int stIdle = 0;
        public const int stSonarFiring = 1;
        public const int stSonarDataTransmission = 2;
        public const int stFrameTransmission = 3;
        public const int stAbilityExecuting = 4;
        
        public int LastUsedAbility;
        public double[][] LastInputVecs;
        public double[] TopNodeInput;

        //Training FSM constructor
        public CartMetaNetworkFSM(LMMobileRobot cart) {
            Cart = cart;

            state = stIdle;
        }


        public void executionStep()
        {
            // changing state
            state = stSonarFiring;
            // firing sonar array
            Cart.fireSonarArray();
        }


        public void transitionAction(ref System.Windows.Forms.Panel panel, 
                                     System.Windows.Forms.TextBox[] texts, ref int pass)
        {
            switch (state)
            {
                case stIdle: // nothing
                    break;
                case stSonarFiring:
                    if (Cart.flagSonarArrayFiringDone)
                    {
                        Cart.flagSonarArrayFiringDone = false;
                        // changing state
                        state = stSonarDataTransmission;
                        Cart.requestSensorData();
                    }
                    break;
                case stSonarDataTransmission:
                    if (Cart.flagSensorDataAcquired)
                    {
                        Cart.fillSonarTextBoxes(texts);

                        state = stFrameTransmission;
                        Cart.retrieveAbstraction();
                    }
                    break;
                case stFrameTransmission:
                    if (Cart.flagAbstractionReady)
                    {
                        Cart.drawAbstraction(panel);
                        state = stAbilityExecuting;
                        // running the cognitive array now
                        double[][] inputVecs = Cart.makeInputVector();
                        pass++;
                        int output = (int)MetaNode.getOutput(Cart.CogTop, inputVecs, pass);
                        if (output < 11)
                            Cart.useAbility((t_CartAbility)output);
                        else 
                            state = stIdle;
                    }
                    break;
                case stAbilityExecuting:
                    if (Cart.flagAbilityDone)
                    {
                        Cart.flagAbilityDone = false;
                        state = stIdle;
                    }
                    break;

            }
        }



    }
}
