using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class CartTrainingFSM
    {
        // the Mobile robot trained by the FSM
        public LMMobileRobot Cart;
        // FSM's internal state
        public int state;

        // state constants
        public const int stIdle = 0;
        public const int stAbilityExecuting = 1;
        public const int stSonarFiring = 2;
        public const int stSonarDataTransmission = 3;
        public const int stFrameTransmission = 4;

        public int LastUsedAbility;
        public double[][] LastInputVecs;
        public double[] TopNodeInput;

        //Training FSM constructor
        public CartTrainingFSM(LMMobileRobot cart) {
            Cart = cart;

            state = stIdle;
        }

        public void trainingStep(int ability, ref int pass)
        {
            pass++;
            // 1. initiating ability
            state = stAbilityExecuting;

            // storing chosen ability
            LastUsedAbility = ability;
            // storing current input vectors
            LastInputVecs = Cart.makeInputVector();

            // finding the input vector for the top node
            TopNodeInput = new double[Cart.CogTop.InputNum];
            int i;
            for (i=0; i<Cart.CogTop.InputNum; i++) 
                TopNodeInput[i] = MetaNode.getOutput(Cart.CogTop.Children[i], LastInputVecs, pass);
            // now training
            double[] DesiredOutputVec = STANN.mapInt2VectorDouble(ability, 2, Cart.CogTop.stann.OutputNum);

            // training 6 times
            for (i=0; i<6; i++)
                Cart.CogTop.stann.backPropagate(TopNodeInput, DesiredOutputVec);
            
            // executing ability now..
            Cart.useAbility((t_CartAbility)ability);
        }

        public void transitionAction(System.Windows.Forms.Panel panel, System.Windows.Forms.TextBox[] texts)
        {
            switch (state)
            {
                case stIdle: // nothing
                    break;
                case stAbilityExecuting:
                    if (Cart.flagAbilityDone)
                    {
                        Cart.flagAbilityDone = false;
                        // ability dopne. Now firing the transducers
                        state = stSonarFiring;
                        Cart.fireSonarArray();
                    }
                    break;

                case stSonarFiring:
                    if (Cart.flagSonarArrayFiringDone)
                    {
                        // now attempting to retrieve the data
                        state = stSonarDataTransmission;
                        Cart.requestSensorData();
                    }
                    break;

                case stSonarDataTransmission:
                    if (Cart.flagSensorDataAcquired)
                    {
                        Cart.fillSonarTextBoxes(texts); 
                        // now retrieving a camera frame abstraction
                        state = stFrameTransmission;
                        Cart.retrieveAbstraction();
                    }
                    break;
                case stFrameTransmission:
                    if (Cart.flagAbstractionReady)
                    {
                        Cart.drawAbstraction(panel); // no need to reset the flag. 
                        // It's being reset inside drawAbstraction()
                        state = stIdle;
                    }
                    break;
            }
        }



    }
}
