using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class RSV2TrainingFSM
    {
        // the robosapien to be trained by the FSM
        public RobosapienV2 Robosapien;

        // state constants
        public const int stIdle = 0;
        public const int stAbilityExecuting = 1;
        public const int stSensorDataTransmission = 3;
        public const int stFrameTransmission = 4;

        public int LastUsedAbility;
        public double[][] LastInputVecs;
        public double[] TopNodeInput;


        public int state;

        // constructor
        public RSV2TrainingFSM(RobosapienV2 rsv2)
        {
            Robosapien = rsv2;

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
            LastInputVecs = Robosapien.makeInputVector();

            // finding the input vector for the top node
            TopNodeInput = new double[Robosapien.CogTop.InputNum];
            int i;
            for (i = 0; i < Robosapien.CogTop.InputNum; i++)
                TopNodeInput[i] = MetaNode.getOutput(Robosapien.CogTop.Children[i], LastInputVecs, pass);
            // now training
            double[] DesiredOutputVec = STANN.mapInt2VectorDouble(ability, 2, Robosapien.CogTop.stann.OutputNum);

            // training 6 times
            for (i = 0; i < 6; i++)
                Robosapien.CogTop.stann.backPropagate(TopNodeInput, DesiredOutputVec);

            // executing ability now..
            Robosapien.useAbility((t_RSV2Ability)ability);
        }

        public void transitionAction(System.Windows.Forms.Panel panel, System.Windows.Forms.TextBox[] texts)
        {
            switch (state)
            {
                case stIdle: // nothing
                    break;
                case stAbilityExecuting:
                    if (Robosapien.flagAbilityDone)
                    {
                        Robosapien.flagAbilityDone = false;
                        // ability dopne. Now firing the transducers
                        state = stSensorDataTransmission;
                        Robosapien.requestSensorData();
                    }
                    break;

                case stSensorDataTransmission:
                    if (Robosapien.flagSensorDataAcquired)
                    {
                        Robosapien.fillSensorTexts(texts);
                        // flag has been cleared inside fillSensorTexts
                        // now retrieving a camera frame abstraction
                        state = stFrameTransmission;
                        Robosapien.retrieveAbstraction();
                    }
                    break;
                case stFrameTransmission:
                    if (Robosapien.flagAbstractionReady)
                    {
                        Robosapien.drawAbstraction(panel); // no need to reset the flag. 
                        // It's being reset inside drawAbstraction()
                        state = stIdle;
                    }
                    break;
            }
        }


    }
}
