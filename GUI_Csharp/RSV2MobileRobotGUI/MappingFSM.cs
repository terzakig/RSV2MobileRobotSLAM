using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;

namespace RobosapienRFControl
{
    struct MapCell {
        public int ipos, jpos;
    }

    class MappingFSM
    {
        // the mobile robot 
        public LMMobileRobot Cart;

        // a robosapienv2 possibly browsing the mapped area
        RSV2BrowsingFSM browsingFSM;

        // the map (public to a robosapien robot)
        public char[][] Map;
        public int MapDim; // dimension of the map
        // map constants
        public const char cellUNKNOWN = 'X';
        public const char cellBLOCK = '*';
        public const char cellVOID = ' ';
        

        // Correction tollerance
        public const int ANGULARERROR_TOLLERANCE = 18; // degrees
        public const int DISPLACEMENT_TOLLERANCE = 15;   // centimeters


        // orientation of the robot
        public int orientation;
        public int previousOrientation;
        // orientation constants
        public const int orNORTH = 0;
        public const int orEAST = 1;
        public const int orSOUTH = 2;
        public const int orWEST = 3;

        // position of the robot (line, column) in the map
        public int ipos, jpos;

        // internal state of the FSM
        public int state;
        // possible states
        public const int stIdle = 0;
        public const int stAbilityExecuting = 1;
        public const int stSonarFiring = 2;
        public const int stTasksExecuting = 3;
        public const int stSonarDataTransmission = 4;
        public const int stCorrectingSonarFiring = 5;
        public const int stCorrectingSonarDataTransmission = 6;
        public const int stCorrectingExecutingTurn = 7;
        public const int stCorrectingSonarFiring1 = 8;
        public const int stCorrectingSonarDataTransmission1 = 9;
        public const int stCorrectingTasksExecuting = 10;
        public const int stCorrectingAbilityExecuting = 11;
        public const int stExtraCorrectionSonarFiring = 12;
        public const int stExtraCorrectionSonarDataTransmission = 13;
        public const int stExtraCorrectionTasksExecuting = 14;
        public const int stExtraCorrectionAbilityExecuting = 15;

        // a uqueue of elementary tasks
        public int[] TaskQueue;
        public int TaskQueueLength;
        public int CurrentTask; // task being currently executed
        public int CurrentCorrectionTask; // correction task currently being executed
        // queue consts
        public const int MAX_QUEUE_LENGTH = 500;
        // task constants
        public const int taskGO_FORWARD = 0;
        public const int taskGO_BACKWARD = 1;
        public const int taskTURN_RIGHT = 2;
        public const int taskTURN_LEFT = 3;
        public const int taskTURN_LEFT10 = 4;
        public const int taskTURN_RIGHT10 = 5;
        public const int taskGO_BACKWARD10 = 6;

        // a flag 
        public Boolean flagMappingDone;

        
        // Last known displacement gradient estimate
        public double DisplacementX, DisplacementY;
        // Last known angular error estimate 
        public double AngularError;
        // number of correction tasks in the queue
        public int CorrectionTaskNum;
        // number of additional correction tasks in the queue
        public int ExtraCorrectionTaskNum;

        // the cart's initial sonar array readings before a turn (used for aditional corrections)
        public ushort[] InitialSonarArray;


        // Last Known position of the Robosapien (if registered)
        public int LastKnownRsv2Ipos, LastKnownRsv2Jpos;

        // probability tables
        double[][] BlockProbability;
        double[][] VoidProbability;

        // constructor
        public MappingFSM(LMMobileRobot cart, int mapdim, int orient, int robi, int robj)
        {
            // setting Last known position of the robosapien to -1, -1 (unregistered)
            LastKnownRsv2Ipos = -1;
            LastKnownRsv2Jpos = -1;

            // Setting displacement and angular error estimates to 0.
            DisplacementX = DisplacementY = 0;
            AngularError = 0;
            // clearing number of correction tasks
            CorrectionTaskNum = ExtraCorrectionTaskNum = 0;
            // creating the Initial Sonar Array
            InitialSonarArray = new ushort[8];


            // copying constructor parameters
            Cart = cart;
            MapDim = mapdim;
            // creating map filled with unknown cells
            /*Map = new char[][]  { new char[] {'*', '*', '*', '*', '*', '*', '*', '*'},
                                  new char[] {'*', ' ', ' ', '*', ' ', '*', ' ', '*'},
                                  new char[] {'*', ' ', ' ', '*', ' ', '*', ' ', '*'},
                                  new char[] {'*', ' ', ' ', '*', ' ', ' ', ' ', '*'},
                                  new char[] {'*', '*', ' ', ' ', ' ', '*', '*', '*'},
                                  new char[] {'*', ' ', '*', ' ', '*', ' ', ' ', '*'},
                                  new char[] {'*', ' ', ' ', ' ', ' ', ' ', '*', '*'},
                                  new char[] {'*', '*', '*', '*', '*', '*', '*', '*'}}; */
            Map = new char[MapDim][];
            BlockProbability = new double[MapDim][];
            VoidProbability = new double[MapDim][];
            int i, j;
            for (i = 0; i < MapDim; i++)
            {
                Map[i] = new char[MapDim];
                BlockProbability[i] = new double[MapDim];
                VoidProbability[i] = new double[MapDim];
                for (j = 0; j < MapDim; j++)
                {
                    // filling map line with unknown cells
                    Map[i][j] = cellUNKNOWN;
                    // setting block probability for each cell to 0.5 (max. entropy prior)
                    BlockProbability[i][j] = 0.5;
                    VoidProbability[i][j] = 0.5;

                }
            }
            Map[robi][robj] = cellVOID;

            // initializing task Queue
            TaskQueue = new int[MAX_QUEUE_LENGTH];
            TaskQueueLength = 0;

            // robot position 
            ipos = robi;
            jpos = robj;
            orientation = orient;
            previousOrientation = orient;

            // marking current position as void.
            Map[ipos][jpos] = cellVOID;
            // setting block probability to 0 for the robot position
            BlockProbability[ipos][jpos] = 0;

            // mapping is not finished
            flagMappingDone = false;

            browsingFSM = null;

            // state is idle
            state = stIdle;
        }


        public void resetFSM(int robi, int robj, int orient, int dim)
        {
            
            // setting Last known position of the robosapien to -1, -1 (unregistered)
            LastKnownRsv2Ipos = -1;
            LastKnownRsv2Jpos = -1;
            // clearing task queue
            TaskQueueLength = 0;
            // reconstructing the map
            MapDim = dim;
            Map = new char[MapDim][];
            BlockProbability = new double[MapDim][];
            VoidProbability = new double[MapDim][];

            int i, j;
            for (i = 0; i < MapDim; i++)
            {
                BlockProbability[i] = new double[MapDim];
                VoidProbability[i] = new double[MapDim];
                Map[i] = new char[MapDim];
                for (j = 0; j < MapDim; j++)
                {
                    Map[i][j] = cellUNKNOWN;
                    BlockProbability[i][j] = 0.5;
                    VoidProbability[i][j] = 0.5;
                }
            }

            // assigning cart coordinates
            ipos = robi;
            jpos = robj;
            orientation = orient;

            // clearing robot cell
            Map[ipos][jpos] = cellVOID;
            // clear browsing fsm
            browsingFSM = null;

            // resetting state
            state = stIdle;
            // clearing completion flag
            flagMappingDone = false;
        }

        public void registerBrowingFSM(RSV2BrowsingFSM bfsm)
        {
            browsingFSM = bfsm;
        }


        public int removeTask()
        {
            int val=0;
            if (TaskQueueLength > 0)
            {
                val = TaskQueue[TaskQueueLength - 1];
                TaskQueueLength--;
            }

            return val;
        }

        public void insertPriorityTask(int task)
        {
            TaskQueueLength++;

            TaskQueue[TaskQueueLength - 1] = task;
        }

        public void insertTask(int task)
        {
            int i;
            for (i = TaskQueueLength; i > 0; i--)
                TaskQueue[i] = TaskQueue[i - 1];
            TaskQueueLength++;

            TaskQueue[0] = task;

        }


        public void mappingStep(ref System.Windows.Forms.TextBox[] texts)
        {
        if (!flagMappingDone)
            switch (state)
            {
                case stIdle: // firing sonar
                    state = stSonarFiring;
                    Cart.fireSonarArray();
                    break;
                case stSonarFiring:
                    if (Cart.flagSonarArrayFiringDone)
                    {
                        // flag reset
                        Cart.flagSonarArrayFiringDone = false;
                        // changing state and retrieveing sonar readings
                        state = stSonarDataTransmission;
                        Cart.requestSensorData();
                    }
                    break;
                case stSonarDataTransmission:
                    if (Cart.flagSensorDataAcquired)
                    {
                        Cart.fillSonarTextBoxes(texts);
                        // Now invoking the planner
                        
                        planner();
                        state = stTasksExecuting;
                        
                    }
                    break;
                case stTasksExecuting:
                    if (TaskQueueLength == 0) state = stIdle;
                    else
                    {
                        CurrentTask = removeTask();

                        if ((CurrentTask == taskTURN_LEFT) || (CurrentTask == taskTURN_RIGHT)) {
                            state = stCorrectingSonarFiring;
                            Cart.fireSonarArray();
                        } else {
                            state = stAbilityExecuting;

                            switch (CurrentTask)
                            {
                                case taskGO_FORWARD: Cart.useAbility(t_CartAbility.abGO_FORWARD);
                                    break;
                                case taskGO_BACKWARD: Cart.useAbility(t_CartAbility.abGO_BACKWARD);
                                    break;
                                case taskTURN_LEFT: Cart.useAbility(t_CartAbility.abTURN_LEFT90);
                                    break;
                                case taskTURN_RIGHT: Cart.useAbility(t_CartAbility.abTURN_RIGHT90);
                                    break;
                            }
                    
                        }
                    }
                    break;

                case stAbilityExecuting:
                    if (Cart.flagAbilityDone)
                    {
                        // reseting the flag
                        Cart.flagAbilityDone = false;
                        // finding new orientation and position
                        switch (orientation)
                        {
                            case orNORTH:
                                switch (CurrentTask)
                                {
                                    case taskGO_FORWARD:
                                        previousOrientation = orientation;
                                        orientation = orNORTH;
                                        ipos--;
                                        break;
                                    case taskGO_BACKWARD:
                                        previousOrientation = orientation;
                                        orientation = orNORTH;
                                        ipos++;
                                        break;
                                    case taskTURN_LEFT: 
                                        previousOrientation = orientation; 
                                        orientation = orWEST;
                                        break;
                                    case taskTURN_RIGHT: 
                                        previousOrientation = orientation;
                                        orientation = orEAST;
                                        break;
                                }
                                break;
                            case orSOUTH:
                                switch (CurrentTask)
                                {
                                    case taskGO_FORWARD:
                                        previousOrientation = orientation;
                                        orientation = orSOUTH;
                                        ipos++;
                                        break;
                                    case taskGO_BACKWARD:
                                        previousOrientation = orientation;
                                        orientation = orSOUTH;
                                        ipos--;
                                        break;
                                    case taskTURN_LEFT:
                                        previousOrientation = orientation;
                                        orientation = orEAST;
                                        break;
                                    case taskTURN_RIGHT:
                                        previousOrientation = orientation;
                                        orientation = orWEST;
                                        break;
                                }
                                break;
                            case orEAST:
                                switch (CurrentTask)
                                {
                                    case taskGO_FORWARD:
                                        previousOrientation = orientation;
                                        orientation = orEAST;
                                        jpos++;
                                        break;
                                    case taskGO_BACKWARD:
                                        previousOrientation = orientation;
                                        orientation = orEAST;
                                        jpos--;
                                        break;
                                    case taskTURN_LEFT:
                                        previousOrientation = orientation;
                                        orientation = orNORTH;
                                        break;
                                    case taskTURN_RIGHT:
                                        previousOrientation = orientation;
                                        orientation = orSOUTH;
                                        break;
                                }
                                break;
                            case orWEST:
                                switch (CurrentTask)
                                {
                                    case taskGO_FORWARD:
                                        previousOrientation = orientation;
                                        orientation = orWEST;
                                        jpos--;
                                        break;
                                    case taskGO_BACKWARD:
                                        previousOrientation = orientation;
                                        orientation = orWEST;
                                        jpos++;
                                        break;
                                    case taskTURN_LEFT:
                                        previousOrientation = orientation;
                                        orientation = orSOUTH;
                                        break;
                                    case taskTURN_RIGHT:
                                        previousOrientation = orientation;
                                        orientation = orNORTH;
                                        break;
                                }
                                break;
                        } // end  switch (orientation)
                       

                        // changing state
                        if (browsingFSM != null)
                        {
                            if ((LastKnownRsv2Ipos != browsingFSM.ipos) || (LastKnownRsv2Jpos != browsingFSM.jpos))
                            {// robosapien has changed his position in the map
                                // clear the queue 
                                TaskQueueLength = 0;
                                // and go back to planning
                                state = stIdle;
                            }
                            else
                            {
                                if (TaskQueueLength == 0)
                                    state = stIdle;
                                else
                                    state = stTasksExecuting;
                            }
                        } else 
                              if (TaskQueueLength == 0) 
                                  state = stIdle;
                        else 
                                  state = stTasksExecuting;
                    } // end if
                    break;
                case stCorrectingSonarFiring: // sonar firing for correction purposes
                    if (Cart.flagSonarArrayFiringDone)
                    {
                        // flag reset
                        Cart.flagSonarArrayFiringDone = false;
                        // changing state and retrieveing sonar readings
                        state = stCorrectingSonarDataTransmission;
                        Cart.requestSensorData();
                    }
                    break;
                case stCorrectingSonarDataTransmission:
                    if (Cart.flagSensorDataAcquired)
                    {
                        Cart.fillSonarTextBoxes(texts);
                        // now executing the turn
                        state = stCorrectingExecutingTurn;
                        if (CurrentTask == taskTURN_LEFT)
                            Cart.useAbility(t_CartAbility.abTURN_LEFT90);
                        else 
                            Cart.useAbility(t_CartAbility.abTURN_RIGHT90);
                    }
                    break;
                case stCorrectingExecutingTurn:
                    if (Cart.flagAbilityDone) {
                        // resetting flag
                        Cart.flagAbilityDone = false;
                        // finding new orientation
                        previousOrientation = orientation;
                        switch(previousOrientation) {
                            case orNORTH:
                                if (CurrentTask == taskTURN_LEFT) 
                                    orientation = orWEST;
                                else 
                                    orientation = orEAST;
                                break;
                            case orEAST:
                                if (CurrentTask == taskTURN_LEFT) 
                                    orientation = orNORTH;
                                else 
                                    orientation = orSOUTH;
                                break;
                            case orSOUTH:
                                if (CurrentTask == taskTURN_LEFT) 
                                    orientation = orEAST;
                                else 
                                    orientation = orWEST;
                                break;
                            case orWEST:
                                if (CurrentTask == taskTURN_LEFT) 
                                    orientation = orSOUTH;
                                else 
                                    orientation = orNORTH;
                                break;
                        }
                        // now must fire the Sonar Array again
                        state = stCorrectingSonarFiring1;
                        Cart.fireSonarArray();
                    }
                    break;
                case stCorrectingSonarFiring1:
                    if (Cart.flagSonarArrayFiringDone) {
                        // clearing flag
                        Cart.flagSonarArrayFiringDone = false;
                        // retrieving the readings
                        state = stCorrectingSonarDataTransmission1;
                        Cart.requestSensorData();
                    }
                    break;
                case stCorrectingSonarDataTransmission1:
                    if (Cart.flagSensorDataAcquired) {
                        // clearing the flag
                        Cart.flagSensorDataAcquired = false;
                        Cart.fillSonarTextBoxes(texts);
                        // must now invoke correction. state will change inside the method
                        correction();
                    }
                    break;
                case stCorrectingTasksExecuting:
                    if (CorrectionTaskNum > 0)
                    {
                        CurrentCorrectionTask = removeTask();
                        // decrease number of correction tasks
                        CorrectionTaskNum--;
                        // switch state to ability execution while correcting
                        state = stCorrectingAbilityExecuting;
                        switch (CurrentCorrectionTask)
                        {
                            case taskGO_BACKWARD10: Cart.useAbility(t_CartAbility.abGO_BACKWARD10);
                                break;
                            case taskTURN_LEFT10: Cart.useAbility(t_CartAbility.abTURN_LEFT10);
                                break;
                            case taskTURN_RIGHT10: Cart.useAbility(t_CartAbility.abTURN_RIGHT10);
                                break;
                        }
                    }
                    else // going back to normal tasks execution
                        state = stTasksExecuting;
                    break;
                case stCorrectingAbilityExecuting:
                    if (Cart.flagAbilityDone)
                    {
                        // reset flag
                         Cart.flagAbilityDone = false;
                        // changing state
                         if ((CurrentCorrectionTask == taskTURN_LEFT10) ||
                             (CurrentCorrectionTask == taskTURN_RIGHT10))
                         { // must check if angular error grew
                             state = stExtraCorrectionSonarFiring;
                             // firing sonar
                             Cart.fireSonarArray();
                         } else 
                            if (CorrectionTaskNum > 0)
                                state = stCorrectingTasksExecuting;
                            else // go back to normal execution
                                state = stTasksExecuting;
                    }
                    break;
                case stExtraCorrectionSonarFiring:
                    if (Cart.flagSonarArrayFiringDone)
                    {
                        // clearing flag
                        Cart.flagSonarArrayFiringDone = false;
                        // now must retrieve data
                        // changing state
                        state = stExtraCorrectionSonarDataTransmission;
                        Cart.requestSensorData();
                    }
                    break;
                case stExtraCorrectionSonarDataTransmission:
                    if (Cart.flagSensorDataAcquired)
                    {
                        // clearing flag
                        Cart.flagSensorDataAcquired = false;
                        // must now call to extra correction
                        extraCorrection(); // state will change inside the method
                    }
                    break;
                case stExtraCorrectionTasksExecuting:
                    if (ExtraCorrectionTaskNum > 0)
                    {
                        ExtraCorrectionTaskNum--;

                        int theTask = removeTask();
                        switch (theTask)
                        {
                            case taskTURN_LEFT10: Cart.useAbility(t_CartAbility.abTURN_LEFT10);
                                break;
                            case taskTURN_RIGHT10: Cart.useAbility(t_CartAbility.abTURN_RIGHT10);
                                break;
                        }
                        // changing state
                        state = stExtraCorrectionAbilityExecuting;
                    }
                    else // going back to normal correction task execution
                        state = stCorrectingTasksExecuting;
                    break;
                case stExtraCorrectionAbilityExecuting:
                    if (Cart.flagAbilityDone)
                    {
                        // clearing flag
                        Cart.flagAbilityDone = false;
                        // changing state
                        state = stExtraCorrectionTasksExecuting;
                    }
                    break;

            } // end REALLY HUGE state switch

            
        
        }


        // a function that estimates the angular displacement error after
        // a rotation correction task has been executed
        public void extraCorrection()
        {
            ExtraCorrectionTaskNum = 0;
            // restoring initial sonar array in the cart
            Cart.PrevSonarArray = InitialSonarArray;
            // computing the position gradient (for display purposes)
            double[] Gxy = getGradient(Cart, orientation, previousOrientation);

            // computing the new angular error estimate
            double newAngularError = getAngleOffset(Cart, orientation, previousOrientation);
            // now checking if the new angular error is greater than the original
            if (newAngularError > AngularError)
            { // must go the other side
                ExtraCorrectionTaskNum = 2;
                if (CurrentCorrectionTask == taskTURN_LEFT10)
                { // must go right
                    insertPriorityTask(taskTURN_RIGHT10);
                    insertPriorityTask(taskTURN_RIGHT10);
                }
                else
                {
                    insertPriorityTask(taskTURN_LEFT10);
                    insertPriorityTask(taskTURN_LEFT10);
                }
                // changing state
                state = stExtraCorrectionTasksExecuting;
            }
            else // going back to the rest of the normal correction tasks left
                state = stCorrectingTasksExecuting;
        }




        // a function that estimates the angular and displacement error
        // and chooses correction action
        public void correction()
        {
            // Keeping the Cart's previous sonar array readings as Initial Sonar Array
            int i;
            for (i = 0; i < 8; i++)
                InitialSonarArray[i] = Cart.PrevSonarArray[i];

            // clearing number of correcting tasks
            CorrectionTaskNum = 0;

            // computing the displacement gradient
            double[] Gxy = getGradient(Cart, orientation, previousOrientation);
            DisplacementX = Gxy[0];
            DisplacementY = Gxy[1];
            // gradient computed

            // now computing the angular error estimate
            double angError = getAngleOffset(Cart, orientation, previousOrientation);
            AngularError = angError;

            if (Math.Abs(DisplacementX) > DISPLACEMENT_TOLLERANCE)
            {
                CorrectionTaskNum++;
                insertPriorityTask(taskGO_BACKWARD10);
            }

            if (Math.Abs(AngularError) > ANGULARERROR_TOLLERANCE)
            {
                CorrectionTaskNum++;
                int corTask = 0;
                switch (orientation)
                {
                    case orNORTH:
                        switch (previousOrientation)
                        {
                            case orWEST:
                                corTask = taskTURN_LEFT10;
                                break;
                            case orEAST:
                                corTask = taskTURN_RIGHT10;
                                break;
                        }
                        break;
                    case orEAST:
                        switch (previousOrientation)
                        {
                            case orNORTH:
                                corTask = taskTURN_LEFT10;
                                break;
                            case orSOUTH:
                                corTask = taskTURN_RIGHT10;
                                break;
                        }
                        break;
                    case orSOUTH:
                        switch (previousOrientation)
                        {
                            case orEAST:
                                corTask = taskTURN_LEFT10;
                                break;
                            case orWEST:
                                corTask = taskTURN_RIGHT10;
                                break;
                        }
                        break;
                    case orWEST:
                        switch (previousOrientation)
                        {
                            case orSOUTH:
                                corTask = taskTURN_LEFT10;
                                break;
                            case orNORTH:
                                corTask = taskTURN_RIGHT10;
                                break;
                        }
                        break;
                } // end big switch

                insertPriorityTask(corTask);
            }
            if (CorrectionTaskNum > 0)
                state = stCorrectingTasksExecuting;
            else // go about doing normal task execution again
                state = stTasksExecuting;
        }


        public Boolean isNeighborUnknown(int i, int j) {
            Boolean result = false;
            if (i>0) 
                if (Map[i-1][j] == cellUNKNOWN)
                    result = true;
            
            if (i<MapDim-1) 
                if (Map[i+1][j] == cellUNKNOWN)
                    result = true;
            
            if (j>0) 
                if (Map[i][j-1] == cellUNKNOWN)
                    result = true;
            
            if (j<MapDim-1) 
                if (Map[i][j+1] == cellUNKNOWN)
                    result = true;
            
            return result;
        }
        

        // The planner
        void planner()
        {
            // call to percept
            percept();

            // finding the closest reachable unknown cell
            int i, j;
            int minpathlen = 1000000, mini, minj;
            APath minpath = null;

            for (i = 0; i < MapDim; i++)
                for (j = 0; j < MapDim; j++)
                    if (Map[i][j] == cellVOID)
                        if (isNeighborUnknown(i, j))
                        {
                            Boolean pfound = false;
                            APath path2cell = shortestPath(ipos, jpos, i, j, null, 0, ref pfound);
                            if (pfound)
                                if (minpathlen > path2cell.pathlen)
                                {
                                    minpathlen = path2cell.pathlen;
                                    minpath = path2cell;
                                    mini = i;
                                    minj = j;
                                }
                        }

            if (minpath == null) 
                flagMappingDone = true;
            else 
                path2QueueTasks(minpath, orientation);
        }



        // the following function realizes neighboring cells 
        // by examining the sensory data
/*        public void percept()
        {
            double[] sensors = new double[8];
            int i;
            for (i = 0; i < 8; i++)
                sensors[i] = (double)Cart.SonarArray[i];

            double Rc = 11, Rd = 35;
            double pB1 = 0.8, pB2 = 0.8, pB3 = 0.8;

            // calculating parameters of desity probability functions
            double a1 = -Math.Log(1 - pB1, Math.Exp(1))/(-Rc + Rd/Math.Cos(Math.PI/4));
            double a2 = -Math.Log(1 - pB2, Math.Exp(1)) / (-Rc + 3*Rd);
            double a3 = -Math.Log(1 - pB3, Math.Exp(1)) / (-Rc + Rd / Math.Cos(Math.PI / 4));



            // ************************ Front side ****************************************
            double Sf0 = sensors[7]  - (Rd/Math.Cos(Math.PI/4) - Rc);
            double Sf1 = sensors[0] - (3*Rd - Rc);
            double Sf2 = sensors[1] - (Rd/Math.Cos(Math.PI/4) - Rc);

            double[] coeffs = new double[] { 0.2, 0.6, 0.2 };

            double frontsum = coeffs[0]*Sf0 + coeffs[1]*Sf1 + coeffs[2]*Sf2;
            //Boolean FrontBlock =  frontsum < 42;
            Boolean FrontBlock = (Sf0 < 0) || (Sf1 < 0) || (Sf2 < 0);

           
            // ******************** Right Side *****************************************
            double Sr0 = sensors[1] - (Rd / Math.Cos(Math.PI / 4) - Rc);
            double Sr1 = sensors[2] - (3*Rd - Rc);
            double Sr2 = sensors[3] - (Rd / Math.Cos(Math.PI / 4) - Rc);


            double rightsum = coeffs[0] * Sr0 + coeffs[1] * Sr1 + coeffs[2] * Sr2;

            //Boolean RightBlock = rightsum < 42;
            Boolean RightBlock = (Sr0 < 0) || (Sr1 <0) || (Sr2 <0);


            // ******************** back Side *****************************************
            double Sb0 = sensors[3] - (Rd / Math.Cos(Math.PI / 4) - Rc);
            double Sb1 = sensors[4] - (3*Rd - Rc);
            double Sb2 = sensors[5] - (Rd / Math.Cos(Math.PI / 4) - Rc);


            double backsum = coeffs[0] * Sb0 + coeffs[1] * Sb1 + coeffs[2] * Sb2;

            //Boolean RearBlock = backsum < 42;
            Boolean RearBlock = (Sb0 < 0) || (Sb1 < 0) || (Sb2 < 0);


            // ******************** Left Side *****************************************
            double Sl0 = sensors[5] - (Rd / Math.Cos(Math.PI / 4) - Rc);
            double Sl1 = sensors[6] - (3*Rd - Rc);
            double Sl2 = sensors[7] - (Rd / Math.Cos(Math.PI / 4) - Rc);


            double leftsum = coeffs[0] * Sl0 + coeffs[1] * Sl1 + coeffs[2] * Sl2;

            //Boolean LeftBlock =  leftsum< 42;
            Boolean LeftBlock = (Sl0 < 0) || (Sl1 < 0) || (Sl2 < 0);

            // updating map
            switch (orientation)
            {
                case orNORTH:
                    if (ipos > 0)
                        if (Map[ipos - 1][jpos] == cellUNKNOWN)
                            Map[ipos - 1][jpos] = FrontBlock ? cellBLOCK : cellVOID;
                    if (jpos > 0)
                        if (Map[ipos][jpos - 1] == cellUNKNOWN)
                            Map[ipos][jpos - 1] = LeftBlock ? cellBLOCK : cellVOID;
                    if (jpos < MapDim - 1)
                        if (Map[ipos][jpos+1] == cellUNKNOWN)
                            Map[ipos][jpos + 1] = RightBlock ? cellBLOCK : cellVOID;
                    if (ipos < MapDim - 1)
                        if (Map[ipos + 1][jpos] == cellUNKNOWN)
                            Map[ipos + 1][jpos] = RearBlock ? cellBLOCK : cellVOID;
                    break;
                case orEAST: 
                    if (ipos > 0)
                        if (Map[ipos - 1][jpos]==cellUNKNOWN)
                            Map[ipos - 1][jpos] = LeftBlock ? cellBLOCK : cellVOID;
                    if (jpos > 0)
                        if (Map[ipos][jpos - 1] == cellUNKNOWN)
                            Map[ipos][jpos - 1] = RearBlock ? cellBLOCK : cellVOID;
                    if (jpos < MapDim - 1)
                        if (Map[ipos][jpos + 1] == cellUNKNOWN)
                            Map[ipos][jpos + 1] = FrontBlock ? cellBLOCK : cellVOID;
                    if (ipos < MapDim - 1)
                        if (Map[ipos + 1][jpos] == cellUNKNOWN)
                            Map[ipos + 1][jpos] = RightBlock ? cellBLOCK : cellVOID;
                    break;
                case orSOUTH:
                    if (ipos > 0)
                        if (Map[ipos - 1][jpos] == cellUNKNOWN)
                            Map[ipos - 1][jpos] = RearBlock ? cellBLOCK : cellVOID;
                    if (jpos > 0)
                        if (Map[ipos][jpos - 1] == cellUNKNOWN)
                            Map[ipos][jpos - 1] = RightBlock ? cellBLOCK : cellVOID;
                    if (jpos < MapDim - 1)
                        if (Map[ipos][jpos + 1] == cellUNKNOWN)
                            Map[ipos][jpos + 1] = LeftBlock ? cellBLOCK : cellVOID;
                    if (ipos < MapDim - 1)
                        if (Map[ipos + 1][jpos] == cellUNKNOWN)
                            Map[ipos + 1][jpos] = FrontBlock ? cellBLOCK : cellVOID;
                    break;
                case orWEST:
                    if (ipos > 0)
                        if (Map[ipos - 1][jpos] == cellUNKNOWN)
                            Map[ipos - 1][jpos] = RightBlock ? cellBLOCK : cellVOID;
                    if (jpos > 0)
                        if (Map[ipos][jpos - 1] == cellUNKNOWN)
                            Map[ipos][jpos - 1] = FrontBlock ? cellBLOCK : cellVOID;
                    if (jpos < MapDim - 1)
                        if (Map[ipos][jpos + 1] == cellUNKNOWN)
                            Map[ipos][jpos + 1] = RearBlock ? cellBLOCK : cellVOID;
                    if (ipos < MapDim - 1)
                        if (Map[ipos + 1][jpos] == cellUNKNOWN)
                            Map[ipos + 1][jpos] = LeftBlock ? cellBLOCK : cellVOID;
                    break;
            }
            // if the other robot is on any of the neighboring cells,
            // the cell should be marked as empty (sonar saw the robot as obstacle)
            if (browsingFSM != null)
            {
                if ((ipos - 1 == browsingFSM.ipos) && (jpos == browsingFSM.jpos))
                    Map[ipos - 1][jpos] = cellVOID;
                else
                    if ((ipos + 1 == browsingFSM.ipos) && (jpos == browsingFSM.jpos))
                        Map[ipos + 1][jpos] = cellVOID;
                    else
                        if ((ipos == browsingFSM.ipos) && (jpos - 1 == browsingFSM.jpos))
                            Map[ipos][jpos - 1] = cellVOID;
                        else
                            if ((ipos == browsingFSM.ipos) && (jpos + 1 == browsingFSM.jpos))
                                Map[ipos][jpos + 1] = cellVOID;
            }
        }
        */



        public void percept()
        {
            double[] sensors = new double[8];
            int i;
            for (i = 0; i < 8; i++)
                sensors[i] = (double)Cart.SonarArray[i];

            double sinphi = Math.Sin(Math.PI/4);
            // ************* User-defined parameters ************************
            // Cart radius (Rd) and sonar ring radius (Rc) in cms
            double Rc = 11, Rd = 18;
            
            // ----------------- Likelihood p.d. functions p(z1|VOID), p(z2|VOID), p(z3|VOID)
            // Measurement threshold up to which, the integral of p(z1|VOID), p(z2|VOID) and p(z3|VOID) is 1
           
            double Bound1 = Rd/sinphi - Rc + 7;
            
            double Bound2 = 3*Rd - Rc + 7;
            
            double Bound3 = Rd/sinphi - Rc + 7;
            

            // calculating parameters of likelihood functions for P(x | BLOCK)and p(x | VOID)
            // conditional pdfs are assumed to be quadratics: a(z-Bound)^2 and az^2
            double a1 = 3 / Math.Pow(Bound1, 3);
            double a2 = 3 / Math.Pow(Bound2, 3);
            double a3 = 3 / Math.Pow(Bound3, 3);
            // Done with all likelihood pdf paparameters. Now computing probabilities


            Boolean FrontValid = false, RightValid = false, LeftValid = false, BackValid = false;
            int FrontI=0, FrontJ=0, LeftI=0, LeftJ=0, RightI=0, RightJ=0, BackI=0, BackJ=0;

            double Pk_1BlockFront=0, Pk_1BlockRight=0, Pk_1BlockBack=0, Pk_1BlockLeft=0;
            double Pk_1VoidFront = 0, Pk_1VoidRight = 0, Pk_1VoidBack = 0, Pk_1VoidLeft = 0;

            

            // examining if neighboring blocks are inside the map
            switch (orientation)
            {
                case orNORTH:
                    FrontValid = ipos > 0;
                    FrontI = ipos - 1;
                    FrontJ = jpos;
                    RightValid = jpos < MapDim - 1;
                    RightI = ipos;
                    RightJ = jpos + 1;
                    LeftValid = jpos > 0;
                    LeftI = ipos;
                    LeftJ = jpos - 1;
                    BackValid = ipos < MapDim - 1;
                    BackI = ipos + 1;
                    BackJ = jpos;
                    break;
                case orEAST:
                    FrontValid = jpos < MapDim - 1;
                    FrontI = ipos;
                    FrontJ = jpos + 1;
                    RightValid = ipos < MapDim - 1;
                    RightI = ipos + 1;
                    RightJ = jpos;
                    LeftValid = ipos > 0;
                    LeftI = ipos - 1;
                    LeftJ = jpos;
                    BackValid = jpos > 0;
                    BackI = ipos;
                    BackJ = jpos -1;
                    break;
                case orWEST:
                    FrontValid = jpos > 0;
                        FrontI = ipos;
                        FrontJ = jpos-1;
                    RightValid = ipos > 0;
                        RightI = ipos - 1;
                        RightJ = jpos;
                    LeftValid = ipos < MapDim - 1;
                        LeftI = ipos + 1;
                        LeftJ = jpos;
                    BackValid = jpos < MapDim - 1;
                        BackI = ipos;
                        BackJ = jpos + 1;
                    break;
                case orSOUTH:
                    FrontValid = ipos < MapDim - 1;
                    FrontI = ipos + 1;
                    FrontJ = jpos;
                    RightValid = jpos > 0;
                    RightI = ipos;
                    RightJ = jpos - 1;
                    LeftValid = jpos < MapDim - 1;
                    LeftI = ipos;
                    LeftJ = jpos + 1;
                    BackValid = ipos > 0;
                    BackI = ipos - 1;
                    BackJ = jpos;
                    break;
            }

            // calculating conditional probabilities P(R|BLOCK) and P(R|VOID) for all neighboring cells
            double pBlockFront = 0, pVoidFront = 0, pBlockRight = 0, pVoidRight = 0;
            double pBlockBack = 0, pVoidBack = 0, pBlockLeft = 0, pVoidLeft = 0;
           

            

            if (FrontValid)
            {
                double R1 = sensors[7] > Bound1 ? Bound1 : sensors[7];
                double R2 = sensors[0] > Bound2 ? Bound2 : sensors[0];
                double R3 = sensors[1] > Bound3 ? Bound3 : sensors[1];
               
                // Calculating p(R1|BLOCK), p(R2|BLOCK) and p(R3|BLOCK)
                double pBlockFront1 = a1 * (R1 - Bound1) * (R1 - Bound1);
                double pBlockFront2 = a2 * (R2 - Bound2) * (R2 - Bound2);
                double pBlockFront3 = a3 * (R3 - Bound3) * (R3 - Bound3);
                // calculating p([R1 R2 R3] | BLOCK) from p(R1|BLOCK), p(R2|BLOCK) and p(R3|BLOCK)
                // as the logic DISJUNCTION of these probabilities (assumming independence)
                pBlockFront = 1 - (1 - pBlockFront1) * (1 - pBlockFront2) * (1 - pBlockFront3);

                // now calculating p(R1|VOID), p(R2|VOID) and p(R3|VOID)
                double pVoidFront1 = a1 * R1 * R1;
                double pVoidFront2 = a2 * R2 * R2;
                double pVoidFront3 = a3 * R3 * R3;
                
                // now calculating p([R1 R2 R3 | VOID) from p(R1 | VOID), p(R2 | VOID) and p(R3 | VOID)
                // as the logic CONJUNCTION of these probabilities (assumming independence)
                pVoidFront = pVoidFront1 * pVoidFront2 * pVoidFront3;

                // retrieving the already stored probability for the front cell: P(BLOCK | Rk-1)= P(BLOCK)
                // and P[VOID | Rk-1] = P(VOID)
                Pk_1BlockFront = BlockProbability[FrontI][FrontJ];
                Pk_1VoidFront = VoidProbability[FrontI][FrontJ];
                    
            }

            if (RightValid)
            {
                double R1 = sensors[1] > Bound1 ? Bound1 : sensors[1];
                double R2 = sensors[2] > Bound1 ? Bound1 : sensors[2];
                double R3 = sensors[3] > Bound3 ? Bound3 : sensors[3];

                // Calculating p(R1|BLOCK), p(R2|BLOCK) and p(R3|BLOCK)
                double pBlockRight1 = a1 * (R1 - Bound1) * (R1 - Bound1);
                double pBlockRight2 = a2 * (R2 - Bound2) * (R2 - Bound2);
                double pBlockRight3 = a3 * (R3 - Bound3) * (R3 - Bound3);           
                // calculating P([R1 R2 R3] | BLOCK) from P(R1|BLOCK), P(R2|BLOCK) and P(R3|BLOCK)
                // as the logic DISJUNCTION of these probabilities
                pBlockRight = 1 - (1 - pBlockRight1) * (1 - pBlockRight2) * (1 - pBlockRight3);
                
                // now calculating p(R1|VOID), p(R2|VOID) and p(R3|VOID)
                double pVoidRight1 = a1 * R1 * R1;
                double pVoidRight2 = a2 * R2 * R2;
                double pVoidRight3 = a3 * R3 * R3;
                
                // now calculating p([R1 R2 R3 | VOID) from p(R1 | VOID), p(R2 | VOID) and p(R3 | VOID)
                // as the logic CONJUNCTION of these probabilities
                pVoidRight = pVoidRight1 * pVoidRight2 * pVoidRight3;

                // retrieving the already stored probability for the right cell (P(BLOCK | Rk-1)= P(BLOCK)
                // and P(VOID) = P(VOID | Rk-1)
                Pk_1BlockRight = BlockProbability[RightI][RightJ];
                Pk_1VoidRight = VoidProbability[RightI][RightJ];
            }

            if (BackValid)
            {
                double R1 = sensors[3] > Bound1 ? Bound1 : sensors[3];
                double R2 = sensors[4] > Bound2 ? Bound2 : sensors[4];
                double R3 = sensors[5] > Bound3 ? Bound3 : sensors[5];

                // Calculating P(R1|BLOCK), P(R2|BLOCK) and P(R3|BLOCK)
                double pBlockBack1 = a1 * (R1 - Bound1) * (R1 - Bound1);
                double pBlockBack2 = a2 * (R2 - Bound2) * (R2 - Bound2);
                double pBlockBack3 = a3 * (R3 - Bound3) * (R3 - Bound3);
                // calculating p([R1 R2 R3] | BLOCK) from p(R1|BLOCK), p(R2|BLOCK) and p(R3|BLOCK)
                // as the logic DISJUNCTION of these probabilities
                pBlockBack = 1 - (1 - pBlockBack1) * (1 - pBlockBack2) * (1 - pBlockBack3);

                // now calculating P(R1|VOID), P(R2|VOID) and P(R3|VOID)
                double pVoidBack1 = a1 * R1 * R1;
                double pVoidBack2 = a2 * R2 * R2;
                double pVoidBack3 = a3 * R3 * R3;
                // now calculating p([R1 R2 R3 | VOID) from p(R1 | VOID), p(R2 | VOID) and p(R3 | VOID)
                // as the logic CONJUNCTION of these probabilities
                pVoidBack = pVoidBack1 * pVoidBack2 * pVoidBack3;

                // retrieving the already stored probability for the back cell (P(BLOCK | Rk-1)= P(BLOCK)
                // and P(VOID) = P(VOID | Rk-1)
                Pk_1BlockBack = BlockProbability[BackI][BackI];
                Pk_1VoidBack = VoidProbability[BackI][BackJ];
            }
            if (LeftValid)
            {
                double R1 = sensors[5] > Bound1 ? Bound1 : sensors[5];
                double R2 = sensors[6] > Bound2 ? Bound2 : sensors[6];
                double R3 = sensors[7] > Bound3 ? Bound3 : sensors[7];

                // Calculating P(R1|BLOCK), P(R2|BLOCK) and P(R3|BLOCK)
                double pBlockLeft1 = a1 * (R1 - Bound1) * (R1 - Bound1);
                double pBlockLeft2 = a2 * (R2 - Bound2) * (R2 - Bound2);
                double pBlockLeft3 = a3 * (R3 - Bound3) * (R3 - Bound3);
                // calculating P([R1 R2 R3] | BLOCK) from P(R1|BLOCK), P(R2|BLOCK) and P(R3|BLOCK)
                // as the logic DISJUNCTION of these probabilities
                pBlockLeft = 1 - (1 - pBlockLeft1) * (1 - pBlockLeft2) * (1 - pBlockLeft3);

                // now calculating p(R1|VOID), p(R2|VOID) and p(R3|VOID)
                double pVoidLeft1 = a1 * R1 * R1;
                double pVoidLeft2 = a2 * R2 * R2;
                double pVoidLeft3 = a3 * R3 * R3;
                // now calculating P([R1 R2 R3 | VOID) from P(R1 | VOID), P(R2 | VOID) and P(R3 | VOID)
                // as the logic CONJUNCTION of these probabilities
                pVoidLeft = pVoidLeft1 * pVoidLeft2 * pVoidLeft3;

                // retrieving the already stored probability for the left cell (P(BLOCK | Rk-1)= P(BLOCK)
                Pk_1BlockLeft = BlockProbability[LeftI][LeftJ];
                Pk_1VoidLeft = VoidProbability[LeftI][LeftJ];
            }
            // now calculating the new conditional probabilities P(BLOCK/R) and P(VOID/R) for all neighboring cells
            double PkBlockFront=0, PkBlockRight=0, PkBlockBack=0, PkBlockLeft=0;
            double PkVoidFront = 0, PkVoidRight = 0, PkVoidBack = 0, PkVoidLeft = 0;
            if (FrontValid)
            {
                // applying Bayes' rule for P(BLOCK | Rk) and P(VOID | Rk)
                PkBlockFront = pBlockFront * Pk_1BlockFront / 
                               (pBlockFront * Pk_1BlockFront + pVoidFront * Pk_1VoidFront);
                PkVoidFront = pVoidFront * Pk_1VoidFront /
                                (pBlockFront * Pk_1BlockFront + pVoidFront * Pk_1VoidFront);
                // updating probability tables
                BlockProbability[FrontI][FrontJ] = PkBlockFront;
                VoidProbability[FrontI][FrontJ] = PkVoidFront;
                // updating map
                if (PkBlockFront >= PkVoidFront)
                    Map[FrontI][FrontJ] = cellBLOCK;
                else
                    Map[FrontI][FrontJ] = cellVOID;
            }
            if (RightValid)
            {
                // applying Bayes' rule for P(BLOCK | Rk) and P(VOID | Rk)
                PkBlockRight = pBlockRight * Pk_1BlockRight / 
                              (pBlockRight * Pk_1BlockRight + pVoidRight * Pk_1VoidRight);
                PkVoidRight = pVoidRight * Pk_1VoidRight /
                              (pBlockRight * Pk_1BlockRight + pVoidRight * Pk_1VoidRight);
                // updating probability tables
                BlockProbability[RightI][RightJ] = PkBlockRight;
                VoidProbability[RightI][RightJ] = PkVoidRight;
                // updating map
                if (PkBlockRight >= PkVoidRight)
                    Map[RightI][RightJ] = cellBLOCK;
                else
                    Map[RightI][RightJ] = cellVOID;
            }
            if (BackValid)
            {
                // applying Bayes' rule for P(BLOCK | Rk) and P(VOID | Rk)
                PkBlockBack = pBlockBack * Pk_1BlockBack / 
                             (pBlockBack * Pk_1BlockBack + pVoidBack * Pk_1VoidBack);
                PkVoidBack = pVoidBack * Pk_1VoidBack /
                             (pBlockBack * Pk_1BlockBack + pVoidBack * Pk_1VoidBack);
                // updating probability tables
                BlockProbability[BackI][BackJ] = PkBlockBack;
                VoidProbability[BackI][BackJ] = PkVoidBack;
                // updating map
                if (PkBlockBack >= PkVoidBack)
                    Map[BackI][BackJ] = cellBLOCK;
                else
                    Map[BackI][BackJ] = cellVOID;
            }
            if (LeftValid)
            {
                // applying Bayes' rule for P(BLOCK | Rk) and P(VOID | Rk)
                PkBlockLeft = pBlockLeft * Pk_1BlockLeft / 
                             (pBlockLeft * Pk_1BlockLeft + pVoidLeft * Pk_1VoidLeft);
                PkVoidLeft = pVoidLeft * Pk_1VoidLeft /
                             (pBlockLeft * Pk_1BlockLeft + pVoidLeft * Pk_1VoidLeft);
                // updating probability tables
                BlockProbability[LeftI][LeftJ] = PkBlockLeft;
                VoidProbability[LeftI][LeftJ] = PkVoidLeft;
                // Updating map
                if (PkBlockLeft >= PkVoidLeft)
                    Map[LeftI][LeftJ] = cellBLOCK;
                else
                    Map[LeftI][LeftJ] = cellVOID;
            }



            
        }





        public Boolean inPath(int ipos, int jpos, MapCell[] path, int pathlen)
        {
            int i;
            Boolean found = false;
            i = 0;
            while ((i < pathlen) && (!found))
            {
                found = (path[i].ipos == ipos) && (path[i].jpos == jpos) ? true : found;
                i++;
            }
            return found;
        }
        
        

        // fill the task queue with a number of moves
        // to follow a certain path
        public void path2QueueTasks(APath path, int corientation)
        {
            int i;
            int curOrient = corientation;

            for (i = 0; i < path.pathlen; i++)
                switch (curOrient)
                {
                    case orNORTH:
                        switch (path.amoves[i])
                        {
                            case APath.instGO_NORTH:
                                insertTask(taskGO_FORWARD);
                                break;
                            case APath.instGO_EAST:
                                insertTask(taskTURN_RIGHT);
                                insertTask(taskGO_FORWARD);
                                curOrient = orEAST;
                                break;
                            case APath.instGO_SOUTH:
                                insertTask(taskGO_BACKWARD);
                                break;
                            case APath.instGO_WEST:
                                insertTask(taskTURN_LEFT);
                                insertTask(taskGO_FORWARD);
                                curOrient = orWEST;
                                break;
                        }
                        break;
                    case orEAST:
                        switch (path.amoves[i])
                        {
                            case APath.instGO_NORTH:
                                insertTask(taskTURN_LEFT);
                                insertTask(taskGO_FORWARD);
                                curOrient = orNORTH;
                                break;
                            case APath.instGO_EAST:
                                insertTask(taskGO_FORWARD);
                                break;
                            case APath.instGO_SOUTH:
                                insertTask(taskTURN_RIGHT);
                                insertTask(taskGO_FORWARD);
                                curOrient = orSOUTH;
                                break;
                            case APath.instGO_WEST:
                                insertTask(taskGO_BACKWARD);
                                break;
                        }
                        break;
                    case orSOUTH:
                        switch (path.amoves[i])
                        {
                            case APath.instGO_NORTH:
                                insertTask(taskGO_BACKWARD);
                                break;
                            case APath.instGO_EAST:
                                insertTask(taskTURN_LEFT);
                                insertTask(taskGO_FORWARD);
                                curOrient = orEAST;
                                break;
                            case APath.instGO_SOUTH:
                                insertTask(taskGO_FORWARD);
                                break;
                            case APath.instGO_WEST:
                                insertTask(taskTURN_RIGHT);
                                insertTask(taskGO_FORWARD);
                                curOrient = orWEST;
                                break;
                        }
                        break;
                    case orWEST:
                        switch (path.amoves[i])
                        {
                            case APath.instGO_NORTH:
                                insertTask(taskTURN_RIGHT);
                                insertTask(taskGO_FORWARD);
                                curOrient = orNORTH;
                                break;
                            case APath.instGO_EAST:
                                insertTask(taskGO_BACKWARD);
                                break;
                            case APath.instGO_SOUTH:
                                insertTask(taskTURN_LEFT);
                                insertTask(taskGO_FORWARD);
                                curOrient = orSOUTH;
                                break;
                            case APath.instGO_WEST:
                                insertTask(taskGO_FORWARD);
                                break;
                        }
                        break;
                }
        }
            
       
        

    


        // an A* shortest path algorithm 
        public APath shortestPath(int startIpos, int startJpos, int destIpos, int destJpos,
                                 MapCell[] Visited, int VisitedLength,ref Boolean PathFound)
        {
            int i;
            int  newVisitedLength = VisitedLength+1;
            MapCell[] newVisited = new MapCell[newVisitedLength];
            APath shortestpath = null;
            // finding second robot (if any)
            int rsv2Ipos = -1, rsv2Jpos = -1;
            if (browsingFSM != null)
            {
                rsv2Ipos = browsingFSM.ipos;
                rsv2Jpos = browsingFSM.jpos;
            }
            // first checking whether we are in /out of map bounds
            if ((startIpos<0)||(startIpos>=MapDim)||(startJpos<0)||(startJpos>=MapDim)) 
                PathFound = false;
            else 
                if (Map[startIpos][startJpos]!= cellVOID) 
                PathFound = false;
            else if (inPath(startIpos, startJpos, Visited, VisitedLength))
                    PathFound = false;
                else if ((rsv2Ipos == startIpos) && (rsv2Jpos == startJpos))
                    PathFound = false;
                else
                {

                    // copying map of visited cells
                    for (i = 0; i < VisitedLength; i++)
                        newVisited[i] = Visited[i];
                    // visited cells list copied
                    // now adding current position
                    newVisited[newVisitedLength - 1].ipos = startIpos;
                    newVisited[newVisitedLength - 1].jpos = startJpos;
                    // checking northern cell
                    if ((startIpos - 1 == destIpos) && (startJpos == destJpos))
                    {
                        PathFound = true;
                        shortestpath = new APath(1);
                        shortestpath.amoves[0] = APath.instGO_NORTH;
                    } // checking eastern cell
                    else if ((startIpos == destIpos) && (startJpos + 1 == destJpos))
                    {
                        PathFound = true;
                        shortestpath = new APath(1);
                        shortestpath.amoves[0] = APath.instGO_EAST;
                    } // checking southern cell
                    else if ((startIpos + 1 == destIpos) && (startJpos == destJpos))
                    {
                        PathFound = true;
                        shortestpath = new APath(1);
                        shortestpath.amoves[0] = APath.instGO_SOUTH;
                    }// checking western cell
                    else if ((startIpos == destIpos) && (startJpos - 1 == destJpos))
                    {
                        PathFound = true;
                        shortestpath = new APath(1);
                        shortestpath.amoves[0] = APath.instGO_WEST;
                    }// recursively checking paths now...
                    else
                    {
                        // sorting out euclidean distances of neighboring cells
                        double[] distances = new double[4]; // 0-north, 1-east, 2-south, 3-west
                        for (i = 0; i < 4; i++)
                            distances[i] = Math.Sqrt(Math.Pow((double)(startIpos - destIpos), 2) + Math.Pow((double)(startJpos - destJpos), 2));
                        // now repeating until we find a valid path
                        Boolean pfound = false;
                        PathFound = false;
                        int counter = 0;
                        double mindistance = 100000;
                        int mindistanceindex = 0;
                        APath newpath;
                        while ((!pfound) && (counter < 4))
                        {
                            mindistance = 100000; // large enough value for minimum euclidean distance
                            for (i = 0; i < 4; i++)
                                if ((distances[i] < mindistance) && (distances[i] >= 0))
                                {
                                    mindistance = distances[i];
                                    mindistanceindex = i;
                                }
                            if (mindistance < 100000)
                            {
                                distances[mindistanceindex] = -1;

                                switch (mindistanceindex)
                                {
                                    case 0: // North
                                        newpath = shortestPath(startIpos - 1, startJpos, destIpos, destJpos,
                                            newVisited, newVisitedLength, ref pfound);
                                        if (pfound)
                                        {
                                            PathFound = true;
                                            shortestpath = new APath(1);
                                            shortestpath.amoves[0] = APath.instGO_NORTH;
                                            shortestpath.appendPath(newpath);
                                        }
                                        break;
                                    case 1: // East
                                        newpath = shortestPath(startIpos, startJpos + 1, destIpos, destJpos,
                                            newVisited, newVisitedLength, ref pfound);
                                        if (pfound)
                                        {
                                            PathFound = true;
                                            shortestpath = new APath(1);
                                            shortestpath.amoves[0] = APath.instGO_EAST;
                                            shortestpath.appendPath(newpath);
                                        }
                                        break;
                                    case 2: // South
                                        newpath = shortestPath(startIpos + 1, startJpos, destIpos, destJpos,
                                            newVisited, newVisitedLength, ref pfound);
                                        if (pfound)
                                        {
                                            PathFound = true;
                                            shortestpath = new APath(1);
                                            shortestpath.amoves[0] = APath.instGO_SOUTH;
                                            shortestpath.appendPath(newpath);
                                        }
                                        break;
                                    case 3: // South
                                        newpath = shortestPath(startIpos, startJpos - 1, destIpos, destJpos,
                                            newVisited, newVisitedLength, ref pfound);
                                        if (pfound)
                                        {
                                            PathFound = true;
                                            shortestpath = new APath(1);
                                            shortestpath.amoves[0] = APath.instGO_WEST;
                                            shortestpath.appendPath(newpath);
                                        }
                                        break;
                                }

                            }
                            counter++;
                        }
                    } // recursion else ends
                } // big else ends

            return shortestpath;
        
        }


        // the following method return the positions of the sonar sensors
        // in 8 points of the horizon (going counter-clock wise starting from
        // zero pointing at the north
        public static int[] getSensorPermutation(int orient)
        {
            int[] perm = new int[8];

            switch (orient)
            {
                case orNORTH:
                    perm[0] = 0;
                    perm[1] = 1;
                    perm[2] = 2;
                    perm[3] = 3;
                    perm[4] = 4;
                    perm[5] = 5;
                    perm[6] = 6;
                    perm[7] = 7;
                    break;
                case orEAST:
                    perm[0] = 6;
                    perm[1] = 7;
                    perm[2] = 0;
                    perm[3] = 1;
                    perm[4] = 2;
                    perm[5] = 3;
                    perm[6] = 4;
                    perm[7] = 5;
                    break;
                case orSOUTH:
                    perm[0] = 4;
                    perm[1] = 5;
                    perm[2] = 6;
                    perm[3] = 7;
                    perm[4] = 0;
                    perm[5] = 1;
                    perm[6] = 2;
                    perm[7] = 3;
                    break;
                case orWEST:
                    perm[0] = 2;
                    perm[1] = 3;
                    perm[2] = 4;
                    perm[3] = 5;
                    perm[4] = 6;
                    perm[5] = 7;
                    perm[6] = 0;
                    perm[7] = 1;
                    break;
            }

            return perm;
        }


        // finds an estimate of the angular offset of the robot (unsigned)
        // after a 90 turn
        public static double getAngleOffset(LMMobileRobot thecart, int curOrientation, int prevOrientation)
        {
            
            double[] Snew = new double[8];
            double[] Sprev = new double[8];
            int i;


            // storing sensor index permutations for both new and previous orientations
            int[] prevPerm = getSensorPermutation(prevOrientation);
            int[] newPerm = getSensorPermutation(curOrientation);

            for (i = 0; i < 8; i++)
            {
                Snew[i] = (double)thecart.SonarArray[newPerm[i]];
                Sprev[i] = (double)thecart.PrevSonarArray[prevPerm[i]];
            }

            // initially applying a Kalman Filter to the measurements
            // a zero-input vector for the process
            double[] U = new double[] { 0, 0, 0, 0, 0, 0, 0, 0 };
            double[] B = new double[] { 0, 0, 0, 0, 0, 0, 0, 0 };
            double[] W0 = new double[] { 0, 0, 0, 0, 0, 0, 0, 0 };
            double[] V0 = new double[] { 2, 2, 2, 2, 2, 2, 2, 2 };

            double[][] A = new double[][] { new double[] { 1, 0, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 1, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 1, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 1, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 1, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 1, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 1, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 0, 1 }
                                            };
            double[][] H = new double[][] { new double[] { 1, 0, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 1, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 1, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 1, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 1, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 1, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 1, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 0, 1 }
                                            };
            double[][] Q = new double[][] { new double[] { 5, 0, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 5, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 5, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 5, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 5, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 5, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 5, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 0, 5 }
                                            };
            double[][] R = new double[][] { new double[] { 2, 0, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 2, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 2, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 2, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 2, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 2, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 2, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 0, 2 }
                                            };
            KalmanFilter filter = new KalmanFilter(8, A, B, W0, H, V0, Q, R, Sprev);

            // retrieving the measurement noise-free estimate by the Kalman filter,
            // in order to use it to calculate the gradient
            double[] Snest = filter.getNewEstimate(Snew, U);


            // now computing the average cosine of the triangles 
            double anglesum = 0;
            int count = 0;
            for (i=0; i<8; i++)
                if (Snest[i] < Snew[i])
                {
                    count++;
                    anglesum += 180*Math.Acos((Snest[i]+11) / (Snew[i]+11))/Math.PI;
                }

            double average = (count == 0) ? 1 : anglesum / count;
            
            
            return average;
        }



        // finds an estimate on the robots position gradient based on the sonar readings
        public static double[] getGradient(LMMobileRobot thecart, int curOrientation, int prevOrientation)
        {
            double[] gradient = new double[] { 0, 0 };
            double D1x, D2x, D3x, D4x, D5x, D6x;
            double D1y, D2y, D3y, D4y, D5y, D6y;

            double[] Snew = new double[8];
            double[] Sprev = new double[8];
            int i;
           

            // storing sensor index permutations for both new and previous orientations
            int[] prevPerm = getSensorPermutation(prevOrientation);
            int[] newPerm = getSensorPermutation(curOrientation);

            for (i = 0; i < 8; i++)
            {
                Snew[i] = (double)thecart.SonarArray[newPerm[i]];
                Sprev[i] = (double)thecart.PrevSonarArray[prevPerm[i]];
            }

            // initially applying a Kalman Filter to the measurements
            // a zero-input vector for the process
            double[] U = new double[] { 0, 0, 0, 0, 0, 0, 0, 0 };
            double[] B = new double[] { 0, 0, 0, 0, 0, 0, 0, 0 };
            double[] W0 = new double[] { 0, 0, 0, 0, 0, 0, 0, 0 };
            double[] V0 = new double[] { 2, 2, 2, 2, 2, 2, 2, 2 };

            double[][] A = new double[][] { new double[] { 1, 0, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 1, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 1, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 1, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 1, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 1, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 1, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 0, 1 }
                                            };
            double[][] H = new double[][] { new double[] { 1, 0, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 1, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 1, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 1, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 1, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 1, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 1, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 0, 1 }
                                            };
            double[][] Q = new double[][] { new double[] { 5, 0, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 5, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 5, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 5, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 5, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 5, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 5, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 0, 5 }
                                            };
            double[][] R = new double[][] { new double[] { 2, 0, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 2, 0, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 2, 0, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 2, 0, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 2, 0, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 2, 0, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 2, 0 },
                                            new double[] { 0, 0, 0, 0, 0, 0, 0, 2 }
                                            };
            KalmanFilter filter = new KalmanFilter(8, A, B, W0, H, V0, Q, R, Sprev);

            // retrieving the measurement noise-free estimate by the Kalman filter,
            // in order to use it to calculate the gradient
            double[] Snest = filter.getNewEstimate(Snew, U);

            

            // computing Gx
            D1x = Sprev[1] * Math.Cos(Math.PI / 4) - Snest[1] * Math.Cos(Math.PI / 4);
            //if (Math.Abs(D1x) > sup) D1x = 0;
            D2x = Sprev[2] - Snest[2];
            //if (Math.Abs(D2x) > sup) D2x = 0;
            D3x = Sprev[3] * Math.Cos(Math.PI / 4) - Snest[3] * Math.Cos(Math.PI / 4);
            //if (Math.Abs(D3x) > sup) D3x = 0;
            D4x = -(Sprev[5] * Math.Cos(Math.PI / 4) - Snest[5] * Math.Cos(Math.PI / 4));
            //if (Math.Abs(D4x) > sup) D4x = 0;
            D5x = -(Sprev[6] - Snest[6]);
            //if (Math.Abs(D5x) > sup) D5x = 0;
            D6x = -(Sprev[7] * Math.Cos(Math.PI / 4) - Snest[7] * Math.Cos(Math.PI / 4));
            //if (Math.Abs(D6x) > sup) D6x = 0;

            gradient[0] = (D1x + D2x + D3x + D4x + D5x + D6x) / 6;
            

            // computing Gy
            D1y = Sprev[0] - Snest[0];
            //if (Math.Abs(D1y) > sup) D1y = 0;
            D2y = Sprev[1] * Math.Cos(Math.PI / 4) - Snest[1] * Math.Cos(Math.PI / 4);
            //if (Math.Abs(D2y) > sup) D2y = 0;
            D3y = -(Sprev[3] * Math.Cos(Math.PI / 4) - Snest[3] * Math.Cos(Math.PI / 4));
            //if (Math.Abs(D3y) > sup) D3y = 0;
            D4y = -(Sprev[4] - Snest[4]);
            //if (Math.Abs(D4y) > sup) D4y = 0;
            D5y = -(Sprev[5] * Math.Cos(Math.PI / 4) - Snest[5] * Math.Cos(Math.PI / 4));
            //if (Math.Abs(D5y) > sup) D5y = 0;
            D6y = Sprev[7] * Math.Cos(Math.PI / 4) - Snest[7] * Math.Cos(Math.PI / 4);
            //if (Math.Abs(D6y) > sup) D6y = 0;

            gradient[1] = (D1y + D2y + D3y + D4y + D5y + D6y) / 6;
            

            return gradient;
        }


        public static void drawMap(MappingFSM mapFSM, System.Windows.Forms.Panel panel)
        {

            int xdim = mapFSM.MapDim;
            int ydim = mapFSM.MapDim;

            int mpanelxdim = panel.Width;
            int mpanelydim = panel.Height;

            Graphics mg = panel.CreateGraphics();

            mg.Clear(System.Drawing.Color.White);
            // now calculating cell width and height
            int cellwidth = (int)(mpanelxdim / xdim);
            int cellheight = (int)(mpanelydim / ydim);
            // cell height-width done

            // now drawing maze (with robot)
            int i, j;
            for (i = 0; i < ydim; i++)
                for (j = 0; j < xdim; j++)
                {
                    int x1, y1, x2 = 0, y2 = 0;
                    x1 = j * cellwidth;
                    y1 = i * cellheight;
                    if (mapFSM.Map[i][j] != MappingFSM.cellVOID)
                    {
                        SolidBrush bluebrush = new SolidBrush(mapFSM.Map[i][j] == MappingFSM.cellBLOCK ? Color.Blue : Color.Gray);
                        System.Drawing.Pen blackpen = new Pen(System.Drawing.Color.Black);
                        mg.FillRectangle(bluebrush, x1, y1, cellwidth, cellheight);
                        mg.DrawRectangle(blackpen, x1, y1, cellwidth, cellheight);
                        blackpen.Dispose();
                        bluebrush.Dispose();
                    }

                }

            // now drawing the movile robot
            MappingFSM.drawRobot(mapFSM, mg, mapFSM.jpos, mapFSM.ipos, mapFSM.orientation, 
                                 panel, Color.Yellow);
            // now drawing the other robot (if any)
            if (mapFSM.browsingFSM != null)
                MappingFSM.drawRobot(mapFSM, mg, mapFSM.browsingFSM.jpos, mapFSM.browsingFSM.ipos,
                                    mapFSM.browsingFSM.orientation, panel, Color.Green);
        }

        public static void drawRobot(MappingFSM mapFSM, Graphics mg, 
                                     int robotx, int roboty, int robotorient,
                                     System.Windows.Forms.Panel panel,
                                Color color)
        {
           
            int xdim = mapFSM.MapDim;
            int ydim = mapFSM.MapDim;
            int mpanelxdim = panel.Width;
            int mpanelydim = panel.Height;

            int cellwidth = (int)(mpanelxdim / xdim);
            int cellheight = (int)(mpanelydim / ydim);

            int x1 = robotx * cellwidth;
            int y1 = roboty * cellheight;
            int x2 = 0, y2 = 0;

            SolidBrush robobrush = new SolidBrush(color);
            mg.FillEllipse(robobrush, x1, y1, cellwidth, cellheight);
            robobrush.Dispose();
            // denoting orientation
            System.Drawing.Pen redpen = new Pen(System.Drawing.Color.Red);
            switch (robotorient)
            {
                case MappingFSM.orNORTH: x1 = robotx * cellwidth + (int)(cellwidth / 2);
                    y1 = roboty * cellheight + (int)(cellheight / 2);
                    x2 = robotx * cellwidth + (int)(cellwidth / 2);
                    y2 = roboty * cellheight;
                    break;

                case MappingFSM.orEAST: x1 = robotx * cellwidth + (int)(cellwidth / 2);
                    y1 = roboty * cellheight + (int)(cellheight / 2);
                    x2 = (robotx + 1) * cellwidth;
                    y2 = roboty * cellheight + (int)(cellheight / 2);
                    break;

                case MappingFSM.orSOUTH: x1 = robotx * cellwidth + (int)(cellwidth / 2);
                    y1 = roboty * cellheight + (int)(cellheight / 2);
                    x2 = robotx * cellwidth + (int)(cellwidth / 2);
                    y2 = (roboty + 1) * cellheight;
                    break;

                case MappingFSM.orWEST: x1 = robotx * cellwidth + (int)(cellwidth / 2);
                    y1 = roboty * cellheight + (int)(cellheight / 2);
                    x2 = robotx * cellwidth;
                    y2 = roboty * cellheight + (int)(cellheight / 2);
                    break;
            }
            mg.DrawLine(redpen, x1, y1, x2, y2);
            redpen.Dispose();

        }


        public static void deleteRobot(MappingFSM mapFSM, int robotx, int roboty, Graphics mg, 
                                       System.Windows.Forms.Panel panel)
        {
            int xdim = mapFSM.MapDim;
            int ydim = mapFSM.MapDim;

            int mpanelxdim = panel.Width;
            int mpanelydim = panel.Height;

            int cellwidth = (int)(mpanelxdim / xdim);
            int cellheight = (int)(mpanelydim / ydim);

            int x1 = robotx * cellwidth;
            int y1 = roboty * cellheight;
            int x2 = 0, y2 = 0;

            SolidBrush whitebrush = new SolidBrush(Color.White);
            mg.FillEllipse(whitebrush, x1, y1, cellwidth, cellheight);
            whitebrush.Dispose();
        }


    }
}
