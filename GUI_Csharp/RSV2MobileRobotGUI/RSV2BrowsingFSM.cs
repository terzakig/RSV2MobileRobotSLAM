using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class RSV2BrowsingFSM
    {
        // orientation constants
        public const int orNORTH = 0;
        public const int orEAST = 1;
        public const int orSOUTH = 2;
        public const int orWEST = 3;

        // map constants
        public const char cellUNKNOWN = 'X';
        public const char cellBLOCK = '*';
        public const char cellVOID = ' ';
        

        // a mapping FSM (containing the map)
        public MappingFSM mappingFSM;
        // the robot
        RobosapienV2 Robosapien;


        // position and orientation
        public int ipos, jpos, orientation;

        // a task queue
        public int[] TaskQueue;
        public int TaskQueueLength;
        public const int MAX_QUEUE_LENGTH = 500;

        // task constants
        public const int taskGO_FORWARD = 0;
        public const int taskGO_BACKWARD = 1;
        public const int taskTURN_RIGHT = 2;
        public const int taskTURN_LEFT = 3;
        public const int taskGO_BACKLEFT = 4;
        public const int taskGO_BACKRIGHT = 5;
        public const int taksGO_FORWARDLEFT = 6;
        public const int taskGO_FORWARDRIGHT = 7;
        public const int taskDISSAPOINTED = 8;
        public const int taskHUG = 9;

        // FSM state
        public int state;

        public const int stIdle = 0;
        public const int stTasksExecuting = 1;
        public const int stAbilityExecuting = 2;
        public const int stSensorDataTransmission = 3;
        
        // current task being executed
        public int CurrentTask;

        // last known position of the cart
        public int LastKnownCartIpos, LastKnownCartJpos;

        // Current destination cell
        public int CurrentDestIpos, CurrentDestJpos;


        // constructor
        public RSV2BrowsingFSM(MappingFSM mapfsm, RobosapienV2 robo,
                                int robIpos, int robJpos, int robOrient)
        {
            mappingFSM = mapfsm;
            
            Robosapien = robo;
            ipos = robIpos;
            jpos = robJpos;
            orientation = robOrient;

            LastKnownCartIpos = mappingFSM.ipos;
            LastKnownCartJpos = mappingFSM.jpos;

            // creating the task queue
            TaskQueue = new int[MAX_QUEUE_LENGTH];
            TaskQueueLength = 0;

            // registering browsing FSM in mapping FSM
            mappingFSM.registerBrowingFSM(this);

            // now setting up our Last Known location inside the mappingh FSM
            mappingFSM.LastKnownRsv2Ipos = robIpos;
            mappingFSM.LastKnownRsv2Jpos = robJpos;

        }


        public void resetBrowsingFSM(int robIpos, int robJpos, int robOrient)
        {
            ipos = robIpos;
            jpos = robJpos;
            orientation = robOrient;

            // clearing robot location cell
            mappingFSM.Map[ipos][jpos] = cellVOID;

            // clearing task queue
            TaskQueueLength = 0;

            // last known coordinates of the mobile robot
            LastKnownCartIpos = mappingFSM.ipos;
            LastKnownCartJpos = mappingFSM.jpos;

            // registering to the mappingFSM
            mappingFSM.registerBrowingFSM(this);

            // now setting up our Last Known location inside the mappingh FSM
            mappingFSM.LastKnownRsv2Ipos = robIpos;
            mappingFSM.LastKnownRsv2Jpos = robJpos;
        }

        public int removeTask()
        {
            int val = 0;
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


        // an A* shortest path algorithm 
        public APath shortestPath(int startIpos, int startJpos, int destIpos, int destJpos,
                                 MapCell[] Visited, int VisitedLength, ref Boolean PathFound)
        {
            int i;
            int newVisitedLength = VisitedLength + 1;
            MapCell[] newVisited = new MapCell[newVisitedLength];
            APath shortestpath = null;
            // first checking whether we are in /out of map bounds
            if ((startIpos < 0) || (startIpos >= mappingFSM.MapDim) || (startJpos < 0) || (startJpos >= mappingFSM.MapDim))
                PathFound = false;
            else
                if (mappingFSM.Map[startIpos][startJpos] == MappingFSM.cellBLOCK)
                    PathFound = false;
                else
                    if (inPath(startIpos, startJpos, Visited, VisitedLength))
                        PathFound = false;
                    else if ((mappingFSM.ipos == startIpos) && (mappingFSM.jpos == startJpos))
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


        public Boolean close2Cart()
        {
            Boolean left = (ipos == mappingFSM.ipos) && (jpos == mappingFSM.jpos - 1);
            Boolean right = (ipos == mappingFSM.ipos) && (jpos == mappingFSM.jpos + 1);
            Boolean up = (ipos == mappingFSM.ipos - 1) && (jpos == mappingFSM.jpos);
            Boolean down = (ipos == mappingFSM.ipos + 1) && (jpos == mappingFSM.jpos);

            return left || right || down || up;
        }



        public void browsingStep(ref System.Windows.Forms.TextBox[] texts)
        {
            if ((!close2Cart()) || ( TaskQueueLength > 0) )
                switch (state)
                {
                    case stIdle: 
                        // call to the planner
                        planner();
                        // now switching to execution
                        state = stTasksExecuting;
                        break;
                    
                    case stSensorDataTransmission:
                        if (Robosapien.flagSensorDataAcquired)
                        {
                            Robosapien.fillSensorTexts(texts);
                            // Now checking if we bumped into something
                            if ((Robosapien.sensorLeft_Hand_Triggered == 1) && (Robosapien.sensorRight_Hand_Triggered == 1))
                            {
                                // re-assigning last move
                                insertPriorityTask(CurrentTask);
                                
                                // inserting correction in queue
                                insertPriorityTask(taskGO_BACKWARD);
                                state = stTasksExecuting;
                            } else
                                if (Robosapien.sensorLeft_Hand_Triggered == 1)
                                {
                                    // re-assigning last move
                                    insertPriorityTask(CurrentTask);
                                
                                    insertPriorityTask(taskGO_BACKLEFT);
                                    state = stTasksExecuting;
                                } else
                                    if (Robosapien.sensorRight_Hand_Triggered == 1)
                                    {
                                        // re-assigning last move
                                        insertPriorityTask(CurrentTask);
                                
                                        insertPriorityTask(taskGO_BACKRIGHT);
                                        state = stTasksExecuting;
                                    }
                                    else // ability was excuted without interruption. finding new position and orientation
                                    {
                                        // updating last known position of the robosapien
                                        // in the mapping FSM
                                        mappingFSM.LastKnownRsv2Ipos = ipos;
                                        mappingFSM.LastKnownRsv2Jpos = jpos;

                                        // finding new orientation and position
                                        switch (orientation)
                                        {
                                            case orNORTH:
                                                switch (CurrentTask)
                                                {
                                                    case taskGO_FORWARD: orientation = orNORTH;

                                                        ipos--;

                                                        break;
                                                    case taskGO_BACKWARD: orientation = orNORTH;

                                                        ipos++;

                                                        break;
                                                    case taskTURN_LEFT: orientation = orWEST;
                                                        break;
                                                    case taskTURN_RIGHT: orientation = orEAST;
                                                        break;
                                                }
                                                break;
                                            case orSOUTH:
                                                switch (CurrentTask)
                                                {
                                                    case taskGO_FORWARD: orientation = orSOUTH;

                                                        ipos++;

                                                        break;
                                                    case taskGO_BACKWARD: orientation = orSOUTH;

                                                        ipos--;

                                                        break;
                                                    case taskTURN_LEFT: orientation = orEAST;
                                                        break;
                                                    case taskTURN_RIGHT: orientation = orWEST;
                                                        break;
                                                }
                                                break;
                                            case orEAST:
                                                switch (CurrentTask)
                                                {
                                                    case taskGO_FORWARD: orientation = orEAST;

                                                        jpos++;

                                                        break;
                                                    case taskGO_BACKWARD: orientation = orEAST;

                                                        jpos--;

                                                        break;
                                                    case taskTURN_LEFT: orientation = orNORTH;
                                                        break;
                                                    case taskTURN_RIGHT: orientation = orSOUTH;
                                                        break;
                                                }
                                                break;
                                            case orWEST:
                                                switch (CurrentTask)
                                                {
                                                    case taskGO_FORWARD: orientation = orWEST;

                                                        jpos--;

                                                        break;
                                                    case taskGO_BACKWARD: orientation = orWEST;

                                                        jpos++;

                                                        break;
                                                    case taskTURN_LEFT: orientation = orSOUTH;
                                                        break;
                                                    case taskTURN_RIGHT: orientation = orNORTH;
                                                        break;
                                                }
                                                break;
                                        } // end  switch (orientation)

                                        

                                        state = stIdle;
                                    }
                        }
                        break;
                    case stTasksExecuting:
                        if (TaskQueueLength == 0)
                        {
                            state = stSensorDataTransmission;
                            Robosapien.requestSensorData();
                        }
                        else
                        {
                            CurrentTask = removeTask();

                            state = stAbilityExecuting;

                            switch (CurrentTask)
                            {
                                case taskGO_FORWARD: Robosapien.useAbility(t_RSV2Ability.abWALK_FORWARD);
                                    break;
                                case taskGO_BACKWARD: Robosapien.useAbility(t_RSV2Ability.abWALK_BACKWARD);
                                    break;
                                case taskTURN_LEFT: Robosapien.useAbility(t_RSV2Ability.abTURN_LEFT);
                                    break;
                                case taskTURN_RIGHT: Robosapien.useAbility(t_RSV2Ability.abTURN_RIGHT);
                                    break;
                                case taskGO_BACKLEFT: Robosapien.useAbility(t_RSV2Ability.abWALK_BACKWARDLEFT);
                                    break;
                                case taskGO_BACKRIGHT: Robosapien.useAbility(t_RSV2Ability.abWALK_BACKWARDRIGHT);
                                    break;
                                case taksGO_FORWARDLEFT: Robosapien.useAbility(t_RSV2Ability.abWALK_FORWARDLEFT);
                                    break;
                                case taskGO_FORWARDRIGHT: Robosapien.useAbility(t_RSV2Ability.abWALK_FORWARDRIGHT);
                                    break;
                                case taskDISSAPOINTED: Robosapien.useAbility(t_RSV2Ability.abSPARE_CHANGE);
                                    break;
                                case taskHUG: Robosapien.useAbility(t_RSV2Ability.abHUG);
                                    break;
                            }
                        }

                        break;

                    case stAbilityExecuting:
                        if (Robosapien.flagAbilityDone)
                        {
                            // reseting the flag
                            Robosapien.flagAbilityDone = false;
                            // requesting sensor data
                            Robosapien.requestSensorData();
                            // changing state
                            state = stSensorDataTransmission;
                        } // end if
                        break;
                } // end state switch



        }




        public Boolean isNeighborCart(int i, int j) {
            Boolean result = false;
            
                if ((i-1 == mappingFSM.ipos) && (j == mappingFSM.jpos)) 
                    result = true;
            
            
                if ((i+1 == mappingFSM.ipos) && (j == mappingFSM.jpos))
                    result = true;
            
            
                if ((i == mappingFSM.ipos) && (j-1 == mappingFSM.jpos))
                    result = true;
            
            
                if ((i == mappingFSM.ipos) && (j+1 == mappingFSM.jpos))
                    result = true;
            
            return result;
        }


        public void planner()
        {
            int i, j;
            if ((mappingFSM.ipos != LastKnownCartIpos) || (mappingFSM.jpos != LastKnownCartJpos))
            {
                // the mobile robot has changed position, 
                // thus we may need to follow a different path to get to it

                // updating last known cart position
                LastKnownCartIpos = mappingFSM.ipos;
                LastKnownCartJpos = mappingFSM.jpos;

                // finding the closest reachable cell next to the cart

                int minpathlen = 1000000, mini, minj;
                APath minpath = null;

                for (i = 0; i < mappingFSM.MapDim; i++)
                    for (j = 0; j < mappingFSM.MapDim; j++)
                        if (mappingFSM.Map[i][j] == cellVOID)
                            if (isNeighborCart(i, j))
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
                // clearing the task queue anyway
                TaskQueueLength = 0;

                if (minpath == null) // no path close to the cell found    
                    // adding a reaction
                    insertTask(taskDISSAPOINTED);
                else // planning to move to the cell
                    path2QueueTasks(minpath, orientation);


            }
            else
                if ((!close2Cart()) && (TaskQueueLength == 0))
                {
                    // finding the closest reachable cell next to the cart

                    int minpathlen = 1000000, mini, minj;
                    APath minpath = null;

                    for (i = 0; i < mappingFSM.MapDim; i++)
                        for (j = 0; j < mappingFSM.MapDim; j++)
                            if (mappingFSM.Map[i][j] == cellVOID)
                                if (isNeighborCart(i, j))
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
                    // clearing the task queue anyway
                    TaskQueueLength = 0;

                    if (minpath == null) // no path close to the cell found    
                        // adding a reaction
                        insertTask(taskDISSAPOINTED);
                    else // planning to move to the cell
                        path2QueueTasks(minpath, orientation);
                }
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

            insertTask(taskHUG);
        }






    }
}
