
#ifndef EZROBONETCMDS_H
#define EZROBONETCMDS_H

#include "WProgram.h"

// list of possible commands to/from the robot 
typedef enum {
               rc_ManualCommand,
               rc_CommandDone,
               rc_SendSensors,
               rc_SendCMUCAMAbstractionData,
               rc_CMUCAMAbstractionData,
               rc_SensorData,
               rc_FireSonarArray,
               rc_SonarFired}RobotCmd;

// types of robots
typedef enum {
              t_RobosapienV2,
              t_HandmadeCart,
              t_Station       }t_Robot;

// A command Message. Should be contained in the payload of the packet
typedef struct  {
  t_Robot robot;
  RobotCmd Cmd;
  uint16_t ParamsLength;
  byte* CmdParams;
} CommandMsg;
  
// Manual Command Parameters (contains the low and high byte of the command)
typedef struct {
  uint16_t Command;
 
}ManualCmdParams;



// CMUCAM scanned line parameters in RGB
typedef struct {
  uint16_t ScanLineIndex;
  uint16_t ImageWidth;
  uint16_t ImageHeight;
  
  byte* RedChannel;
  byte* GreenChannel;
  byte* BlueChannel;
}CMUCAMLineParams;

// convert a Command Message to raw bytes
byte* convertCommandMsg2Bytes(CommandMsg* msg);

// convert Manual Command Paramaters to bytes
byte* convertManualCmdParams2Bytes(ManualCmdParams* params);


// convert CMU line parameters to bytes
byte* convertCMUCAMLineParams2Bytes(CMUCAMLineParams* params);


// disposal of a Command Message sructure
void disposeCommandMsg(CommandMsg* msg);


// disposal of Manual Command Parameter structure
void disposeManualCmdParams(ManualCmdParams* params);


// disposal of CMUCAM Scan Line parameters structure
void disposeCMUCAMLineParams(CMUCAMLineParams* params);


// convert raw bytes to a CMUCAM Line Parameters structure
CMUCAMLineParams* convertBytes2CMUCAMLineparams(byte* rawparms);


// convert manual Command Parameters (ability id)
ManualCmdParams* convertBytes2ManualCmdparams(byte* rawparms);

// convert bytes to a command message structure
CommandMsg* convertBytes2CommandMsg(byte* rawmsg);



#endif

