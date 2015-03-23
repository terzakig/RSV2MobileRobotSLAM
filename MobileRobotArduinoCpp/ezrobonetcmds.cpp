#include "ezrobonetcmds.h"


// convert Manual Command Paramaters to bytes
byte* convertManualCmdParams2Bytes(ManualCmdParams* params) {
  byte* rawparams = (byte*)malloc(2); 
  //command low byte
  rawparams[0] = params->Command & 0x0FF;
  // command high byte
  rawparams[1] = (params->Command >> 8) & 0x0FF;
  
  return rawparams;
}



// convert CMU line parameters to bytes
byte* convertCMUCAMLineParams2Bytes(CMUCAMLineParams* params) {
  uint16_t imgwidth = params->ImageWidth;
  byte* rawparams = (byte*)malloc(imgwidth*3+6);
  
  rawparams[0] = params->ScanLineIndex & 0x0FF;
  rawparams[1] = (params->ScanLineIndex >> 8) & 0x0FF;
  
  rawparams[2] = params->ImageWidth & 0x0FF;
  rawparams[3] = (params->ImageWidth >> 8) & 0x0FF;
  
  rawparams[4] = params->ImageHeight & 0x0FF;
  rawparams[5] = (params->ImageHeight >> 8) & 0x0FF;
  
  // adding Channels
  int i, index = 6;
  for (i=0; i<params->ImageWidth; i++) {
    rawparams[index+i*3] = params->RedChannel[i];
    rawparams[index+i*3+1] = params->GreenChannel[i];
    rawparams[index+i*3+2] = params->BlueChannel[i];
  }
    
    return rawparams;
}


// disposal of a Command Message sructure
void disposeCommandMsg(CommandMsg* msg) {
  if (msg!=NULL) {
    if (msg->CmdParams!=NULL)
      free(msg->CmdParams);
    free(msg);
  }
}


// disposal of Manual Command Parameter structure
void disposeManualCmdParams(ManualCmdParams* params) {
  free(params);
}


// disposal of CMUCAM Scan Line parameters structure
void disposeCMUCAMLineParams(CMUCAMLineParams* params) {
  // first disposing the channel data
  if (params->RedChannel!=NULL)
      free(params->RedChannel);
  if (params->GreenChannel!=NULL)
      free(params->GreenChannel);
  if (params->BlueChannel!=NULL)
      free(params->BlueChannel);
  // disposing the structure
  free(params);
}

// convert raw bytes to a CMUCAM Line Parameters structure
CMUCAMLineParams* convertBytes2CMUCAMLineParams(byte* rawparms) {
  
   CMUCAMLineParams* parms = (CMUCAMLineParams*)malloc(sizeof(CMUCAMLineParams));

   parms->ScanLineIndex = rawparms[0] + rawparms[1] * 256;

   parms->ImageWidth = rawparms[2] + rawparms[3] * 256;

   parms->ImageHeight = rawparms[4] + rawparms[5] * 256;

   parms->RedChannel = (byte*)malloc(parms->ImageWidth);
   parms->GreenChannel = (byte*)malloc(parms->ImageWidth);
   parms->BlueChannel = (byte*)malloc(parms->ImageWidth);


   int i, index = 5;
   
   for (i=0; parms->ImageWidth*3; i++) {
     parms->RedChannel[i] = rawparms[index+i*3];
     parms->GreenChannel[i] = rawparms[index+i*3+1];
     parms->BlueChannel[i] = rawparms[index+i*3+2];
   }

    return parms;
 }


// convert manual Command Parameters (ability id)
ManualCmdParams* convertBytes2ManualCmdparams(byte* rawparms) {
        
  ManualCmdParams* parms = (ManualCmdParams*)malloc(sizeof(ManualCmdParams));
  parms->Command = rawparms[0] + rawparms[1] * 256;

  return parms;
}

// convert bytes to a command message structure
CommandMsg* convertBytes2CommandMsg(byte* rawmsg) {
    
  CommandMsg* msg = (CommandMsg*)malloc(sizeof(CommandMsg));
  msg->robot = (t_Robot)rawmsg[0];
  msg->Cmd = (RobotCmd)rawmsg[1];
  msg->ParamsLength = rawmsg[2] + rawmsg[3] * 256;

  int i;
  msg->CmdParams = (byte*)malloc(msg->ParamsLength);
  for (i = 0; i < msg->ParamsLength; i++)
      msg->CmdParams[i] = rawmsg[4 + i];

  return msg;
}


// convert a Command Message to raw bytes
byte* convertCommandMsg2Bytes(CommandMsg* msg) {
  int i;
  int msglength = msg->ParamsLength + 4;
  byte* rawmsg = (byte*)malloc(msglength);
  rawmsg[0] = (byte)msg->robot;
  rawmsg[1] = (byte)msg->Cmd;
  rawmsg[2] = msg->ParamsLength & 0x0FF;
  rawmsg[3] = (msg->ParamsLength >> 8) & 0x0FF;
  
  for (i=0; i<msg->ParamsLength; i++)
    rawmsg[4+i] = msg->CmdParams[i];
    
  return rawmsg;
}
