// *************************** RobosapienV2 class for Arduino Mega ***********************
// *                                                                                     *
// *                      George Terzakis (Tsambikos) 2010                               *
// *                                                                                     *
// *                     A class to control the Robosapien V2                            *
// *                   and gain feedback from most of the sensors                        *
// ***************************************************************************************


#include "robosapienv2.h"



RobosapienV2::RobosapienV2(HardwareSerial* RadioComm, byte node_id, HardwareSerial* CMUCamComm) {
  
  // initializing the CMUCAM sensor
  pCamera = new CMUCAM(CMUCamComm);
  
  pRadioPort = RadioComm;
  
  // Initializing radio serial port
  pNetDevice = new EZRoboNetDevice(node_id, t_Node_Robot, pRadioPort);
   
  // Configuring I/O
  // Digital I/O
  pinMode(pinCOMMAND_LINE, OUTPUT); // rsv2 command line 
  pinMode(pinLEFT_FOOT_REAR_BUMPER, INPUT); 
  pinMode(pinLEFT_FOOT_FRONT_BUMPER, INPUT);
  pinMode(pinRIGHT_FOOT_REAR_BUMPER, INPUT);
  pinMode(pinRIGHT_FOOT_FRONT_BUMPER, INPUT);
  pinMode(pinLEFT_HAND_BUMPER, INPUT);
  pinMode(pinRIGHT_HAND_BUMPER, INPUT);
  pinMode(pinLEFT_HAND_PICKUP, INPUT);
  pinMode(pinRIGHT_HAND_PICKUP, INPUT);
  // Digital I/O Done. Analog inputs should be ok
  
  // raising command line pin to high
  digitalWrite(pinCOMMAND_LINE, HIGH);
  
  // clearing the mic and bumper sensor trigger flags
  sensorRight_Mic_Triggered = 0;
  sensorLeft_Mic_Triggered = 0;
  sensorRight_Foot_Front_Triggered = 0;
  sensorRight_Foot_Rear_Triggered = 0;
  sensorLeft_Foot_Front_Triggered = 0;
  sensorLeft_Foot_Rear_Triggered = 0;
  sensorRight_Hand_Triggered = 0;
  sensorLeft_Hand_Triggered = 0;
  
  // setting up analog reference
  analogReference(DEFAULT);
  
  // ability execution flags
  flagAbilityExecuting = flagAbilityDone = false;
  
}


// command transmission method
void RobosapienV2::transmitCommand(unsigned int cmd) {
  byte command[2];
  command[0] = lowByte(cmd);
  command[1] = highByte(cmd);
  int i, j;
  // sending start bit
  digitalWrite(13, LOW);
  delayMicroseconds(transmissionSTART);
  // sending payload from MS bit to LS bit
  for (i=1; i>=0; i--)
    for (j=7; j>=0; j--) {
      if ((i==1)&&(j>3)) {
          digitalWrite(13, LOW);
          delayMicroseconds(transmissionBREAK);
      }
        else {
          int bit = (command[i] >> j) & 0x1;
          int delay_time;
          digitalWrite(pinCOMMAND_LINE, HIGH);
          if (bit == 1) delay_time = transmissionONE;
           else delay_time = transmissionZERO; 
          delayMicroseconds(delay_time);
          // now creating a break
          digitalWrite(pinCOMMAND_LINE, LOW);
          delayMicroseconds(transmissionBREAK);
        }
    }
  // Going back high
  digitalWrite(pinCOMMAND_LINE, HIGH);
}

// poll all sensors and assign their values to class members
void RobosapienV2::pollSensors() {
  sensorLeft_Foot_Rear_Bumper = digitalRead(pinLEFT_FOOT_REAR_BUMPER);
  sensorLeft_Foot_Front_Bumper = digitalRead(pinLEFT_FOOT_FRONT_BUMPER);
  sensorRight_Foot_Rear_Bumper = digitalRead(pinRIGHT_FOOT_REAR_BUMPER);
  sensorRight_Foot_Front_Bumper = digitalRead(pinRIGHT_FOOT_FRONT_BUMPER);
  sensorLeft_Hand_Bumper = digitalRead(pinLEFT_HAND_BUMPER);
  sensorRight_Hand_Bumper = digitalRead(pinRIGHT_HAND_BUMPER);
  sensorLeft_Hand_Pickup = digitalRead(pinLEFT_HAND_PICKUP);
  sensorRight_Hand_Pickup = digitalRead(pinRIGHT_HAND_PICKUP);
  
  sensorCamera_HSD = analogRead(pinCAMERA_HSD);
  
  sensorCamera_RST = analogRead(pinCAMERA_RST);
  
  sensorCamera_SK = analogRead(pinCAMERA_SK);
  
  sensorCamera_RDY = analogRead(pinCAMERA_RDY);
  
  sensorLeft_Shoulder_Pot = analogRead(pinLEFT_SHOULDER_POT);
  
  sensorRight_Shoulder_Pot = analogRead(pinRIGHT_SHOULDER_POT);
  
  sensorLeft_Mic = analogRead(pinLEFT_MIC);
   
  sensorRight_Mic = analogRead(pinRIGHT_MIC);
  
  sensorLeft_Head_IR = analogRead(pinLEFT_HEAD_IR);
  delay(10);
  sensorCenter_Head_IR = analogRead(pinCENTER_HEAD_IR);
  delay(10);
  sensorRight_Head_IR = analogRead(pinRIGHT_HEAD_IR);
  
  // polling wrist encoders
  sensorRight_Wrist_Encoder = getRightWristEncoder();
  
  sensorLeft_Wrist_Encoder = getLeftWristEncoder();
  
  // assigning values to trigger flags
  if (sensorRight_Mic > 100) 
      sensorRight_Mic_Triggered = 1;
  if (sensorLeft_Mic > 100) 
      sensorLeft_Mic_Triggered = 1;
  if (sensorRight_Foot_Front_Bumper == 0) 
      sensorRight_Foot_Front_Triggered =  1 ; // normally closed sensor
  if (sensorRight_Foot_Rear_Bumper == 0)
      sensorRight_Foot_Rear_Triggered =  1; // normally closed
  if (sensorLeft_Foot_Front_Bumper == 0)
      sensorLeft_Foot_Front_Triggered = 1; // normally closed
  if (sensorLeft_Foot_Rear_Bumper == 0)
      sensorLeft_Foot_Rear_Triggered = 1;   // normally closed
  if (sensorRight_Hand_Bumper == 1)
      sensorRight_Hand_Triggered = 1;           // normally open
  if (sensorLeft_Hand_Bumper == 1)
      sensorLeft_Hand_Triggered = 1;
  
}

//Left foot rear bumper only
uint8_t RobosapienV2::getLeftFootRearBumper() {
  sensorLeft_Foot_Rear_Bumper = digitalRead(pinLEFT_FOOT_REAR_BUMPER);

  return sensorLeft_Foot_Rear_Bumper;
}  

// Left foot front bumper only
uint8_t RobosapienV2::getLeftFootFrontBumper() {
  sensorLeft_Foot_Front_Bumper = digitalRead(pinLEFT_FOOT_FRONT_BUMPER);
  
  return sensorLeft_Foot_Front_Bumper;
}

//Right foot rear bumper only
uint8_t RobosapienV2::getRightFootRearBumper() {
  sensorRight_Foot_Rear_Bumper = digitalRead(pinRIGHT_FOOT_REAR_BUMPER);

  return sensorRight_Foot_Rear_Bumper;
}  

// Right foot front bumper only
uint8_t RobosapienV2::getRightFootFrontBumper() {
  sensorRight_Foot_Front_Bumper = digitalRead(pinRIGHT_FOOT_FRONT_BUMPER);
  
  return sensorRight_Foot_Front_Bumper;
}

// Left Hand Bumper only
uint8_t RobosapienV2::getLeftHandBumper() {
  sensorLeft_Hand_Bumper = digitalRead(pinLEFT_HAND_BUMPER);
  
  return sensorLeft_Hand_Bumper;
}

// Right Hand Bumper Only
uint8_t RobosapienV2::getRightHandBumper() {
  sensorRight_Hand_Bumper = digitalRead(pinRIGHT_HAND_BUMPER);
  
  return sensorRight_Hand_Bumper;
}

// Left hand Pickup Only
uint8_t RobosapienV2::getLeftHandPickup() {
  sensorLeft_Hand_Pickup = digitalRead(pinLEFT_HAND_PICKUP);
  
  return sensorLeft_Hand_Pickup;
}

// Right Hand Pickup only
uint8_t RobosapienV2::getRightHandPickup() {
   sensorRight_Hand_Pickup = digitalRead(pinRIGHT_HAND_PICKUP);
   
   return sensorRight_Hand_Pickup;
}

// Left wrist encoder
uint8_t RobosapienV2::getLeftWristEncoder() {
  uint8_t bit0 = digitalRead(pinLEFT_WRIST_ENCODER_BIT0);
  uint8_t bit1 = digitalRead(pinLEFT_WRIST_ENCODER_BIT1);

  uint8_t value = bit0 + 2*bit1;
  
  return value;
}

// right wrist encoder
uint8_t RobosapienV2::getRightWristEncoder() {
  uint8_t bit0 = digitalRead(pinRIGHT_WRIST_ENCODER_BIT0);
  uint8_t bit1 = digitalRead(pinRIGHT_WRIST_ENCODER_BIT1);

  uint8_t value = bit0 + 2*bit1;
  
  return value;
}


// Camera HSD only
uint16_t RobosapienV2::getCameraHSD() {
  sensorCamera_HSD = analogRead(pinCAMERA_HSD);
  
  return sensorCamera_HSD;
}

// Camera RST
uint16_t RobosapienV2::getCameraRST() {
  sensorCamera_RST = analogRead(pinCAMERA_RST);
  
  return sensorCamera_RST;
}

// Camera RDY
uint16_t RobosapienV2::getCameraRDY() {
  sensorCamera_RDY = analogRead(pinCAMERA_RDY);
  
  return sensorCamera_RDY;
}

// Camera SK
uint16_t RobosapienV2::getCameraSK() {
  sensorCamera_SK = analogRead(pinCAMERA_SK);
  
  return sensorCamera_SK;
}

// Left Shoulder pot only
uint16_t RobosapienV2::getLeftShoulderPot() {
  sensorLeft_Shoulder_Pot = analogRead(pinLEFT_SHOULDER_POT);
  
  return sensorLeft_Shoulder_Pot;
}

// Right Shoulder Pot
uint16_t RobosapienV2::getRightShoulderPot() {
  sensorRight_Shoulder_Pot = analogRead(pinRIGHT_SHOULDER_POT);
  
  return sensorRight_Shoulder_Pot;
}

// Left Head IR
uint16_t RobosapienV2::getLeftHeadIR() {
  sensorLeft_Head_IR = analogRead(pinLEFT_HEAD_IR);
  
  return sensorLeft_Head_IR;
}


// Center head IR
uint16_t RobosapienV2::getCenterHeadIR() {
  sensorCenter_Head_IR = analogRead(pinCENTER_HEAD_IR);
  
  return sensorCenter_Head_IR;
}

// Right Hand IR
uint16_t RobosapienV2::getRightHeadIR() {
  sensorRight_Head_IR = analogRead(pinRIGHT_HEAD_IR);
  
  return sensorRight_Head_IR;
}

// Left Mic
uint16_t RobosapienV2::getLeftMic() {
  sensorLeft_Mic = analogRead(pinLEFT_MIC);
  
  return sensorLeft_Mic;
}

// Right Mic
uint16_t RobosapienV2::getRightMic() {
  sensorRight_Mic = analogRead(pinRIGHT_MIC);
  
  return sensorRight_Mic;
}


// here comes methods concerning the radio comms

// handle Messages "picks" up packets available in the
// NetDevice and triggers the appropriate actions
void RobosapienV2::handleMessages() {
  int i;
  
  //1. Handling Ability Execution (skipping if the ability has not been finished)
  if (flagAbilityExecuting) 
      if (millis()-AbExecStartTime >= AbilityExecTime*1000) {
          // stoping the robosapien
          if (AbilityShouldStop)
              useAbility(abSTOP);
          // creating a Command Done packet
          EZPacket* donepack = createCommandDonePacket(CommandSenderID);
          if (pNetDevice->pushOutboundPacket(donepack)) 
              flagAbilityExecuting = false;
          else 
            EZRoboNetDevice::disposePacket(donepack);
            
      }
  
    
  
  //Serial.println("Still looping");
  
  // 1. polling the sensors
  pollSensors();
  
  // 2. handling the CMUCAM's state
  
    if (pCamera->flagAbstractionDataReady) { // a new scanline is ready for transmission
    Serial.println("Hooked the scanline flag");
    
           EZPacket* pack = pCamera->createAbstractionChunkPacket(pNetDevice->NodeID, CMUCAMDataReceiver);
        
          // must now send the packet
          if (pNetDevice->pushOutboundPacket(pack)) {
            Serial.println("Pushing the frame packet "); 
            pCamera->flagAbstractionDataReady = false;
          }
          else
             EZRoboNetDevice::disposePacket(pack);
      
    } 
          
       
        
      
  
  // 3. checking the NetDevice for available packets in order to act
  if (pNetDevice->packetsAvailable()>0) { // packet is available
    EZPacket* pack = pNetDevice->getNextAvailablePacket();
    
    // unwrapping the packet
    CommandMsg* msg = (CommandMsg*)malloc(sizeof(CommandMsg));
    msg->robot = (t_Robot)pack->Message[0];
    msg->Cmd = (RobotCmd)pack->Message[1];
    msg->ParamsLength = pack->Message[2] + pack->Message[3]*256;
    msg->CmdParams = (byte*)malloc(msg->ParamsLength);
    
    int i;
    for (i=0; i<msg->ParamsLength; i++)
      msg->CmdParams[i] = pack->Message[4+i];
    // packet unwrapped
    
    // acting upon the content of the message
    switch(msg->Cmd) {
      case rc_SendCMUCAMAbstractionData: 
                                   // designating image data receiver as the packet sender
                                   CMUCAMDataReceiver = pack->SenderID;
                                   Serial.println("Got an Abstraction request!");
                                   pCamera->getAbstractionData();
                               
                                   
                              break;
      case rc_ManualCommand: // forcing us to trigger an ability
                            // raising execution flag
                            flagAbilityExecuting = true;
                            // storing the id of the command sender
                            CommandSenderID = pack->SenderID;
                            // storing execution time
                            AbilityExecTime = AbilityTime[msg->CmdParams[0]];
                            
                            // raising a stoppage flag
                            AbilityShouldStop = AbilityStop[msg->CmdParams[0]];
                            
                            // marking start of execution time stamp
                            AbExecStartTime = millis();
                            // executing ability
                            useAbility(msg->CmdParams[0]);
                            break;
      case rc_CommandDone:   // 
                            break;
      case rc_SendSensors:  // must add code that sends sensory data through the network
                            EZPacket* sensorpack = createSensorDataPacket(pack->SenderID);
                            
                            // clearing the sensor triggers
                            sensorRight_Mic_Triggered = 0;
                            sensorLeft_Mic_Triggered = 0;
                            sensorRight_Foot_Front_Triggered = 0;
                            sensorRight_Foot_Rear_Triggered = 0;
                            sensorLeft_Foot_Front_Triggered = 0;
                            sensorLeft_Foot_Rear_Triggered = 0;
                            sensorRight_Hand_Triggered = 0;
                            sensorLeft_Hand_Triggered = 0;
                            
                            // pushing the packet into the outbound queue
                            if (!pNetDevice->pushOutboundPacket(sensorpack))
                                  EZRoboNetDevice::disposePacket(sensorpack);  
                           break;
    } 
    // disposing the Message
    disposeCommandMsg(msg);
    // disposing the packet
    EZRoboNetDevice::disposePacket(pack);
   }
  
}

// decodes a manual command fron the network and riggers the corresponding ability
void RobosapienV2::useAbility(byte ability) {
  uint16_t rsv2CommandCode = Ability2CodeMap[ability];
  
  transmitCommand(rsv2CommandCode);
}


// create a Command Message containing sensory data
CommandMsg* RobosapienV2::createSensorDataMessage() {
  CommandMsg* msg = (CommandMsg*)malloc(sizeof(CommandMsg));
  msg->robot = t_RobosapienV2;
  msg->Cmd = rc_SensorData;
  msg->ParamsLength = 40;
  // allocating the parameter section. Should be 40 bytes
  msg->CmdParams = (byte*)malloc(40);
  
  // assigning sensor readings
  msg->CmdParams[0] = sensorLeft_Foot_Front_Bumper;
  msg->CmdParams[1] = sensorLeft_Foot_Rear_Bumper;
  msg->CmdParams[2] = sensorRight_Foot_Front_Bumper;
  msg->CmdParams[3] = sensorRight_Foot_Rear_Bumper;
  msg->CmdParams[4] = sensorLeft_Hand_Bumper;
  msg->CmdParams[5] = sensorRight_Hand_Bumper;
  msg->CmdParams[6] = sensorLeft_Hand_Pickup;
  msg->CmdParams[7] = sensorRight_Hand_Pickup;

  msg->CmdParams[8] = sensorLeft_Wrist_Encoder;
  msg->CmdParams[9] = sensorRight_Wrist_Encoder;

  // analog sensors
  msg->CmdParams[10] = lowByte(sensorCamera_HSD);
  msg->CmdParams[11] = highByte(sensorCamera_HSD);
  
  msg->CmdParams[12] = lowByte(sensorCamera_RST);
  msg->CmdParams[13] = highByte(sensorCamera_RST);
  msg->CmdParams[14] = lowByte(sensorCamera_RDY);
  msg->CmdParams[15] = highByte(sensorCamera_RDY);
  msg->CmdParams[16] = lowByte(sensorCamera_SK);
  msg->CmdParams[17] = highByte(sensorCamera_SK);
  
  msg->CmdParams[18] = lowByte(sensorLeft_Shoulder_Pot);
  msg->CmdParams[19] = highByte(sensorLeft_Shoulder_Pot);
  msg->CmdParams[20] = lowByte(sensorRight_Shoulder_Pot);
  msg->CmdParams[21] = highByte(sensorRight_Shoulder_Pot);

  msg->CmdParams[22] = lowByte(sensorLeft_Mic);
  msg->CmdParams[23] = highByte(sensorLeft_Mic);
  msg->CmdParams[24] = lowByte(sensorRight_Mic);
  msg->CmdParams[25] = highByte(sensorRight_Mic);

  msg->CmdParams[26] = lowByte(sensorLeft_Head_IR);
  msg->CmdParams[27] = highByte(sensorLeft_Head_IR);
  msg->CmdParams[28] = lowByte(sensorCenter_Head_IR);
  msg->CmdParams[29] = highByte(sensorCenter_Head_IR);
  msg->CmdParams[30] = lowByte(sensorRight_Head_IR);
  msg->CmdParams[31] = highByte(sensorRight_Head_IR);

  // the mic and bumper sensor triggers
  msg->CmdParams[32] = sensorLeft_Mic_Triggered;
  msg->CmdParams[33] = sensorRight_Mic_Triggered;
  msg->CmdParams[34] = sensorLeft_Foot_Rear_Triggered;
  msg->CmdParams[35] = sensorLeft_Foot_Front_Triggered;
  msg->CmdParams[36] = sensorRight_Foot_Rear_Triggered;
  msg->CmdParams[37] = sensorRight_Foot_Front_Triggered;
  msg->CmdParams[38] = sensorLeft_Hand_Triggered;
  msg->CmdParams[29] = sensorRight_Hand_Triggered;
  
  return msg;
}

// create a packet containing sensory data
EZPacket* RobosapienV2::createSensorDataPacket(byte receiverid) {
  EZPacket* pack = (EZPacket*)malloc(sizeof(EZPacket));
  
  pack->SenderID = pNetDevice->NodeID;
  pack->ReceiverID = receiverid;
  pack->SenderNodeType = t_Node_Robot;
  pack->MessageLength = 40+4;
  pack->Message = (byte*)malloc(pack->MessageLength);
  
  CommandMsg* msg = createSensorDataMessage();  
  
  
  pack->Message[0] = (byte)msg->robot;
  pack->Message[1] = (byte)msg->Cmd;
  pack->Message[2] = lowByte(msg->ParamsLength);
  pack->Message[3] = highByte(msg->ParamsLength);
  
  
  
  int i; 
    for (i=0; i<40; i++)
       pack->Message[4+i] = msg->CmdParams[i];
  // disposing the message
  disposeCommandMsg(msg);

   return pack;
}

CommandMsg* RobosapienV2::createCommandDoneMessage() {
  int i;
  CommandMsg* msg = (CommandMsg*)malloc(sizeof(CommandMsg));
  msg->robot = t_HandmadeCart;
  msg->Cmd = rc_CommandDone;
  msg->ParamsLength = 1;
  // allocating the parameter section. Should be 16 bytes
  msg->CmdParams = (byte*)malloc(1); //1 byte parameters
  msg->CmdParams[0] = 0; // ignored anyway
  
  return msg;

}

EZPacket* RobosapienV2::createCommandDonePacket(byte receiverid) {
  EZPacket* pack = (EZPacket*)malloc(sizeof(EZPacket));

  pack->SenderID = pNetDevice->NodeID;
  pack->ReceiverID = receiverid;
  pack->SenderNodeType = t_Node_Robot;
  pack->MessageLength = 5;
  pack->Message = (byte*)malloc(pack->MessageLength);

  CommandMsg* msg = createCommandDoneMessage();  


  pack->Message[0] = (byte)msg->robot;
  pack->Message[1] = (byte)msg->Cmd;
  pack->Message[2] = lowByte(msg->ParamsLength);
  pack->Message[3] = highByte(msg->ParamsLength);
  pack->Message[4] = msg->CmdParams[0];
  
  disposeCommandMsg(msg);
  
  return pack;
}

