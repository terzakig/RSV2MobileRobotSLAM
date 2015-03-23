// ***************************************************
// A class that implements all local functionality of a drone mobile robot
// (from performing motion primitives in a controlled fashion, to firing sensors and obtaining frame captures from the CMUCAM2)
//
// The robot acts as a drone of a roboentity running in a statiuonary PC and uses EZRoboNet to receive commands
//
// Typically, the robot does Grid based SLAM, but the roboentity can use it in other ways such as, train a perceptron to control motion
//
//              George Terzakis 2010


#include "lmmobilerobot.h"



// Voltatile variables (used by the interrupts)
volatile float Reference1, Reference2;
volatile float Feedback1, Feedback2;   // motor encoder feedback (accumulated revolutions)

volatile float PlantVoltage1, PlantVoltage2;  // -7.2 to +7.2V
// Encoder use
volatile int counter1,  counter3 ;
volatile byte cha1state,  cha3state, chb1state, chb3state;
volatile byte newcha1state, newcha3state, newchb1state, newchb3state;

// ***************** volatiles done


// **************************** Interrupt Serveice Roytines **************************************
// encoder 3 ISR (left side motors)
void encoder3ISR() {

  newcha3state = digitalRead(pinENCODER3_CHA);

  if (newcha3state != cha3state) {
    cha3state = newcha3state;

    if (PlantVoltage2>=0) counter3++;
    else counter3--;
  }  
  Feedback2 = (1.0*counter3)/64.0;



} 

/*
void encoder3ISR() {
 
 newcha3state = digitalRead(pinENCODER3_CHA);
 newchb3state = digitalRead(pinENCODER3_CHB);
 if (PlantVoltage2>0) {
 if (newcha3state != cha3state) {
 cha3state = newcha3state;
 
 counter3++;
 }
 } else {
 if (newchb3state != chb3state) {
 chb3state = newchb3state;
 counter3--;
 }
 }  
 
 Feedback2 = (1.0*counter3)/64.0;
 }
 */
// encoder 1 ISR (right side motors)
void encoder1ISR() {

  newcha1state = digitalRead(pinENCODER1_CHA);
  // newchb1state = digitalRead(pinENCODER1_CHB);

  if (newcha1state != cha1state) {
    cha1state = newcha1state;
    if (PlantVoltage1>=0) 
      counter1++; 
    else  
      counter1--;
  }
  Feedback1 = (1.0*counter1)/64.0; 

}

/*  void encoder1ISR() {
 
 newcha1state = digitalRead(pinENCODER1_CHA);
 newchb1state = digitalRead(pinENCODER1_CHB);
 
 if (PlantVoltage1>=0) {
 if (newcha1state != cha1state) {
 cha1state = newcha1state;
 counter1++; 
 }
 } else {
 if (newchb1state != chb1state) {
 chb1state = newchb1state; 
 counter1--;
 }
 }
 Feedback1 = (1.0*counter1)/64.0; 
 
 } */





// ********************************** ISRs Done *******************************************************




LMMobileRobot::LMMobileRobot(HardwareSerial* RadioComm, byte node_id, HardwareSerial* CMUCamComm) {

  // **************** initializing I/O ***************
  // motor pins setup
  pinMode(pinRIGHT_MOTORS, OUTPUT);
  pinMode(pinLEFT_MOTORS, OUTPUT);

  // sonar pin direction setup
  pinMode(pinSONAR_INIT, OUTPUT);
  pinMode(pinSONAR_ECHO, INPUT);

  // Sonar MUX Address pins setup  
  pinMode(pinSONAR_ADDRESS0, OUTPUT);
  pinMode(pinSONAR_ADDRESS1, OUTPUT);
  pinMode(pinSONAR_ADDRESS2, OUTPUT);

  // Encoder pins setup
  pinMode(pinENCODER1_CHA, INPUT);
  pinMode(pinENCODER1_CHB, INPUT);
  pinMode(pinENCODER1_DIR, INPUT);

  pinMode(pinENCODER2_CHA, INPUT);
  pinMode(pinENCODER2_CHB, INPUT);
  pinMode(pinENCODER2_DIR, INPUT);

  pinMode(pinENCODER3_CHA, INPUT);
  pinMode(pinENCODER3_CHB, INPUT);
  pinMode(pinENCODER3_DIR, INPUT);

  pinMode(pinENCODER4_CHA, INPUT);
  pinMode(pinENCODER4_CHB, INPUT);
  pinMode(pinENCODER4_DIR, INPUT);


  digitalWrite(pinRIGHT_MOTORS, LOW);
  digitalWrite(pinLEFT_MOTORS, LOW);

  digitalWrite(pinSONAR_INIT, LOW);
  digitalWrite(pinSONAR_ADDRESS0, LOW);
  digitalWrite(pinSONAR_ADDRESS1, LOW);
  digitalWrite(pinSONAR_ADDRESS2, LOW);

  // setting analog reference to common ground
  analogReference(DEFAULT);

  // setting up sampling period for rotation
  TRmil = 5; // 5 ms

  // stopping motors
  stopMotors();

  // **************** I/O initialized ***********************

  // ability execution flags
  flagAbilityExecuting = flagAbilityDone = false;
  // Initializing controllers
  initControllers(0.001, 0.002,  70, 4.7, -20);


  // initializing the CMUCAM sensor
  pCamera = new CMUCAM(CMUCamComm);

  // assigning r/f serial port
  pRadioPort = RadioComm;

  // Initializing radio serial port
  pNetDevice = new EZRoboNetDevice(node_id, t_Node_Robot, pRadioPort);
}





void LMMobileRobot::initControllers(float clp1, float clp2, int PeriodMil, float Kmot, float Pmot) {

  PlantVoltage1 = PlantVoltage2 = 0; 

  Kmotor = Kmot;
  Pmotor = Pmot;

  Tmil = PeriodMil;    //  period in milliseconds (used to clock the controller's step)

  T = (1.0*Tmil)/1000.0; // calculating period in seconds (will be used in calculations)


  Reference1 = Reference2 = 0;
  p1 = clp1;
  p2 = clp2;


  // calculating state feedback gains
  f1 = exp(Pmotor*T)-p1*p2;
  f2 = p1+p2-(1+exp(Pmotor*T));


  // calculating feed-forward gain
  K = (1-p1)*(1-p2)/(Kmotor*(1-exp(Pmotor*T)));
  // calculating observer coefficients
  n1 = (exp(Pmotor*T)-c1*c2)/(Kmotor*(1-exp(Pmotor*T))*exp(Pmotor*T));
  n2 = (1+exp(Pmotor*T)-(c1+c2))/(Kmotor*(1-exp(Pmotor*T)));
  // calculating observer state matrix L
  L[0][0] = 0;
  L[0][1] = 1-Kmotor*(1-exp(Pmotor*T))*n1;
  L[1][0] = -exp(Pmotor*T);
  L[1][1] = 1+exp(Pmotor*T) - Kmotor*(1-exp(Pmotor*T))*n2;

  // initializing all observed current and previous states to 0
  m1x1cur = m1x2cur = m1x1prev = m1x2prev = 0;
  m2x1cur = m2x2cur = m2x1prev = m2x2prev = 0;


  counter1 = counter3 = 0;
  Feedback1 = Feedback2 = 0;

  ControllerExecuting = false;

}

void LMMobileRobot::resetControllers() {
  // detaching interrupts
  detachInterrupts();  
  // stopping motors
  stopMotors(); 
  // initializing all observed current and previous states to 0

  m1x1cur = m1x2cur = m1x1prev = m1x2prev = 0;
  m2x1cur = m2x2cur = m2x1prev = m2x2prev = 0;

  PlantVoltage1 = PlantVoltage2 = 0;

  counter1 = counter3 = 0;
  Feedback1 = Feedback2 = 0;

  ControllerExecuting = false;
}


void LMMobileRobot::startRotation(float angle) {    // angle in degrees
  float offset1, offset2;
  resetControllers();

  // attaching interrupts
  attachInterrupts();

  // figuring out the reference values

    if (angle>=0) { // positive is couter-clockwise
    offset1 = -(angle/180.0)*PI*CART_RADIUS;
    offset2 = (angle/180.0)*PI*CART_RADIUS;
  } 
  else {
    offset1 = (abs(angle)/180.0)*PI*CART_RADIUS;
    offset2 = -(abs(angle)/180.0)*PI*CART_RADIUS;
  } 
  Reference1 = offset1/(2*PI*WHEEL_RADIUS);
  Reference2 = offset2/(2*PI*WHEEL_RADIUS);

  if (abs(Reference1)>0.6) ErrorMargin1 = 0.13;
  else ErrorMargin1 = 0.1;
  if (abs(Reference2)>0.6) ErrorMargin2 = 0.13;
  else ErrorMargin2 = 0.1;

  ControllerPrevStep = millis();
  ControllerExecuting = true;
}

void LMMobileRobot::startLinearMotion(float offset) { // offset in cms
  resetControllers();
  // attaching interrupts
  attachInterrupts();  
  // figuring-out the refernce values
  Reference1 = -offset/(2*PI*WHEEL_RADIUS);
  Reference2 = -offset/(2*PI*WHEEL_RADIUS);

  if (abs(Reference1)>0.6) ErrorMargin1 = 0.3;
  else ErrorMargin1 = 0.2;
  if (abs(Reference2)>0.6) ErrorMargin2 = 0.3;
  else ErrorMargin2 = 0.2;



  ControllerPrevStep = millis();
  ControllerExecuting = true;
}

void LMMobileRobot::updateObservedStates() {
  // right wheels state updates
  m1x1cur = m1x2prev;
  m1x2prev = m1x2cur;
  m1x2cur = Feedback1/(Kmotor*(1-exp(Pmotor*T)));

  // Left wheels state updates
  m2x1cur = m2x2prev;
  m2x2prev = m2x2cur;
  m2x2cur = Feedback2/(Kmotor*(1-exp(Pmotor*T)));


}


void LMMobileRobot::driveMotors(float voltage1, float voltage2) {
  float duty1, duty2;
  float LowVoltageThreshold = 3.0;
  float UpperVoltageThreshold = 6;
  int pulseLength1, pulseLength2, pulseduration2, pulseduration1;
  float abvolt;

  if (voltage1>=0) { // going positive forward
    if (voltage1 >= UpperVoltageThreshold) duty1 = UpperVoltageThreshold/7.2;
    else if (voltage1 >= LowVoltageThreshold) duty1 = voltage1/7.2;
    else duty1 = LowVoltageThreshold/7.2;

    pulseLength1 = int(duty1*500.0);
    pulseduration1 = 1400+pulseLength1;
  } 
  else {
    abvolt = abs(voltage1);
    if (abvolt > UpperVoltageThreshold) duty1 = UpperVoltageThreshold/7.2;
    else if (abvolt >= LowVoltageThreshold) duty1 = abvolt/7.2;
    else duty1 = LowVoltageThreshold/7.2;

    pulseLength1 = int(duty1*500.0);
    pulseduration1 = 1400-pulseLength1;
  }

  if (voltage2>=0) { // going positive forward
    if (voltage2>UpperVoltageThreshold) duty2 = UpperVoltageThreshold/7.2;
    else if (voltage2 >= LowVoltageThreshold) duty2 = voltage2/7.2;
    else duty2 = LowVoltageThreshold/7.2;

    pulseLength2 = (int)(duty2*500.0);
    pulseduration2 = 1400+pulseLength2;
  } 
  else {
    abvolt = abs(voltage2);
    if (abvolt > UpperVoltageThreshold) duty2 = UpperVoltageThreshold/7.2;
    else if (abvolt >= LowVoltageThreshold) duty2 = abvolt/7.2;
    else duty2 = LowVoltageThreshold/7.2;

    pulseLength2 = int(duty2*500.0);
    pulseduration2 = 1400-pulseLength2;
  }

  //stopMotors();

  digitalWrite(pinRIGHT_MOTORS, HIGH);
  delayMicroseconds(pulseduration1);
  digitalWrite(pinRIGHT_MOTORS, LOW);

  digitalWrite(pinLEFT_MOTORS, HIGH);
  delayMicroseconds(pulseduration2);
  digitalWrite(pinLEFT_MOTORS, LOW);

}


void LMMobileRobot::stopMotors() {
  digitalWrite(pinRIGHT_MOTORS, LOW);
  digitalWrite(pinLEFT_MOTORS, LOW);
  delayMicroseconds(200);
  digitalWrite(pinRIGHT_MOTORS, HIGH);
  digitalWrite(pinLEFT_MOTORS, HIGH);
  delayMicroseconds(1400);
  digitalWrite(pinLEFT_MOTORS, LOW);
  digitalWrite(pinRIGHT_MOTORS, LOW);
  delay(3);
  // Do it again!
  digitalWrite(pinRIGHT_MOTORS, HIGH);
  digitalWrite(pinLEFT_MOTORS, HIGH);
  delayMicroseconds(1400);
  digitalWrite(pinLEFT_MOTORS, LOW);
  digitalWrite(pinRIGHT_MOTORS, LOW);

}

void LMMobileRobot::controllerAction() {

  if (ControllerExecuting) 
    if (((abs(Feedback1-Reference1)<=ErrorMargin1)||
      (abs(Feedback2-Reference2)<=ErrorMargin2)||
      (abs(Feedback1) >= abs(Reference1))||
      (abs(Feedback2) >= abs(Reference1)) )&&
      ((abs(Kmotor*(1-exp(Pmotor*T))*(m1x2cur-m1x2prev))<0.2)||
      (abs(Kmotor*(1-exp(Pmotor*T))*(m2x2cur-m2x2prev))<0.2))) {
      stopMotors();
      resetControllers(); 
      // handling flags
      flagAbilityExecuting = false;
      flagAbilityDone = true;
    } 
    else { 
      unsigned long stepDifference = millis() - ControllerPrevStep;
      if (stepDifference>=Tmil) { // execute controller
        ControllerPrevStep = millis();
        // updating the observed states
        updateObservedStates();    
        // Now computing the appropriate input to be fed to the plant
        PlantVoltage1 = K*Reference1+f1*m1x1prev+f2*m1x2prev;
        PlantVoltage2 = K*Reference2+f1*m2x1prev+f2*m2x2prev;



        // driving the motors

        driveMotors(PlantVoltage1, PlantVoltage2);

      }
    }
}



void LMMobileRobot::attachInterrupts() {

  cha1state = digitalRead(pinENCODER1_CHA);
  chb1state = digitalRead(pinENCODER1_CHB);
  attachInterrupt(5, encoder1ISR, RISING); 
  // encoder3 interrupt
  cha3state = digitalRead(pinENCODER3_CHA);
  chb3state = digitalRead(pinENCODER3_CHB);
  attachInterrupt(3, encoder3ISR, RISING); 
}


void LMMobileRobot::detachInterrupts() {
  detachInterrupt(5);
  detachInterrupt(3);
}



// fire transducers
void LMMobileRobot::fireSonarArray() {
  byte i;
  byte bit0, bit1, bit2;
  unsigned long starttime, starttimemils, endtime, distance, duration;
  byte echo;
  boolean timeout;

  for (i=0; i<8; i++) {
    bit0 = i & 0x01;
    bit1 = (i >> 1) & 0x01;
    bit2 = (i >> 2) & 0x01;

    // addressing transducer-i
    digitalWrite(pinSONAR_ADDRESS0, (bit0==1) ? HIGH : LOW);
    digitalWrite(pinSONAR_ADDRESS1, (bit1==1) ? HIGH : LOW);
    digitalWrite(pinSONAR_ADDRESS2, (bit2==1) ? HIGH : LOW);
    // 100 ms delay for CMOS logic settling
    delay(200);
    // firing transducer
    digitalWrite(pinSONAR_INIT, HIGH);
    // ranging module settling 1 ms time
    delay(1);
   
    timeout = false;
    starttimemils = millis();
    starttime = micros();

    echo = 0;


    while ((echo = digitalRead(pinSONAR_ECHO)!=HIGH)&&(!timeout)); 
     if (millis()-starttimemils>60) timeout = true;

    endtime = micros();
    digitalWrite(pinSONAR_INIT, LOW);

    duration = endtime - starttime;

    if (!timeout)             
      distance = 17*duration/1000;
    else distance = 10000; // invalid distance (100m)

    SonarArray[i] = int(distance);
    delay(10);
  }

}

void LMMobileRobot::useAbility(t_CartAbility ability) {
  switch(ability) {
  case abGO_FORWARD: 
    startLinearMotion(43);
    break;
  case abGO_BACKWARD: 
    startLinearMotion(-49);
    break;
  case abGO_FORWARD10:
    startLinearMotion(15);
    break;
  case abGO_BACKWARD10:
    startLinearMotion(-15);
    break;

  case abTURN_LEFT45:  
    turnCartLeft45();
    break;
  case abTURN_LEFT90: 
    turnCartLeft90();
    break;
  case abTURN_RIGHT45: 
    turnCartRight45();
    break;
  case abTURN_RIGHT90: 
    turnCartRight90();
    break;
  case abTURN_LEFT10:
    turnCartLeft10();
    break;
  case abTURN_RIGHT10:
    turnCartRight10();
    break;
  default: flagAbilityExecuting = false;
           flagAbilityDone = true; 
      break;
  }
}  

void LMMobileRobot::handleMessages() {
  int i;
  //1. Handling Ability Execution (skipping if the ability has not been finished)
  if (!flagAbilityExecuting) {
    if (flagAbilityDone) {
      
      EZPacket* donepack = createCommandDonePacket(CommandSenderID);
      if (!pNetDevice->pushOutboundPacket(donepack)) 
        EZRoboNetDevice::disposePacket(donepack);
      else 
        flagAbilityDone = false;
      
      
    }
    //Serial.println("Still looping");


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
      byte packsender = pack->SenderID;
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
      if(msg->Cmd == rc_FireSonarArray) { // firing the sonar array
             fireSonarArray();
            // sending verification that sonar array has been fired
            EZPacket* sdonepack = createSonarFiringDonePacket(packsender);
            if (!pNetDevice->pushOutboundPacket(sdonepack))
              EZRoboNetDevice::disposePacket(sdonepack); 
      } else if (msg->Cmd == rc_SendCMUCAMAbstractionData) { 
          // designating image data receiver as the packet sender
          CMUCAMDataReceiver = packsender;
          Serial.println("Got an Abstraction request!");
          pCamera->getAbstractionData();

      } else if ( msg->Cmd == rc_SendSensors) {  // pack and send sensor data over the network
            EZPacket* sensorpack = createSensorDataPacket(packsender);

            // pushing the packet into the outbound queue
            if (!pNetDevice->pushOutboundPacket(sensorpack))
              EZRoboNetDevice::disposePacket(sensorpack);  
      } else if (msg->Cmd == rc_ManualCommand) { // forcing us to trigger an ability
            // raising execution flag
            flagAbilityExecuting = true;
            // storing the id of the command sender
            CommandSenderID = packsender;
            useAbility((t_CartAbility)msg->CmdParams[0]);
      }
          
         
        // disposing the Message
        disposeCommandMsg(msg);
        // disposing the packet
        EZRoboNetDevice::disposePacket(pack);
      }
    }
}


// create a Command Message containing sensory data
CommandMsg* LMMobileRobot::createSensorDataMessage() {
  int i;
  CommandMsg* msg = (CommandMsg*)malloc(sizeof(CommandMsg));
  msg->robot = t_HandmadeCart;
  msg->Cmd = rc_SensorData;
  msg->ParamsLength = 16;
  // allocating the parameter section. Should be 16 bytes
  msg->CmdParams = (byte*)malloc(16); //8 2-byte sensor readings 


  // assigning sensor readings
  for (i=0; i<8; i++) {
    msg->CmdParams[2*i] = lowByte(SonarArray[i]);
    msg->CmdParams[2*i+1] = highByte(SonarArray[i]);
  }


  return msg;
}


CommandMsg* LMMobileRobot::createCommandDoneMessage() {
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


CommandMsg* LMMobileRobot::createSonarFiringDoneMessage() {
  int i;
  CommandMsg* msg = (CommandMsg*)malloc(sizeof(CommandMsg));
  msg->robot = t_HandmadeCart;
  msg->Cmd = rc_SonarFired;
  msg->ParamsLength = 1;
  // allocating the parameter section. Should be 16 bytes
  msg->CmdParams = (byte*)malloc(1); //1 byte parameters
  msg->CmdParams[0] = 0; // ignored anyway
  
  return msg;

}


// create a packet containing sensory data
EZPacket* LMMobileRobot::createSensorDataPacket(byte receiverid) {
  EZPacket* pack = (EZPacket*)malloc(sizeof(EZPacket));

  pack->SenderID = pNetDevice->NodeID;
  pack->ReceiverID = receiverid;
  pack->SenderNodeType = t_Node_Robot;
  pack->MessageLength = 16+4;
  pack->Message = (byte*)malloc(pack->MessageLength);

  CommandMsg* msg = createSensorDataMessage();  


  pack->Message[0] = (byte)msg->robot;
  pack->Message[1] = (byte)msg->Cmd;
  pack->Message[2] = lowByte(msg->ParamsLength);
  pack->Message[3] = highByte(msg->ParamsLength);



  int i; 
  for (i=0; i<16; i++)
    pack->Message[4+i] = msg->CmdParams[i];
  // disposing the message
  disposeCommandMsg(msg);

  return pack;
}


EZPacket* LMMobileRobot::createCommandDonePacket(byte receiverid) {
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

EZPacket* LMMobileRobot::createSonarFiringDonePacket(byte receiverid) {
  EZPacket* pack = (EZPacket*)malloc(sizeof(EZPacket));

  pack->SenderID = pNetDevice->NodeID;
  pack->ReceiverID = receiverid;
  pack->SenderNodeType = t_Node_Robot;
  pack->MessageLength = 5;
  pack->Message = (byte*)malloc(pack->MessageLength);

  CommandMsg* msg = createSonarFiringDoneMessage();  


  pack->Message[0] = (byte)msg->robot;
  pack->Message[1] = (byte)msg->Cmd;
  pack->Message[2] = lowByte(msg->ParamsLength);
  pack->Message[3] = highByte(msg->ParamsLength);
  pack->Message[4] = msg->CmdParams[0];
  
  disposeCommandMsg(msg);
  
  return pack;
}



void LMMobileRobot::resetCompass() {
  int i;
  angle = 0;
  V_2 = V_1 = V = V1 = V2 = 0;

  TR = (1.0*TRmil)/1000.0;

  ZeroRef = 0;
  for (i=0; i<20; i++) {
    ZeroRef += analogRead(0);
    delay(10);
  }
  ZeroRef /= 20;
  Serial.print("Zero Reading: ");
  Serial.println(ZeroRef, DEC);

  Serial.print("Period : ");
  Serial.println(TR, 4);

  StepCounter = 0;
  PreviousStep = millis();
}

void LMMobileRobot::boolIntegral() {
  CurrentStep = millis();
  if (CurrentStep-PreviousStep >= TRmil) {
    PreviousStep = CurrentStep;
    V_2 = V_1;
    V_1 = V;
    V = V1;
    V1 = V2;

    int reading = analogRead(0);
    float temp = (5000.0/1023.0)*(1.0*reading - 1.0*ZeroRef)/3.3;
    if (abs(temp)>0.0) V2 = temp;
    else V2 = 0.0;
    //Serial.print("Calculated Speed: ");
    //Serial.println(V1, 4);
    if (StepCounter <4) StepCounter++;
    else {
      StepCounter = 0;
      angle += (2*TR/45)*(7*V_2 + 32*V_1 + 12*V + 32*V1+7*V2 );
    }
  }
}

void LMMobileRobot::rotateCart(float dangle) {
  float sign1, sign2;
  if (dangle>0) {
    sign1 = 1;
    sign2 = -1;
  } 
  else {
    sign1 = -1;
    sign2 = 1;
  }
  float abangle = abs(dangle);
  resetCompass();

  // running motors
  int speed1 = (dangle>0) ? 1700 : 1100;
  int speed2 = (dangle>0) ? 1100 : 1700;

  digitalWrite(pinRIGHT_MOTORS, HIGH);
  delayMicroseconds(speed1);
  digitalWrite(pinRIGHT_MOTORS, LOW);
  digitalWrite(pinLEFT_MOTORS, HIGH);
  delayMicroseconds(speed2);
  digitalWrite(pinLEFT_MOTORS, LOW);
  int region=1;
  while ((abs(angle)<abs(dangle))&&(abs(dangle-angle)>1.1)) {
    boolIntegral();
    /* if ((abs(angle)>abs(dangle)/4)&&(region==1)) {
     region=2;
     speed1 = (dangle>0) ? 1700 : 1100;
     speed2 = (dangle>0) ? 1100 : 1700;
     
     digitalWrite(pinRIGHT_MOTORS, HIGH);
     delayMicroseconds(speed1);
     digitalWrite(pinRIGHT_MOTORS, LOW);
     
     digitalWrite(pinLEFT_MOTORS, HIGH);
     delayMicroseconds(speed2);
     digitalWrite(pinLEFT_MOTORS, LOW);
     } */
    // slowing down
   /* if ((abs(angle)>abs(dangle)/2)&&(region==1)) {
      region=2;
      speed1 = (dangle>0) ? 1630 : 1130;
      speed2 = (dangle>0) ? 1130 : 1630;

      digitalWrite(pinRIGHT_MOTORS, HIGH);
      delayMicroseconds(speed1);
      digitalWrite(pinRIGHT_MOTORS, LOW);
      delay(1);
      digitalWrite(pinLEFT_MOTORS, HIGH);
      delayMicroseconds(speed2);
      digitalWrite(pinLEFT_MOTORS, LOW);
    }*/   
    if (abs(dangle)-abs(angle)<1.1) stopMotors();

  }

  stopMotors();
  // raising command done flag
  flagAbilityExecuting = false;
  flagAbilityDone = true;
  Serial.print(" Roation finished. Angle is: ");
  Serial.println(angle, 4);
}


void LMMobileRobot::turnCartLeft45() {
  rotateCart(45);
}

void LMMobileRobot::turnCartRight45() {
  rotateCart(-45);
}

void LMMobileRobot::turnCartLeft90() {
  rotateCart(82);
}

void LMMobileRobot::turnCartRight90() {
  rotateCart(-82);
}

void LMMobileRobot::turnCartLeft10() {
  rotateCart(10);
}

void LMMobileRobot::turnCartRight10() {
  rotateCart(-10);
}

