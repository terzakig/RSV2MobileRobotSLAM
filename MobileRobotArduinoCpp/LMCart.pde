////////////// Handmade Mobile Robot Arduino Node ////////////////////////
//
// This program enables to the robot to act as a drone with several local capabilities (aka "abilities"):
//
//	1. A local state feedbackl controller for forward linear motiona and rotation (feedback from optical encoders and gyro)
//
//  2. Fire a ring of 8 ultrasonics transducers and estimate distances to obstacles.
//
//  3. Obtain frame captures from a CMUCAM2.

//
//              The node is connected to a roboentity in a stationary PC via the EZRoboNet protocol.
//				Thus, the roboentity has full control and it simply acts as a drone.
//
//							George Terzakis 2009


#include "lmmobilerobot.h"
#include "cppfix.h"



LMMobileRobot *pcart;

     
void setup() {
  
  pcart = new LMMobileRobot(&Serial2, 3, &Serial3);
  
  
  Serial.begin(19200);
 
  
  
}

void loop() {
  
  pcart->controllerAction();
  
  // the net device must process internal events and change state
  pcart->pNetDevice->processEvents();
  
  pcart->controllerAction();
  
  // the Net Device must handle Data (append the packet queue) 
  pcart->pNetDevice->processData();

  pcart->controllerAction();

  // call to camera internal state-changing function
  pcart->pCamera->transitionAction();
  
  pcart->controllerAction();
  
  // call to the camera's event handler (scan line or abstracted frame arrived)
  pcart->pCamera->processEvents();

  pcart->controllerAction();
   
  // robosapien must handle possible packets in the queue of the Net Device
  pcart->handleMessages();
  
  /*
  
  if (Serial.available()>0) {
    byte ch = Serial.read();
    
    switch(ch) {
      case 's' : cart->stopMotors();
                 break;
      
      case 'f' : cart->startLinearMotion(40);
                 break;
      case 'b' : cart->startLinearMotion(-45);
                 break;

      case 'r' : cart->startRotation(66);
                 //cart->turnCartRight();
                 break;
      case 'l' : cart->startRotation(-110);
                 //cart->turnCartLeft();
                 break;
      case 'u' : digitalWrite(pinSONAR_INIT, HIGH);
                 delay(1); 

                 unsigned long starttime = micros();
                 int echo = 0;
                 
                 while (echo = digitalRead(pinSONAR_ECHO)!=1);
                 unsigned long endtime = micros();
                 digitalWrite(pinSONAR_INIT, LOW);
                 
                 unsigned long duration = endtime - starttime;
                 
                 
                 unsigned long distance = 17*duration/1000;
                 Serial.print("Distance Measured: ");
                 Serial.println(distance, DEC);
                 break;
      
                
                
              
    }
            
     if ((ch>='0') && (ch<='8')) {
                byte addr = ch-'0';
                byte bit0 = addr & 0x01;
                byte bit1 = (addr >> 1) & 0x01;
                byte bit2 = (addr >> 2) & 0x01;
                
                digitalWrite(pinSONAR_ADDRESS0, (bit0==1) ? HIGH : LOW);
                digitalWrite(pinSONAR_ADDRESS1, (bit1==1) ? HIGH : LOW);
                digitalWrite(pinSONAR_ADDRESS2, (bit2==1) ? HIGH : LOW);
                }
                
  }*/ 
}
