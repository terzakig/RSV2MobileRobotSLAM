////////////// Robosapien V2 Arduino Node ////////////////////////
//
// The Ardunio controls ALL robosapien moves and actions
// and polls all digital and analog sensors) directly or indirectly (through the local dsp)
//
//              The node is connected to a roboentity in a stationary PC via the EZRoboNet protocol.
//				Thus, the roboentity has full control and it simply acts as a drone.
//				Moreover, a CMUCAM2 is installed, and the roboentity may retrieve captures in various resolutions
//
//							George Terzakis 2009

#include "robosapienv2.h"
#include "cppfix.h"



RobosapienV2 *probosapien;

void setup() {
  
  probosapien = new RobosapienV2(&Serial1, 2, &Serial3);
  
  Serial.begin(19200);
}

void loop() {
  
 /* while (1) {
    probosapien->transmitCommand(0x35F);
    delay(90);
  }*/
  // the net device must process internal events and change state
  probosapien->pNetDevice->processEvents();
  
  // the Net Device must handle Data (append the packet queue) 
  probosapien->pNetDevice->processData();


  // call to camera internal state-changing function
  probosapien->pCamera->transitionAction();
  // call to the camera's event handler (scan line or abstracted frame arrived)
  probosapien->pCamera->processEvents();

   
  // robosapien must handle possible packets in the queue of the Net Device
  probosapien->handleMessages();
  
  
  
    
}
