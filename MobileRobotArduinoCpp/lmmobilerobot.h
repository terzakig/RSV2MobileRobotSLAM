// ***************************************************
// The header file for the mobile robot class
//
//              George Terzakis 2010

#ifndef LMMOBILEROBOT_H
#define LMMOBILEROBOT_H

#include "WProgram.h"
#include "cmucam.h"
#include "ezrobonet.h"
#include "ezrobonetcmds.h"


typedef enum {
  abGO_FORWARD = 0,
  abGO_BACKWARD,
  abGO_FORWARD10,
  abGO_BACKWARD10,
  abTURN_LEFT45,
  abTURN_LEFT90,
  abTURN_RIGHT45,
  abTURN_RIGHT90,
  abTURN_LEFT10,
  abTURN_RIGHT10, 
  ABILITIES_NUM
}t_CartAbility;

// ******************** Encoder Interrupt Service Routines
// encoder 1
void encoder1ISR();

// encoder 3
void encoder3ISR();

// Voltatile variables (used by the interrupts)
extern volatile float Reference1, Reference2;
extern volatile float Feedback1, Feedback2;   // motor encoder feedback (accumulated revolutions)

extern volatile float PlantVoltage1, PlantVoltage2;  // -7.2 to +7.2V
// Encoder use
extern volatile int counter1,  counter3 ;
extern volatile byte cha1state,  cha3state, chb1state, chb3state;
extern volatile byte newcha1state, newcha3state, newchb3state, newchb1state;
         
// ***************** volatiles done



const float WHEEL_RADIUS = 6.5;  // 6.5 cms wheel radius
const float CART_RADIUS = 17.5;    // 22 cms cart radius

const int pinRIGHT_MOTORS = 12;
const int pinLEFT_MOTORS = 13;

const int pinGYRO = 0;      // LISYS Gyro analog line

const int pinSONAR_INIT = 11;
const int pinSONAR_ECHO = 10;

const int pinSONAR_ADDRESS0 = 9;
const int pinSONAR_ADDRESS1 = 8;
const int pinSONAR_ADDRESS2 = 7;

// Right-Front encoder pins
const int pinENCODER1_CHA = 52;
const int pinENCODER1_CHB = 50;
const int pinENCODER1_DIR = 48;
const int pinENCODER1_CLK = 18; // external pin interrupt 5

// Right-Rear Encoder pins
const int pinENCODER2_CHA = 49;
const int pinENCODER2_CHB = 47;
const int pinENCODER2_DIR = 45;
const int pinENCODER2_CLK = 19; // external pin interrupt 4

// Left-Front encoder
const int pinENCODER3_CHA = 37;
const int pinENCODER3_CHB = 35;
const int pinENCODER3_DIR = 33;
const int pinENCODER3_CLK = 20; // external pin interrupt 3

// Left-Rear encoder
const int pinENCODER4_CHA = 32;
const int pinENCODER4_CHB = 30;
const int pinENCODER4_DIR = 28;
const int pinENCODER4_CLK = 21; // external pin interrupt 2

class LMMobileRobot {
  
  // Class members
  public:  HardwareSerial* pRadioPort; // pointer to serial port connected to R/F transceiver
           CMUCAM* pCamera;
           
           EZRoboNetDevice* pNetDevice;
           
           // CMUCAM frame Receiver node ID
           byte CMUCAMDataReceiver;
           
           // ability execution flags
           boolean flagAbilityExecuting, flagAbilityDone;
           // ability sender id
           byte CommandSenderID;
  
  // Last sonar sensor readings
  public: uint16_t SonarArray[8];
                   

  public: float ErrorMargin1, ErrorMargin2;  // depending on reference, bound will encapsulate valid outputs before shutting down the controller


  public: float f1, f2, K; // state feedback gains and input feed-forward gain
  public: float n1, n2; // state observer gains

  public: float T;      // period in seconds
          int Tmil;     // period in milliseconds

  public: float L[2][2];   // observer state matrix

  public: float p1, p2;   // poles of the closed loop system
          float c1, c2;   // poles of the observer

          float m1x1cur, m1x2cur;   // Right Wheels current observed states (x1(k), x2(k))
          float m1x1prev, m1x2prev; // Riught wheels previous observed states (x1(k-1), x2(k-1))

          float m2x1cur, m2x2cur;   // Left wheels current observed states (x1(k), x2(k))
          float m2x1prev, m2x2prev; // Left wheels previous observed states (x1(k-1), x2(k-1))

          unsigned long ControllerPrevStep; // time of previous excution of the state feedback controller

          

// Assumming the motor has a 1st order transfer function, Let Kmotor be a constant
// and Pmotor be the pole of its transfer function such as G(s) = Kmotor/(s+Pmotor)
        float Kmotor, Pmotor; 

        boolean ControllerExecuting; 
        
// Some members regarding rotation control and feedback
        unsigned long PreviousStep, CurrentStep;  // rotatation current/prev step in milliseconds

        float angle, V_2, V_1, V, V1, V2;     // Boolean numerical integration rule states (V[k-2], V[k-1], V[k], V[k+1], V[k+2])
                                              // angle is the calculated integral

        int ZeroRef, StepCounter;            // counts up to 4 consecutive state updates for the Boolean rule
        int TRmil;                              // sampling period used by the Boolean Numerical Integration rule
        float TR;                           // sampling period in seconds (float)



// constructor
public: LMMobileRobot(HardwareSerial* RadioComm, byte node_id, HardwareSerial* CMUCamComm);

// initialization of controllers
// clp1, clp2 are the closed-loop system's poles. PerioMil is the sampling period in ms.
// Kmot is the motor's constant and Pmot is motors pole if G(s)=Kmot/(s+Pmot)
public: void initControllers(float clp1, float clp2, int PeriodMil, float Kmot, float Pmot);

// resetting controllers (dettaching interrupts and zeroing state variables)
public: void resetControllers();

// initiate a rotation by an angle
public: void startRotation(float angle);

// initiate linear motion forward/backwards
public: void startLinearMotion(float offset);

// update the system's states using the encoder feedback
public: void updateObservedStates();

// drive the motors at a given voltage level
public: void driveMotors(float voltage1, float voltage2);

// stop the motors (Hopfully!!!)
public: void stopMotors();

// controller action at a given instant k*T
public: void controllerAction();

// turn the cart 45 degrees left
public: void turnCartLeft45();
// turn the cart 45 degrees right
public: void turnCartRight45();
// turn the cart 90 degrees left
public: void turnCartLeft90();
// turn the cart 90 degrees Right
public: void turnCartRight90();

// turn cart10 degrees (approx) left
public: void turnCartLeft10();

// turn cart right by 10 degrees (approx)
public: void turnCartRight10();

// attach all interrupts
public: void attachInterrupts();

// detach all interrupts
public: void detachInterrupts();

// fire all 8 sonar transducers and store the readings
public: void fireSonarArray();

// use an ability
public: void useAbility(t_CartAbility ability);

public: void handleMessages();

public: CommandMsg* createSensorDataMessage();

public: CommandMsg* createCommandDoneMessage();

// create a packet containing sensory data
public: EZPacket* createSensorDataPacket(byte receiverid);

// create a packet containing a Command Done message
public: EZPacket* createCommandDonePacket(byte receiverid);

// create a packet containing a sonar firing done message
public: EZPacket* createSonarFiringDonePacket(byte receiverid);
// create a message containing a SonarFired dONE MESSAGE
public: CommandMsg* createSonarFiringDoneMessage();
// the following method resets the compass (gyro)
void resetCompass();

// Boolean Numerical Integration step
void boolIntegral();

// cart rotation by "dangle" degrees
void rotateCart(float dangle);

}; // class ends


#endif // end  
