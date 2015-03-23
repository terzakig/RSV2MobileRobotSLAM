// *************************** RobosapienV2 class for Arduino Mega ***********************
// *                                                                                     *
// *                      George Terzakis (Tsambikos) 2010                               *
// *                                                                                     *
// *                     A class to control the Robosapien V2                            *
// *                   and gain feedback from most of the sensors                        *
// ***************************************************************************************


#ifndef ROBOSAPIENV2_H
#define ROBOSAPIENV2_H

#include "WProgram.h"
#include "ezrobonet.h"
#include "ezrobonetcmds.h"
#include "cmucam.h"

// *************** Robosapien Command Constants ****************
// walk / turn commands
const uint16_t rsv2WALK_FORWARD = 0x300;          // walk FORWARD
const uint16_t rsv2WALK_BACKWARD = 0x301;         // walk BACKWARD
const uint16_t rsv2TURN_RIGHT = 0x302;            // turn RIGHT
const uint16_t rsv2TURN_LEFT = 0x303;             // turn LEFT
const uint16_t rsv2WALK_FORWARDRIGHT = 0x304;    // walk FORWARD- RIGHT
const uint16_t rsv2WALK_FORWARDLEFT = 0x305;     // walk FORWARD-LEFT
const uint16_t rsv2WALK_BACKWARDRIGHT = 0x306;   // walk BACKWARD-RIGHT
const uint16_t rsv2WALK_BACKWARDLEFT = 0x307;    // walk BACKWARD-LEFT

// Buldozer walk
const uint16_t rsv2BULLDOZER_FORWARD = 0x36A;      // Bulldozer FORWARD
const uint16_t rsv2BULLDOZER_BACKWARD = 0x372;     // Bulldozer BACKWARD
// walk/turn commands ends

// Upper Body+Head motion
const uint16_t rsv2LEAN_BACKWARD = 0x320;       //  LEAN FORWARD
const uint16_t rsv2LEAN_FORWARD = 0x321;      //  LEAN BACKWARD
const uint16_t rsv2LEAN_RIGHT = 0x322;         //  LEAN RIGHT
const uint16_t rsv2LEAN_LEFT = 0x323;          //  LEAN LEFT
const uint16_t rsv2LEAN_BACKWARDRIGHT = 0x324;  // LEAN FORWARD-RIGHT
const uint16_t rsv2LEAN_BACKWARDLEFT = 0x325;   // LEAN FORWARD-LEFT
const uint16_t rsv2LEAN_FORWARDLEFT = 0x326; // LEAN BACKWARD-RIGHT
const uint16_t rsv2LEAN_FORWARDRIGHT = 0x327;  // LEAN BACKWARD-LEFT
// Upper Body+Head motion commands ends

// Secondary Upper Body+Head motion (same as primary)
const uint16_t rsv2SEC_LEAN_BACKWARD = 0x310;      // LEAN FORWARD
const uint16_t rsv2SEC_LEAN_FORWARD = 0x311;     // LEAN BACKWARD
const uint16_t rsv2SEC_LEAN_RIGHT = 0x312;         //  LEAN RIGHT
const uint16_t rsv2SEC_LEAN_LEFT = 0x313;          //  LEAN LEFT
const uint16_t rsv2SEC_LEAN_BACKWARDRIGHT = 0x314;  // LEAN FORWARD-RIGHT
const uint16_t rsv2SEC_LEAN_BACKWARDLEFT = 0x315;   // LEAN FORWARD-LEFT
const uint16_t rsv2SEC_LEAN_FORWARDRIGHT = 0x316; // LEAN BACKWARD-RIGHT
const uint16_t rsv2SEC_LEAN_FORWARDLEFT = 0x317;  // LEAN BACKWARD-LEFT
// Secondary Upper Body+Head motion commands ends



const uint16_t rsv2FREE_ROAM = 0x382;     // 
const uint16_t rsv2STOP = 0x3AA;          // STOP

const uint16_t rsv2MIC_SENSORS_ONOFF = 0x383;   // Turns sound sensors ON/OFF
const uint16_t rsv2VISION_ONOFF = 0x380;        // Vision ON/OFF (possibly includes IR)

const uint16_t rsv2POS_PROG_ENTRY = 0x381;      // Positional Program Entry
const uint16_t rsv2GAIT_CHANGE = 0x390;         // Gait Change/Quick Reset

const uint16_t rsv2RESET = 0x353;             // RESET
const uint16_t rsv2SLEEPWAKE = 0x386;         // SLEEP / WAKE UP

// right arm motion commands
const uint16_t rsv2RIGHT_ARM_THROW = 0x355;        // Right Arm THROW
const uint16_t rsv2RIGHT_ARM_LOW_PICKUP = 0x356;   // Right arm LOW PICKUP
const uint16_t rsv2RIGHT_ARM_HIGH_PICKUP = 0x357;  // Right Arm HIGH PICKUP
const uint16_t rsv2RIGHT_ARM_GRAB = 0x358;         // Right Arm GRAB (from the floor)
const uint16_t rsv2RIGHT_ARM_GIVE = 0x359;         // Right Arm GIVE
const uint16_t rsv2RIGHT_ARM_ROLL = 0x35A;         // Right Arm ROLL
// directional right arm motion
const uint16_t rsv2RIGHT_ARM_UP = 0x328;           // Right Arm UP
const uint16_t rsv2RIGHT_ARM_DOWN = 0x329;         // Right Arm DOWN
const uint16_t rsv2RIGHT_ARM_RIGHT = 0x32A;        // Right Arm RIGHT
const uint16_t rsv2RIGHT_ARM_LEFT = 0x32B;         // Right Arm LEFT
const uint16_t rsv2RIGHT_ARM_UPRIGHT = 0x32C;      // Right Arm UP AND RIGHT
const uint16_t rsv2RIGHT_ARM_UPLEFT = 0x32D;       // Right Arm UP and LEFT
const uint16_t rsv2RIGHT_ARM_DOWNRIGHT = 0x32E;    // Right Arm DOWN and RIGHT
const uint16_t rsv2RIGHT_ARM_DOWNLEFT = 0x32F;     // Right Arm DOWN and LEFT
// right arm motion commands ends

// left arm motion commands
const uint16_t rsv2LEFT_ARM_THROW = 0x35C;         // Left Arm THROW
const uint16_t rsv2LEFT_ARM_LOW_PICKUP = 0x35D;    // Left Arm LOW PICKUP
const uint16_t rsv2LEFT_ARM_HIGH_PICKUP = 0x35E;   // Left Arm HIGH PICKUP
const uint16_t rsv2LEFT_ARM_GRAB = 0x35F;          // Left Arm GRAB
const uint16_t rsv2LEFT_ARM_GIVE = 0x360;          // Left Arm GIVE
const uint16_t rsv2LEFT_ARM_ROLL = 0x361;          // Left Arm ROLL
// directional left arm motion
const uint16_t rsv2LEFT_ARM_UP = 0x330;            // Left Arm UP
const uint16_t rsv2LEFT_ARM_DOWN = 0x331;          // Left Arm DOWN
const uint16_t rsv2LEFT_ARM_RIGHT = 0x332;         // Left Arm RIGHT
const uint16_t rsv2LEFT_ARM_LEFT = 0x333;          // Left Arm LEFT
const uint16_t rsv2LEFT_ARM_UPRIGHT = 0x334;       // Left Arm UP and RIGHT
const uint16_t rsv2LEFT_ARM_UPLEFT = 0x335;        // Left Arm UP and LEFT
const uint16_t rsv2LEFT_ARM_DOWNRIGHT = 0x336;     // Left Arm DOWN and RIGHT
const uint16_t rsv2LEFT_ARM_DOWNLEFT = 0x337;      // Left Arm DOWN and LEFT


const uint16_t rsv2DANCE_DEMO = 0x350;       // Dance DEMO
const uint16_t rsv2MOVEMENT_DEMO = 0x354;    // Movement DEMO
const uint16_t rsv2LIE_DOWN = 0x351;         // LIE DOWN->SIT UP->LIE DOWN->STAND UP
const uint16_t rsv2GET_UP = 0x364;           // GET UP

// special moves 
const uint16_t rsv2RIGHT_KICK = 0x362;       // RIGHT KICK
const uint16_t rsv2RIGHT_PUSH = 0x365;       // RIGHT PUSH
const uint16_t rsv2RIGHT_CHOP = 0x366;       // RIGHT CHOP
const uint16_t rsv2LEFT_CHOP = 0x367;        // LEFT CHOP
const uint16_t rsv2LEFT_PUSH = 0x368;        // LEFT PUSH
const uint16_t rsv2LEFT_KICK = 0x363;        // LEFT KICK
const uint16_t rsv2OOPS = 0x35B;             // OOPS (whatever that means)

// Hip and Waist Tilt Motion Commands
const uint16_t rsv2WAIST_FORWARD = 0x340;        // Waist-Hip tilt FORWARD
const uint16_t rsv2WAIST_BACKWARD = 0x341;       // Waist-Hip tilt BCKWARDS
const uint16_t rsv2WAIST_RIGHT = 0x342;          // Waist-Hip tilt RIGHT
const uint16_t rsv2WAIST_LEFT = 0x343;           // Waist-Hip tilt LEFT
const uint16_t rsv2WAIST_FORWARDRIGHT = 0x344;   // Waist-Hip tilt FORWARD and RIGHT
const uint16_t rsv2WAIST_FORWARDLEFT = 0x345;   // Waist-Hip tilt FORWARD and LEFT
const uint16_t rsv2WAIST_BACKWARDRIGHT = 0x346; // Waist-Hip tilt BACKWARD and RIGHT
const uint16_t rsv2WAIST_BACKWARDLEFT = 0x347;  // Waist-Hip tilt BEACKWARDS and LEFT
// Hip and Waist tilt motion ends

// Misc motion and Programming
const uint16_t rsv2CLEAR_PROGRAM = 0x3F4;     // CLEAR PROGRAM
const uint16_t rsv2SOUND_PROGRAM = 0x3F1;     // SOUND PROGRAM
const uint16_t rsv2VISION_PROGRAM = 0x3F0;    // VISION PROGRAM
const uint16_t rsv2MAIN_PROGRAM = 0x3F5;      // MAIN PROGRAM
const uint16_t rsv2RUN_PROGRAM = 0x3F6;       // RUN PROGRAM
const uint16_t rsv2GUARD_MODE = 0x3F2;        // GUARD MODE
const uint16_t rsv2CLEAR_ENTRY = 0x3F3;       // CLEAR ENTRY
const uint16_t rsv2HIGH_FIVE = 0x369;         // HIGH FIVE

// Both Arms Motion Commands
const uint16_t rsv2BOTHARMS_UP = 0x338;         // Both Arms UP
const uint16_t rsv2BOTHARMS_DOWN = 0x339;       // Both Arms DOWN
const uint16_t rsv2BOTHARMS_RIGHT = 0x33A;      // Both Arms RIGHT
const uint16_t rsv2BOTHARMS_LEFT = 0x33B;       // Both Arms LEFT
const uint16_t rsv2BOTHARMS_UPRIGHT = 0x33C;    // Both Arms UP and RIGHT
const uint16_t rsv2BOTHARMS_UPLEFT = 0x33D;     // Both Arms UP and LEFT
const uint16_t rsv2BOTHARMS_DOWNRIGHT = 0x33E;  // Both Arms DOWN and RIGHT
const uint16_t rsv2BOTHARMS_DOWNLEFT = 0x33F;   // Both Arms DOWN and LEFT
// Both Arms motion comands ends


// Behavioral expressions etc. and other moves
const uint16_t rsv2LAUGH = 0x36C;            // LAUGH
const uint16_t rsv2INSULT = 0x36D;           // INSULT
const uint16_t rsv2RIGHT_ARM_DROP = 0x36E;   // Right Arm DROP
const uint16_t rsv2LEFT_ARM_DROP = 0x36F;    // Left Arm DROP
const uint16_t rsv2PLAN = 0x370;             // PLAN (very curious about this...)
const uint16_t rsv2SPARE_CHANGE = 0x371;     // SPARE CHANGE (again very curious...)
const uint16_t rsv2HEY_BABY = 0x36B;         // HEY BABY (!)
// more expressions...
const uint16_t rsv2ROAR = 0x374;             // ROARRRRRR
const uint16_t rsv2DIODE = 0x375;            // DIODE (? - should try that out...)
const uint16_t rsv2FETCH = 0x376;            // FETCH (? again curious...)
const uint16_t rsv2DANGER = 0x377;           // DANGER (? - curious...)
const uint16_t rsv2CALM_DOWN = 0x378;        // CALM DOWN
const uint16_t rsv2HUG = 0x379;              // HUG!
const uint16_t rsv2BURP = 0x373;             // BURP 
// behavior and misc move commands ends 

// Head only Motion Commands
const uint16_t rsv2HEAD_UP = 0x348;         // HEAD UP
const uint16_t rsv2HEAD_DOWN = 0x349;       // HEAD DOWN
const uint16_t rsv2HEAD_RIGHT = 0x34A;      // HEAD RIGHT
const uint16_t rsv2HEAD_LEFT = 0x34B;       // HEAD LEFT
const uint16_t rsv2HEAD_UPRIGHT = 0x34C;    // HEAD UP-RIGHT 
const uint16_t rsv2HEAD_UPLEFT = 0x34D;     // HEAD UP-LEFT
const uint16_t rsv2HEAD_DOWNRIGHT = 0x34E;  // HEAD DOWN-RIGHT
const uint16_t rsv2HEAD_DOWNLEFT = 0x34F;   // HEAD DOWN-LEFT
// Head Motion Comands ends

// several special functions
const uint16_t rsv2POWER_DOWN = 0x384;           // POWER DOWN
const uint16_t rsv2RSV2_INTERACTION = 0x385;     // Robosapien V2 INTERACTION
const uint16_t rsv2RRAPTOR_INTERACTION = 0x387;  // Roboraptor INTERACTION
const uint16_t rsv2RPET_INTERACTION = 0x388;     // Robopet INTERACTIION
const uint16_t rsv2MAN_COL_MODE_YELLOW = 0x38A;  // Manual Color Mode: YELLOW (I wish i knew...)
const uint16_t rsv2MAN_COL_MODE_WHITE = 0x38B;   // Manual Color Mode: WHITE (go figure...)
const uint16_t rsv2DONT_PRESS = 0x37A;           // DON'T PRESS (i will press...)
// special functions ends

// Upper Body ONLY motion commands
const uint16_t rsv2BODY_BACKWARD = 0x318;    // Body lean BACKWARD
const uint16_t rsv2BODY_FORWARD = 0x319;     // Body lean FORWARD 
const uint16_t rsv2BODY_RIGHT = 0x31A;        // Body lean RIGHT
const uint16_t rsv2BODY_LEFT = 0x31B;         // Body lean LEFT
const uint16_t rsv2BODY_BACKWARDRIGHT = 0x31C;  // Body lean BACKWARD-RIGHT
const uint16_t rsv2BODY_BACKWARDLEFT = 0x31D;  // Body lean BACKWARD-LEFT
const uint16_t rsv2BODY_FORWARDRIGHT = 0x31E;  // Body lean FORWARD-RIGHT
const uint16_t rsv2BODY_FORWARDLEFT = 0x31F;   // Body lean FORWARD-LEFT


// *************************** Robosapien V2 Commands Ends Here *************************************

// ************************** Enumeration of the rsv2ilities of the Robot *****************************

typedef enum {
        abWALK_FORWARD = 0,
        abWALK_BACKWARD,         // walk BACKWARD
        abTURN_RIGHT,            // turn RIGHT
        abTURN_LEFT,             // turn LEFT
        abWALK_FORWARDRIGHT,    // walk FORWARD- RIGHT
        abWALK_FORWARDLEFT,     // walk FORWARD-LEFT
        abWALK_BACKWARDRIGHT,   // walk BACKWARD-RIGHT
        abWALK_BACKWARDLEFT,    // walk BACKWARD-LEFT

        // Buldozer walk
        abBULLDOZER_FORWARD,      // Bulldozer FORWARD
        abBULLDOZER_BACKWARD,     // Bulldozer BACKWARD
        // walk/turn abilities ends

        // Upper Body+Head motion
        abLEAN_BACKWARD,       //  LEAN FORWARD
        abLEAN_FORWARD,      //  LEAN BACKWARD
        abLEAN_RIGHT,         //  LEAN RIGHT
        abLEAN_LEFT,          //  LEAN LEFT
        abLEAN_BACKWARDRIGHT,  // LEAN FORWARD-RIGHT
        abLEAN_BACKWARDLEFT,   // LEAN FORWARD-LEFT
        abLEAN_FORWARDRIGHT, // LEAN BACKWARD-RIGHT
        abLEAN_FORWARDLEFT,  // LEAN BACKWARD-LEFT
        // Upper Body+Head motion commands ends


        abFREE_ROAM,     // 
        abSTOP,          // STOP

        // right arm motion commands
        abRIGHT_ARM_THROW,        // Right Arm THROW
        abRIGHT_ARM_LOW_PICKUP,   // Right arm LOW PICKUP
        abRIGHT_ARM_HIGH_PICKUP,  // Right Arm HIGH PICKUP
        abRIGHT_ARM_GRAB,         // Right Arm GRAB (from the floor)
        abRIGHT_ARM_GIVE,         // Right Arm GIVE
        abRIGHT_ARM_ROLL,         // Right Arm ROLL
        // directional right arm motion
        abRIGHT_ARM_UP,           // Right Arm UP
        abRIGHT_ARM_DOWN,         // Right Arm DOWN
        abRIGHT_ARM_RIGHT,        // Right Arm RIGHT
        abRIGHT_ARM_LEFT,         // Right Arm LEFT
        abRIGHT_ARM_UPRIGHT,      // Right Arm UP AND RIGHT
        abRIGHT_ARM_UPLEFT,       // Right Arm UP and LEFT
        abRIGHT_ARM_DOWNRIGHT,    // Right Arm DOWN and RIGHT
        abRIGHT_ARM_DOWNLEFT,     // Right Arm DOWN and LEFT
        // right arm motion commands ends

        // left arm motion commands
        abLEFT_ARM_THROW,         // Left Arm THROW
        abLEFT_ARM_LOW_PICKUP,    // Left Arm LOW PICKUP
        abLEFT_ARM_HIGH_PICKUP,   // Left Arm HIGH PICKUP
        abLEFT_ARM_GRAB,          // Left Arm GRAB
        abLEFT_ARM_GIVE,          // Left Arm GIVE
        abLEFT_ARM_ROLL,          // Left Arm ROLL
        // directional left arm motion
        abLEFT_ARM_UP,            // Left Arm UP
        abLEFT_ARM_DOWN,          // Left Arm DOWN
        abLEFT_ARM_RIGHT,         // Left Arm RIGHT
        abLEFT_ARM_LEFT,          // Left Arm LEFT
        abLEFT_ARM_UPRIGHT,       // Left Arm UP and RIGHT
        abLEFT_ARM_UPLEFT,        // Left Arm UP and LEFT
        abLEFT_ARM_DOWNRIGHT,     // Left Arm DOWN and RIGHT
        abLEFT_ARM_DOWNLEFT,      // Left Arm DOWN and LEFT
  
        abLIE_DOWN,         // LIE DOWN->SIT UP->LIE DOWN->STAND UP
        abGET_UP,           // GET UP

        // special moves 
        abRIGHT_KICK,       // RIGHT KICK 
        abRIGHT_PUSH,       // RIGHT PUSH
        abRIGHT_CHOP,       // RIGHT CHOP
        abLEFT_CHOP,        // LEFT CHOP
        abLEFT_PUSH,        // LEFT PUSH
        abLEFT_KICK,        // LEFT KICK
        abOOPS,             // OOPS (whatever that means)

        // Hip and Waist Tilt Motion Commands
        abWAIST_FORWARD,        // Waist-Hip tilt FORWARD
        abWAIST_BACKWARD,       // Waist-Hip tilt BCKWARDS
        abWAIST_RIGHT,          // Waist-Hip tilt RIGHT
        abWAIST_LEFT,           // Waist-Hip tilt LEFT
        abWAIST_FORWARDRIGHT,   // Waist-Hip tilt FORWARD and RIGHT
        abWAIST_FORWARDLEFT,   // Waist-Hip tilt FORWARD and LEFT
        abWAIST_BACKWARDRIGHT, // Waist-Hip tilt BACKWARD and RIGHT
        abWAIST_BACKWARDLEFT,  // Waist-Hip tilt BEACKWARDS and LEFT
        // Hip and Waist tilt motion ends

        // Misc motion
        abHIGH_FIVE,         // HIGH FIVE

        // Both Arms Motion Commands
        abBOTHARMS_UP,         // Both Arms UP
        abBOTHARMS_DOWN,       // Both Arms DOWN
        abBOTHARMS_RIGHT,      // Both Arms RIGHT
        abBOTHARMS_LEFT,       // Both Arms LEFT
        abBOTHARMS_UPRIGHT,    // Both Arms UP and RIGHT
        abBOTHARMS_UPLEFT,     // Both Arms UP and LEFT
        abBOTHARMS_DOWNRIGHT,  // Both Arms DOWN and RIGHT
        abBOTHARMS_DOWNLEFT,   // Both Arms DOWN and LEFT
        // Both Arms motion comands ends

        // Behavioral expressions etc. and other moves
        abLAUGH,            // LAUGH
        abINSULT,           // INSULT
        abRIGHT_ARM_DROP,   // Right Arm DROP
        abLEFT_ARM_DROP,    // Left Arm DROP
        abPLAN,             // PLAN (very curious about this...)
        abSPARE_CHANGE,     // SPARE CHANGE (again very curious...)
        abHEY_BABY,         // HEY BABY (!)
        // more expressions...
        abROAR,             // ROARRRRRR
        abDIODE,            // DIODE (? - should try that out...)
        abFETCH,            // FETCH (? again curious...)
        abDANGER,           // DANGER (? - curious...)
        abCALM_DOWN,        // CALM DOWN
        abHUG,              // HUG!
        abBURP,             // BURP 
        // behavior and misc move commands ends 

        // Head only Motion Commands
        abHEAD_UP,         // HEAD UP
        abHEAD_DOWN,       // HEAD DOWN
        abHEAD_RIGHT,      // HEAD RIGHT
        abHEAD_LEFT,       // HEAD LEFT
        abHEAD_UPRIGHT,    // HEAD UP-RIGHT 
        abHEAD_UPLEFT,     // HEAD UP-LEFT
        abHEAD_DOWNRIGHT,  // HEAD DOWN-RIGHT
        abHEAD_DOWNLEFT,   // HEAD DOWN-LEFT
        // Head Motion Comands ends

        // Upper Body ONLY motion commands
        abBODY_BACKWARD,    // Body lean BACKWARD
        abBODY_FORWARD,     // Body lean FORWARD 
        abBODY_RIGHT,        // Body lean RIGHT
        abBODY_LEFT,         // Body lean LEFT
        abBODY_BACKWARDRIGHT,  // Body lean BACKWARD-RIGHT
        abBODY_BACKWARDLEFT,  // Body lean BACKWARD-LEFT
        abBODY_FORWARDRIGHT,  // Body lean FORWARD-RIGHT
        abBODY_FORWARDLEFT,   // Body lean FORWARD-LEFT
        abDANCE_DEMO,         // Dance Demo (pretty cool)
        abMIC_SENSORS_ONOFF,           // Microphones off
        abRESET,              // RESET
        ABILITIES_NUM
    } t_RSV2Ability;

// ********************************** Enumeration of rsv2 abilities ends *****************************************

// ********************************** Mapping Abilities to an execution time ************************************
const uint8_t AbilityTime[ABILITIES_NUM] = {
        14,         // walk FORWARD
        6,         // walk BACKWARD
        12,            // turn RIGHT
        12,             // turn LEFT
        6,    // walk FORWARD- RIGHT
        6,     // walk FORWARD-LEFT
        6,   // walk BACKWARD-RIGHT
        6,    // walk BACKWARD-LEFT

        // Buldozer walk
        10,      // Bulldozer FORWARD
        10,     // Bulldozer BACKWARD
        // walk/turn rsv2ilities ends

        // Upper Body+Head motion
        1,       //  LEAN FORWARD
        1,      //  LEAN BACKWARD
        1,         //  LEAN RIGHT
        1,          //  LEAN LEFT
        1,  // LEAN FORWARD-RIGHT
        1,   // LEAN FORWARD-LEFT
        1, // LEAN BACKWARD-RIGHT
        1,  // LEAN BACKWARD-LEFT
        // Upper Body+Head motion commands ends


        10,     // FREE ROAM 
        1,          // STOP

        // right arm motion commands
        6,        // Right Arm THROW
        20,   // Right arm LOW PICKUP
        17,  // Right Arm HIGH PICKUP
        17,         // Right Arm GRAB (from the floor)
        6,         // Right Arm GIVE
        5,         // Right Arm ROLL
        // directional right arm motion
        1,           // Right Arm UP
        1,         // Right Arm DOWN
        1,        // Right Arm RIGHT
        1,         // Right Arm LEFT
        1,      // Right Arm UP AND RIGHT
        1,       // Right Arm UP and LEFT
        1,    // Right Arm DOWN and RIGHT
        1,     // Right Arm DOWN and LEFT
        // right arm motion commands ends

        // left arm motion commands
        6,         // Left Arm THROW
        20,    // Left Arm LOW PICKUP
        18,   // Left Arm HIGH PICKUP
        6,          // Left Arm GRAB
        6,          // Left Arm GIVE
        5,          // Left Arm ROLL
        // directional left arm motion
        1,            // Left Arm UP
        1,          // Left Arm DOWN
        1,         // Left Arm RIGHT
        1,          // Left Arm LEFT
        1,       // Left Arm UP and RIGHT
        1,        // Left Arm UP and LEFT
        1,     // Left Arm DOWN and RIGHT
        1,      // Left Arm DOWN and LEFT
  
        15,         // LIE DOWN->SIT UP->LIE DOWN->STAND UP
        15,           // GET UP

        // special moves 
        6,       // RIGHT KICK 
        6,       // RIGHT PUSH
        6,       // RIGHT CHOP
        6,        // LEFT CHOP
        6,        // LEFT PUSH
        6,        // LEFT KICK
        7,             // OOPS (whatever that means)

        // Hip and Waist Tilt Motion Commands
        1,        // Waist-Hip tilt FORWARD
        1,       // Waist-Hip tilt BCKWARDS
        1,          // Waist-Hip tilt RIGHT
        1,           // Waist-Hip tilt LEFT
        3,   // Waist-Hip tilt FORWARD and RIGHT
        3,   // Waist-Hip tilt FORWARD and LEFT
        3, // Waist-Hip tilt BACKWARD and RIGHT
        3,  // Waist-Hip tilt BEACKWARDS and LEFT
        // Hip and Waist tilt motion ends

        // Misc motion
        6,         // HIGH FIVE

        // Both Arms Motion Commands
        1,         // Both Arms UP
        1,       // Both Arms DOWN
        1,      // Both Arms RIGHT
        1,       // Both Arms LEFT
        1,    // Both Arms UP and RIGHT
        1,     // Both Arms UP and LEFT
        1,  // Both Arms DOWN and RIGHT
        1,   // Both Arms DOWN and LEFT
        // Both Arms motion comands ends

        // Behavioral expressions etc. and other moves
        6,            // LAUGH
        6,           // INSULT
        6,   // Right Arm DROP
        6,    // Left Arm DROP
        6,             // PLAN (very curious rsv2out this...)
        6,     // SPARE CHANGE (again very curious...)
        6,         // HEY BABY (!)
        // more expressions...
        6,             // ROARRRRRR
        6,            // DIODE (? - should try that out...)
        6,            // FETCH (? again curious...)
        6,           // DANGER (? - curious...)
        6,        // CALM DOWN
        6,              // HUG!
        7,             // BURP 
        // behavior and misc move commands ends 

        // Head only Motion Commands
        1,         // HEAD UP
        1,       // HEAD DOWN
        1,      // HEAD RIGHT
        1,       // HEAD LEFT
        1,    // HEAD UP-RIGHT 
        1,     // HEAD UP-LEFT
        1,  // HEAD DOWN-RIGHT
        1,   // HEAD DOWN-LEFT
        // Head Motion Comands ends

        // Upper Body ONLY motion commands
        1,    // Body lean BACKWARD
        1,     // Body lean FORWARD 
        1,        // Body lean RIGHT
        1,         // Body lean LEFT
        1,  // Body lean BACKWARD-RIGHT
        1,  // Body lean BACKWARD-LEFT
        1,  // Body lean FORWARD-RIGHT
        1,   // Body lean FORWARD-LEFT
        30,          // Dance demo
        1,           // MICS ON/OFF
        1            // RESET
};             
  

// ********************************** Defining Abilities that require stopping ************************************
const boolean AbilityStop[ABILITIES_NUM] = {
        true,
        true,         // walk BACKWARD
        true,            // turn RIGHT
        true,             // turn LEFT
        true,    // walk FORWARD- RIGHT
        true,     // walk FORWARD-LEFT
        true,   // walk BACKWARD-RIGHT
        true,    // walk BACKWARD-LEFT

        // Buldozer walk
        false,      // Bulldozer FORWARD
        false,     // Bulldozer BACKWARD
        // walk/turn rsv2ilities ends

        // Upper Body+Head motion
        false,       //  LEAN FORWARD
        false,      //  LEAN BACKWARD
        false,         //  LEAN RIGHT
        false,          //  LEAN LEFT
        false,  // LEAN FORWARD-RIGHT
        false,   // LEAN FORWARD-LEFT
        false, // LEAN BACKWARD-RIGHT
        false,  // LEAN BACKWARD-LEFT
        // Upper Body+Head motion commands ends


        true,     // FREE ROAM
        false,          // STOP

        // right arm motion commands
        false,        // Right Arm THROW
        true,   // Right arm LOW PICKUP
        true,  // Right Arm HIGH PICKUP
        true,         // Right Arm GRAB (from the floor)
        false,         // Right Arm GIVE
        false,         // Right Arm ROLL
        // directional right arm motion
        false,           // Right Arm UP
        false,         // Right Arm DOWN
        false,        // Right Arm RIGHT
        false,         // Right Arm LEFT
        false,      // Right Arm UP AND RIGHT
        false,       // Right Arm UP and LEFT
        false,    // Right Arm DOWN and RIGHT
        false,     // Right Arm DOWN and LEFT
        // right arm motion commands ends

        // left arm motion commands
        false,         // Left Arm THROW
        true,    // Left Arm LOW PICKUP
        true,   // Left Arm HIGH PICKUP
        true,          // Left Arm GRAB
        false,          // Left Arm GIVE
        false,          // Left Arm ROLL
        // directional left arm motion
        false,            // Left Arm UP
        false,          // Left Arm DOWN
        false,         // Left Arm RIGHT
        false,          // Left Arm LEFT
        false,       // Left Arm UP and RIGHT
        false,        // Left Arm UP and LEFT
        false,     // Left Arm DOWN and RIGHT
        false,      // Left Arm DOWN and LEFT
  
        false,         // LIE DOWN->SIT UP->LIE DOWN->STAND UP
        false,           // GET UP

        // special moves 
        false,       // RIGHT KICK 
        false,       // RIGHT PUSH
        false,       // RIGHT CHOP
        false,        // LEFT CHOP
        false,        // LEFT PUSH
        false,        // LEFT KICK
        false,             // OOPS (whatever that means)

        // Hip and Waist Tilt Motion Commands
        true,        // Waist-Hip tilt FORWARD
        true,       // Waist-Hip tilt BCKWARDS
        true,          // Waist-Hip tilt RIGHT
        true,           // Waist-Hip tilt LEFT
        true,   // Waist-Hip tilt FORWARD and RIGHT
        true,   // Waist-Hip tilt FORWARD and LEFT
        true, // Waist-Hip tilt BACKWARD and RIGHT
        true,  // Waist-Hip tilt BEACKWARDS and LEFT
        // Hip and Waist tilt motion ends

        // Misc motion
        false,         // HIGH FIVE

        // Both Arms Motion Commands
        false,         // Both Arms UP
        false,       // Both Arms DOWN
        false,      // Both Arms RIGHT
        false,       // Both Arms LEFT
        false,    // Both Arms UP and RIGHT
        false,     // Both Arms UP and LEFT
        false,  // Both Arms DOWN and RIGHT
        false,   // Both Arms DOWN and LEFT
        // Both Arms motion comands ends

        // Behavioral expressions etc. and other moves
        false,            // LAUGH
        false,           // INSULT
        false,   // Right Arm DROP
        false,    // Left Arm DROP
        false,             // PLAN (very curious rsv2out this...)
        false,     // SPARE CHANGE (again very curious...)
        false,         // HEY BABY (!)
        // more expressions...
        false,             // ROARRRRRR
        false,            // DIODE (? - should try that out...)
        false,            // FETCH (? again curious...)
        false,           // DANGER (? - curious...)
        false,        // CALM DOWN
        false,              // HUG!
        false,             // BURP 
        // behavior and misc move commands ends 

        // Head only Motion Commands
        false,         // HEAD UP
        false,       // HEAD DOWN
        false,      // HEAD RIGHT
        false,       // HEAD LEFT
        false,    // HEAD UP-RIGHT 
        false,     // HEAD UP-LEFT
        false,  // HEAD DOWN-RIGHT
        false,   // HEAD DOWN-LEFT
        // Head Motion Comands ends

        // Upper Body ONLY motion commands
        false,    // Body lean BACKWARD
        false,     // Body lean FORWARD 
        false,        // Body lean RIGHT
        false,         // Body lean LEFT
        false,  // Body lean BACKWARD-RIGHT
        false,  // Body lean BACKWARD-LEFT
        false,  // Body lean FORWARD-RIGHT
        false,   // Body lean FORWARD-LEFT
        false,          // Dance demo
        false,         // Mics ON/OFF
        false          // RESET
};             


// ******************* Mapping rsv2ilities to robosapien command codes ****************************************
const uint16_t Ability2CodeMap[ABILITIES_NUM] = {
        rsv2WALK_FORWARD,
        rsv2WALK_BACKWARD,         // walk BACKWARD
        rsv2TURN_RIGHT,            // turn RIGHT
        rsv2TURN_LEFT,             // turn LEFT
        rsv2WALK_FORWARDRIGHT,    // walk FORWARD- RIGHT
        rsv2WALK_FORWARDLEFT,     // walk FORWARD-LEFT
        rsv2WALK_BACKWARDRIGHT,   // walk BACKWARD-RIGHT
        rsv2WALK_BACKWARDLEFT,    // walk BACKWARD-LEFT

        // Buldozer walk
        rsv2BULLDOZER_FORWARD,      // Bulldozer FORWARD
        rsv2BULLDOZER_BACKWARD,     // Bulldozer BACKWARD
        // walk/turn rsv2ilities ends

        // Upper Body+Head motion
        rsv2LEAN_BACKWARD,       //  LEAN FORWARD
        rsv2LEAN_FORWARD,      //  LEAN BACKWARD
        rsv2LEAN_RIGHT,         //  LEAN RIGHT
        rsv2LEAN_LEFT,          //  LEAN LEFT
        rsv2LEAN_BACKWARDRIGHT,  // LEAN FORWARD-RIGHT
        rsv2LEAN_BACKWARDLEFT,   // LEAN FORWARD-LEFT
        rsv2LEAN_FORWARDRIGHT, // LEAN BACKWARD-RIGHT
        rsv2LEAN_FORWARDLEFT,  // LEAN BACKWARD-LEFT
        // Upper Body+Head motion commands ends


        rsv2FREE_ROAM,     // 
        rsv2STOP,          // STOP

        // right arm motion commands
        rsv2RIGHT_ARM_THROW,        // Right Arm THROW
        rsv2RIGHT_ARM_LOW_PICKUP,   // Right arm LOW PICKUP
        rsv2RIGHT_ARM_HIGH_PICKUP,  // Right Arm HIGH PICKUP
        rsv2RIGHT_ARM_GRAB,         // Right Arm GRAB (from the floor)
        rsv2RIGHT_ARM_GIVE,         // Right Arm GIVE
        rsv2RIGHT_ARM_ROLL,         // Right Arm ROLL
        // directional right arm motion
        rsv2RIGHT_ARM_UP,           // Right Arm UP
        rsv2RIGHT_ARM_DOWN,         // Right Arm DOWN
        rsv2RIGHT_ARM_RIGHT,        // Right Arm RIGHT
        rsv2RIGHT_ARM_LEFT,         // Right Arm LEFT
        rsv2RIGHT_ARM_UPRIGHT,      // Right Arm UP AND RIGHT
        rsv2RIGHT_ARM_UPLEFT,       // Right Arm UP and LEFT
        rsv2RIGHT_ARM_DOWNRIGHT,    // Right Arm DOWN and RIGHT
        rsv2RIGHT_ARM_DOWNLEFT,     // Right Arm DOWN and LEFT
        // right arm motion commands ends

        // left arm motion commands
        rsv2LEFT_ARM_THROW,         // Left Arm THROW
        rsv2LEFT_ARM_LOW_PICKUP,    // Left Arm LOW PICKUP
        rsv2LEFT_ARM_HIGH_PICKUP,   // Left Arm HIGH PICKUP
        rsv2LEFT_ARM_GRAB,          // Left Arm GRAB
        rsv2LEFT_ARM_GIVE,          // Left Arm GIVE
        rsv2LEFT_ARM_ROLL,          // Left Arm ROLL
        // directional left arm motion
        rsv2LEFT_ARM_UP,            // Left Arm UP
        rsv2LEFT_ARM_DOWN,          // Left Arm DOWN
        rsv2LEFT_ARM_RIGHT,         // Left Arm RIGHT
        rsv2LEFT_ARM_LEFT,          // Left Arm LEFT
        rsv2LEFT_ARM_UPRIGHT,       // Left Arm UP and RIGHT
        rsv2LEFT_ARM_UPLEFT,        // Left Arm UP and LEFT
        rsv2LEFT_ARM_DOWNRIGHT,     // Left Arm DOWN and RIGHT
        rsv2LEFT_ARM_DOWNLEFT,      // Left Arm DOWN and LEFT
  
        rsv2LIE_DOWN,         // LIE DOWN->SIT UP->LIE DOWN->STAND UP
        rsv2GET_UP,           // GET UP

        // special moves 
        rsv2RIGHT_KICK,       // RIGHT KICK 
        rsv2RIGHT_PUSH,       // RIGHT PUSH
        rsv2RIGHT_CHOP,       // RIGHT CHOP
        rsv2LEFT_CHOP,        // LEFT CHOP
        rsv2LEFT_PUSH,        // LEFT PUSH
        rsv2LEFT_KICK,        // LEFT KICK
        rsv2OOPS,             // OOPS (whatever that means)

        // Hip and Waist Tilt Motion Commands
        rsv2WAIST_FORWARD,        // Waist-Hip tilt FORWARD
        rsv2WAIST_BACKWARD,       // Waist-Hip tilt BCKWARDS
        rsv2WAIST_RIGHT,          // Waist-Hip tilt RIGHT
        rsv2WAIST_LEFT,           // Waist-Hip tilt LEFT
        rsv2WAIST_FORWARDRIGHT,   // Waist-Hip tilt FORWARD and RIGHT
        rsv2WAIST_FORWARDLEFT,   // Waist-Hip tilt FORWARD and LEFT
        rsv2WAIST_BACKWARDRIGHT, // Waist-Hip tilt BACKWARD and RIGHT
        rsv2WAIST_BACKWARDLEFT,  // Waist-Hip tilt BEACKWARDS and LEFT
        // Hip and Waist tilt motion ends

        // Misc motion
        rsv2HIGH_FIVE,         // HIGH FIVE

        // Both Arms Motion Commands
        rsv2BOTHARMS_UP,         // Both Arms UP
        rsv2BOTHARMS_DOWN,       // Both Arms DOWN
        rsv2BOTHARMS_RIGHT,      // Both Arms RIGHT
        rsv2BOTHARMS_LEFT,       // Both Arms LEFT
        rsv2BOTHARMS_UPRIGHT,    // Both Arms UP and RIGHT
        rsv2BOTHARMS_UPLEFT,     // Both Arms UP and LEFT
        rsv2BOTHARMS_DOWNRIGHT,  // Both Arms DOWN and RIGHT
        rsv2BOTHARMS_DOWNLEFT,   // Both Arms DOWN and LEFT
        // Both Arms motion comands ends

        // Behavioral expressions etc. and other moves
        rsv2LAUGH,            // LAUGH
        rsv2INSULT,           // INSULT
        rsv2RIGHT_ARM_DROP,   // Right Arm DROP
        rsv2LEFT_ARM_DROP,    // Left Arm DROP
        rsv2PLAN,             // PLAN (very curious about this...)
        rsv2SPARE_CHANGE,     // SPARE CHANGE (again very curious...)
        rsv2HEY_BABY,         // HEY BABY (!)
        // more expressions...
        rsv2ROAR,             // ROARRRRRR
        rsv2DIODE,            // DIODE (? - should try that out...)
        rsv2FETCH,            // FETCH (? again curious...)
        rsv2DANGER,           // DANGER (? - curious...)
        rsv2CALM_DOWN,        // CALM DOWN
        rsv2HUG,              // HUG!
        rsv2BURP,             // BURP 
        // behavior and misc move commands ends 

        // Head only Motion Commands
        rsv2HEAD_UP,         // HEAD UP
        rsv2HEAD_DOWN,       // HEAD DOWN
        rsv2HEAD_RIGHT,      // HEAD RIGHT
        rsv2HEAD_LEFT,       // HEAD LEFT
        rsv2HEAD_UPRIGHT,    // HEAD UP-RIGHT 
        rsv2HEAD_UPLEFT,     // HEAD UP-LEFT
        rsv2HEAD_DOWNRIGHT,  // HEAD DOWN-RIGHT
        rsv2HEAD_DOWNLEFT,   // HEAD DOWN-LEFT
        // Head Motion Comands ends

        // Upper Body ONLY motion commands
        rsv2BODY_BACKWARD,    // Body lean BACKWARD
        rsv2BODY_FORWARD,     // Body lean FORWARD 
        rsv2BODY_RIGHT,        // Body lean RIGHT
        rsv2BODY_LEFT,         // Body lean LEFT
        rsv2BODY_BACKWARDRIGHT,  // Body lean BACKWARD-RIGHT
        rsv2BODY_BACKWARDLEFT,  // Body lean BACKWARD-LEFT
        rsv2BODY_FORWARDRIGHT,  // Body lean FORWARD-RIGHT
        rsv2BODY_FORWARDLEFT,   // Body lean FORWARD-LEFT
        rsv2DANCE_DEMO,          // Dance demo
        rsv2MIC_SENSORS_ONOFF,   // MICS ON/OFF
        rsv2RESET
};             


// *********************** Command Transmission Parameters (time intervals in usec) *************************
const int transmissionBREAK = 1600; // break interval (low)
const int transmissionSTART = 3000; // start interval (low)
const int transmissionZERO = 1000;  // logic "0" interval (high)
const int transmissionONE = 3700;   // logic "1" interval (high)
// ****************************** Transmission Parameters ends **********************************************


class RobosapienV2 {
  // ******* I/O pin assignments ***********
  public: static const int pinCOMMAND_LINE = 13;              // OUTPUT
          // ------------------ DIGITAL SENSOR INPUTS ----------------------------
          static const int pinLEFT_FOOT_REAR_BUMPER = 12;     // INPUT   (Normally HIGH)
          static const int pinLEFT_FOOT_FRONT_BUMPER = 11;    // INPUT   (Normally HIGH)
          static const int pinRIGHT_FOOT_REAR_BUMPER = 10;    // INPUT   (Normally HIGH) 
          static const int pinRIGHT_FOOT_FRONT_BUMPER = 9;    // INPUT   (Normally HIGH)
          static const int pinLEFT_HAND_BUMPER = 8;           // INPUT   (Normally LOW)
          static const int pinRIGHT_HAND_BUMPER = 7;          // INPUT   (Normally LOW)
          
          // grip switches
          static const int pinLEFT_HAND_PICKUP = 6;                 // INPUT - LEFT grip CLOSED
          static const int pinRIGHT_HAND_PICKUP = 5;                // INPUT - RUGHT grip OPEN / CLOSED

          // wrist encoders
          static const int pinLEFT_WRIST_ENCODER_BIT0 = 46;
          static const int pinLEFT_WRIST_ENCODER_BIT1 = 48;
          
          static const int pinRIGHT_WRIST_ENCODER_BIT0 = 50;
          static const int pinRIGHT_WRIST_ENCODER_BIT1 = 52;
          
          // ---------------------- DIGITAL SENSORS DONE -------------------------
          
          // ---------------------- ANALOG SENSORS -------------------------------
          // Camera Signals
          static const int pinCAMERA_RST = 0;            // BIT INPUT ?? - Possibly indicates Camera active
          static const int pinCAMERA_HSD = 1;            // CAMERA HSD analog signal
          static const int pinCAMERA_RDY = 2;            // CAMERA RDY analog signal
          static const int pinCAMERA_SK = 3;             // CAMERA SK analog signal
          // Head IR Sensors
          static const int pinLEFT_HEAD_IR = 4;         // LEFT HEAD IR analog signal (sampling per 50 ms)
          static const int pinCENTER_HEAD_IR = 5;       // CENTER HEAD IR analog signal (sampling per 50 ms)
          static const int pinRIGHT_HEAD_IR = 6;        // RIGHT HEAD IR analog signal (sampling per 50 ms)
          // MICS
          static const int pinLEFT_MIC = 7;    // LEFT EAR mic sensor
          static const int pinRIGHT_MIC = 8;   // RIGHT EAR mic sensor
          
          // Pots
          static const int pinLEFT_SHOULDER_POT = 9;    // LEFT SHOULDER potentiometer
          static const int pinRIGHT_SHOULDER_POT = 10;   // RIGHT SHOULDER potentiometer

  // Class members
  public:  HardwareSerial* pRadioPort; // index of serial port connected to R/F transceiver
           CMUCAM* pCamera;
           
           EZRoboNetDevice* pNetDevice;
           
           // frame sending flag
           byte CMUCAMDataReceiver;
           
  public:  // switch sensors
           uint8_t sensorLeft_Foot_Front_Bumper;
           uint8_t sensorLeft_Foot_Rear_Bumper;
           uint8_t sensorRight_Foot_Front_Bumper;
           uint8_t sensorRight_Foot_Rear_Bumper;
           uint8_t sensorLeft_Hand_Bumper;
           uint8_t sensorRight_Hand_Bumper;
           uint8_t sensorLeft_Hand_Pickup;
           uint8_t sensorRight_Hand_Pickup;
           // the wrist encoders can have values from 0 - 3.
           uint8_t sensorLeft_Wrist_Encoder;
           uint8_t sensorRight_Wrist_Encoder;
           
           // analog sensors
           uint16_t sensorCamera_HSD;
           uint16_t sensorCamera_RST;
           uint16_t sensorCamera_RDY;
           uint16_t sensorCamera_SK;
           
           uint16_t sensorLeft_Shoulder_Pot;
           uint16_t sensorRight_Shoulder_Pot;
           
           uint16_t sensorLeft_Mic;
           uint16_t sensorRight_Mic;
           
           uint16_t sensorLeft_Head_IR;
           uint16_t sensorCenter_Head_IR;
           uint16_t sensorRight_Head_IR;
           // the following members contain recent triggers of the mic and the bumper sensors
           uint8_t sensorLeft_Mic_Triggered;
           uint8_t sensorRight_Mic_Triggered;
           uint8_t sensorLeft_Foot_Rear_Triggered;
           uint8_t sensorLeft_Foot_Front_Triggered;
           uint8_t sensorRight_Foot_Rear_Triggered;
           uint8_t sensorRight_Foot_Front_Triggered;
           uint8_t sensorLeft_Hand_Triggered;
           uint8_t sensorRight_Hand_Triggered;

           // ability execution flags
           boolean flagAbilityExecuting, flagAbilityDone;
           // ability sender id
           byte CommandSenderID;
           // current ability execution time in seconds
           byte AbilityExecTime;
           // start of execution timer stamp in milliseconds
           unsigned long AbExecStartTime;
           
           // a falg indicating that following execution the rsv2 should be stopped
           boolean AbilityShouldStop;
           


// constructor   
public: RobosapienV2(HardwareSerial* RadioComm, byte node_id, HardwareSerial* CMUcamComm);

// command transmission 
public: void transmitCommand(uint16_t cmd);

// poll all the sensors
public: void pollSensors();

// Left Foot Rear bumper
public: uint8_t getLeftFootRearBumper();
// Left Foot Front bumper   
public: uint8_t getLeftFootFrontBumper();
// Right Foot Rear bumper
public: uint8_t getRightFootRearBumper();
// Right Foot Front bumper   
public: uint8_t getRightFootFrontBumper();
// Left Hand Bumper
public: uint8_t getLeftHandBumper();
// Right Hand Bumper
public: uint8_t getRightHandBumper();
// Left Hand pickup
public: uint8_t getLeftHandPickup();
// Right Hand Pickup
public: uint8_t getRightHandPickup();

// Left wrist encoder (2 bits)
public: uint8_t getLeftWristEncoder();

// Right wrist encoder (2 bits)
public: uint8_t getRightWristEncoder();

// Camera HSD
public: uint16_t getCameraHSD();
// Camera RDY
public: uint16_t getCameraRDY();
// Camera RST
public: uint16_t getCameraRST();
// camera SK
public: uint16_t getCameraSK();
// Left Shoulder pot
public: uint16_t getLeftShoulderPot();
// Right Shoulder pot
public: uint16_t getRightShoulderPot();
// Left Head IR
public: uint16_t getLeftHeadIR();
// Center Head IR
public: uint16_t getCenterHeadIR();
// Right Head IR
public: uint16_t getRightHeadIR();
// Left Mic
public: uint16_t getLeftMic();
// Right Mic
public: uint16_t getRightMic();


// here comes methods concerning the radio comms

// handle Messages "picks" up packets available in the
// NetDevice and triggers the appropriate actions
public: void handleMessages();

// decodes a manual command fron the network and riggers the corresponding ability
public: void useAbility(byte ability);

// creates a Message containing the sensory data previously observed 
public: CommandMsg* createSensorDataMessage();

// creates a packet containing the sensor readings of the RSV2
public: EZPacket* createSensorDataPacket(byte receiverid);

// create a COMMAND DONE message
CommandMsg* createCommandDoneMessage();

// create a COMMAND DONE message
EZPacket* createCommandDonePacket(byte receiverid);

};       
          
          
#endif
