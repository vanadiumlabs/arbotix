//=============================================================================
// Based upon Kurt's PX Reactor arm code.
// https://github.com/KurtE
// This code provides serial control of the Interbotix line of robotic arms, which are sold by Trossen
// Robotics: http://www.trossenrobotics.com/robotic-arms.aspx
// http://learn.trossenrobotics.com/interbotix/robot-arms
//=============================================================================
//armlink packet structure is as follows
//
// 255, XH, XL, YH, YL, ZH, ZL, WAH, WAL, WRH, WAL, GH, GL, DTIME, BUTTONS, EXT, CHECKSUM
//
// (please note, actual XYZ min/max for specific arm defined below)
//
// Protocol value ranges
//
// XH = high byte X-axis
// XL = low byte, 0-1023 (-512 through +512 via armlink)
//
// YH = high byte Y-axis
// YL = low byte, 0-1023
//
// ZH = high byte Z-axis
// ZL = low byte, 0-1023 
//
// WAH = high byte (unused for now, placeholder for higher res wrist angle)
// WAL = low byte, 0-180 (-90 through +90 via armlink)
//
// WRH = high byte 
// WRL = low byte, 0-1023. 512 center
//
// GH = high byte
// GL = low byte, 0-512. 256 center
//
// DTIME = byte. DTIME*16 = interpolation delta time
//
// Buttons = byte (not implemented)
//
// EXT = byte. Extended instruction set.
// EXT < 16 = no action
// EXT = 32 = 3D Cartesian IK
// EXT = 48 = Cylindrical IK Xaxis = 0-4096 value, untested
// EXT = 64 = BackHoe aka passthrough UNTESTED
//
// CHECKSUM = (unsigned char)(255 - (XH+XL+YH+YL+ZH+ZL+WAH+WAL+WRH+WRL+GH+GL+DTIME+BUTTONS+EXT)%256)

//  This code is a Work In Progress and is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
//  
//=============================================================================

//=============================================================================
// Define Options
//=============================================================================

#define PINCHER 1
#define REACTOR 2
#define WIDOWX 3

//#define ARMTYPE PINCHER
//#define ARMTYPE REACTOR
//#define ARMTYPE WIDOWX

#if !defined(ARMTYPE) 
   #error YOU HAVE TO SELECT THE ARM YOU ARE USING! Uncomment the correct line above for your arm
#endif




#define SOUND_PIN    7      // Tell system we have added speaker to IO pin 1
#define MAX_SERVO_DELTA_PERSEC 512
//#define DEBUG             // Enable Debug mode via serial

//=============================================================================
// Global Include files
//=============================================================================
#include <ax12.h>
#include <BioloidController.h>
#include <ArmLink.h>
#include "InputControl.h"

//=============================================================================
// Global Objects
//=============================================================================
BioloidController bioloid = BioloidController(1000000);
ArmLink armlink = ArmLink();



// Message informatino
unsigned long   ulLastMsgTime;          // Keep track of when the last message arrived to see if controller off
byte            buttonsPrev;            // will use when we wish to only process a button press once



//===================================================================================================
// Setup 
//====================================================================================================
void setup() {
  //Serial activity LED output
  pinMode(0,OUTPUT);  
  // Lets initialize the Serial Port
  Serial.begin(38400);


  delay(50);
  IDPacket();
  Serial.println("Interbotix Robot Arm Online.");
  

  // Next initialize the Bioloid
  bioloid.poseSize = CNT_SERVOS;

  // Read in the current positions...
  bioloid.readPose();
  delay(100);
  // Start off to put arm to sleep...
  PutArmToSleep();
  //Send ID Packet

  MSound(3, 60, 2000, 80, 2250, 100, 2500);

  //set Gripper Compliance so it doesn't tear itself apart
  ax12SetRegister(SID_GRIP, AX_CW_COMPLIANCE_SLOPE, 128);
  ax12SetRegister(SID_GRIP, AX_CCW_COMPLIANCE_SLOPE, 128);

}


//===================================================================================================
// loop: Our main Loop!
//===================================================================================================
void loop() {

  boolean fChanged = false;

  if (armlink.ReadMsgs()) {

    digitalWrite(0,HIGH-digitalRead(0));    

    // Check EXT packet to determine action. Found in InputControl.h
    ExtArmState();  
    DigitalOutputs();

    // See if the Arm is active yet...
    if (g_fArmActive) {

      sBase = g_sBase;
      sShoulder = g_sShoulder;
      sElbow = g_sElbow; 
      sWrist = g_sWrist;
      sWristRot = g_sWristRot;      
      sGrip = g_sGrip;

      // Set InputControl function based on which IKMode we're in

      switch (g_bIKMode) {
      case IKM_IK3D_CARTESIAN:
        fChanged |= ProcessUserInput3D();
        break;
      case IKM_IK3D_CARTESIAN_90:
        fChanged |= ProcessUserInput3D90();
        break;          
      case IKM_CYLINDRICAL:
        fChanged |= ProcessUserInputCylindrical();       
        break;
      case IKM_CYLINDRICAL_90:
        fChanged |= ProcessUserInputCylindrical90();       
        break;
      case IKM_BACKHOE:
        fChanged |= ProcessUserInputBackHoe();
        break;
      }
      // If something changed and we are not in an error condition
      if (fChanged && (g_bIKStatus != IKS_ERROR)) {
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, sDeltaTime, true);
      }
      else if (bioloid.interpolating > 0) {
        bioloid.interpolateStep();
      }
    }
    //    buttonsPrev = armlink.buttons;

  }
  else {
    if (bioloid.interpolating > 0) {
      bioloid.interpolateStep();
    }
  }
} 






// BUGBUG:: Move to some library...
//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
#ifdef SOUND_PIN
void SoundNoTimer(unsigned long duration,  unsigned int frequency)
{
#ifdef __AVR__
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;
#else
  volatile uint32_t *pin_port;
  volatile uint16_t pin_mask;
#endif
  long toggle_count = 0;
  long lusDelayPerHalfCycle;

  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(SOUND_PIN));
  pin_mask = digitalPinToBitMask(SOUND_PIN);

  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    *pin_port ^= pin_mask;

    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  *pin_port &= ~(pin_mask);  // keep pin low after stop

}

void MSound(byte cNotes, ...)
{
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);

  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}
#else
void MSound(byte cNotes, ...)
{
};
#endif





