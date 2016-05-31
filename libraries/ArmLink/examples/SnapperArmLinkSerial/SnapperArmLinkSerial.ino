/*
//=============================================================================
   Sources used:
   https://github.com/KurtE
  
   http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino

   Application Note 44 - Controlling a Lynx6 Robotic Arm 
   http://www.micromegacorp.com/appnotes.html
   http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf

//=============================================================================
//Armcontrol packet structure is as follows
//
// 255, XH, XL, YH, YL, ZH, ZL, WAH, WAL, WRH, WAL, GH, GL, DTIME, BUTTONS, EXT, CHECKSUM
//
// (please note, actual XYZ min/max for specific arm defined below)
//
// Protocol value ranges
//
// XH = high byte X-axis
// XL = low byte, 0-1023 (-512 through +512 via ArmControl)
//
// YH = high byte Y-axis
// YL = low byte, 0-1023
//
// ZH = high byte Z-axis
// ZL = low byte, 0-1023 
//
// WAH = high byte (unused for now, placeholder for higher res wrist angle)
// WAL = low byte, 0-180 (-90 through +90 via ArmControl)
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
// EXT = 48 = Cylindrical IK
// 
//
// CHECKSUM = (unsigned char)(255 - (XH+XL+YH+YL+ZH+ZL+WAH+WAL+WRH+WRL+GH+GL+DTIME+BUTTONS+EXT)%256)

//  This code is a Work In Progress and is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
//  
//=============================================================================

  Sources used:
   http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino

   Application Note 44 - Controlling a Lynx6 Robotic Arm 
   http://www.micromegacorp.com/appnotes.html
   http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf

*/

#define ROBOT_GEEK_9G_GRIPPER 1
#define ROBOT_GEEK_PARALLEL_GRIPPER 2

//The 9G gripper is the gripper with the small blue 9g servo
//The Parralle gripper has a full robotgeek servo and paralle rails
//Uncomment one of the following lines depending on which gripper you are using.
//#define GRIPPER_TYPE ROBOT_GEEK_9G_GRIPPER
#define GRIPPER_TYPE ROBOT_GEEK_PARALLEL_GRIPPER

#ifndef GRIPPER_TYPE
   #error YOU HAVE TO SELECT THE GRIPPER YOU ARE USING! Uncomment the correct line above for your gripper
#endif

#include <ArmLink.h>
#include <ServoEx.h>
#include "InputControl.h"

ArmLink armlink = ArmLink();
ServoEx    ArmServo[5];


void setup(){
  // Attach servo and set limits
  ArmServo[BAS_SERVO].attach(3, BASE_MIN, BASE_MAX);
  ArmServo[SHL_SERVO].attach(5, SHOULDER_MIN, SHOULDER_MAX);
  ArmServo[ELB_SERVO].attach(6, ELBOW_MIN, ELBOW_MAX);
  ArmServo[WRI_SERVO].attach(9, WRIST_MIN, WRIST_MAX);
  ArmServo[GRI_SERVO].attach(10, GRIPPER_MIN, GRIPPER_MAX);
  // send arm to default X,Y,Z coord
  doArmIK(true, g_sIKX,g_sIKY,g_sIKZ,g_sIKGA);
  SetServo(sDeltaTime);
  // send arm to center position, used for error adjustment

  // start serial
  Serial.begin(38400);
  delay(50);
  IDPacket();
  Serial.println("Starting RobotGeek ArmControl Demo");
  delay(500);
}
 
void loop(){
  //  if we receive data, do something
  if (armlink.ReadMsgs()) 
  {
  boolean fChanged = false;
      //Read EXT packet to see if we need to switch IK modes or do anything else
      ExtArmState();
      //process digital outouts
      DigitalOutputs();
      
      if(g_fArmActive == true)
      {
          
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
//        case IKM_IK3D_CARTESIAN_90:
//          fChanged |= ProcessUserInput3D90();
//          break;          
        case IKM_CYLINDRICAL:
          fChanged |= ProcessUserInputCylindrical();       
          break;
//        case IKM_CYLINDRICAL_90:
//          fChanged |= ProcessUserInputCylindrical90();       
//          break;
        case IKM_BACKHOE:
          fChanged |= ProcessUserInputBackHoe();
          break;
        }
      // If something changed and we are not in an error condition
      if (fChanged && (g_bIKStatus != IKS_ERROR)) {
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, sDeltaTime, true);
      }
   
   
    }
    
    
    
    
        
        
      //Process serial input from ArmControl, translate to working X,Y,Z,GA Coord
      //ProcessUserInput3D();

      //Calculate goal positions of servos based on X,Y,Z,GA coord determined by ProcessUserInput3D()
      //doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 

      //Set servo positions via sDeltaTime interpolation value (set in UserInput as well)
      SetServo(sDeltaTime);
      
  }
 }
 
 

