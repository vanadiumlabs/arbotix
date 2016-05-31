#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H

#include "Kinematics.h"
#include "GlobalArm.h"

extern ArmLink armlink;

extern void MoveArmToHome(void);
extern void MoveArmTo90Home(void);
extern void IDPacket(void);
extern void PutArmToSleep(void);
extern void ReportAnalog(unsigned char, unsigned int);
extern void ReportServoRegister(unsigned char,  int,  int,  int);
extern void SetServoRegister(unsigned char,  int,  int,  int, int);

//===================================================================================================
// Check EXT packet to determine action
//===================================================================================================
   void ExtArmState(){
       
     
      g_fArmActive = false;  //start with g_fArmActive being false, so that only movement commands will activate the arm movement in the main loop
        
     if(armlink.ext  == 0){
        // no special action, '0' indicates that a movement command has come through
        g_fArmActive = true;
     }
      
      
      else if(armlink.ext == 0x11){  //17
        EmergencyStop();
        IDPacket();        
      }
      
      else if(armlink.ext == 0x20){  //32
        g_bIKMode = IKM_IK3D_CARTESIAN;
        MoveArmToHome(); 
        IDPacket();
      }
      else if(armlink.ext == 0x28){  //40
        g_bIKMode = IKM_IK3D_CARTESIAN_90;
        MoveArmTo90Home(); 
        IDPacket();
      }        
      else if(armlink.ext == 0x30){  //48
        g_bIKMode = IKM_CYLINDRICAL;
        MoveArmToHome(); 
        IDPacket();        
      }
      else if(armlink.ext == 0x38){  //56
        g_bIKMode = IKM_CYLINDRICAL_90;
        MoveArmTo90Home(); 
        IDPacket();        
      }        
      else if(armlink.ext == 0x40){  //64
        g_bIKMode = IKM_BACKHOE;
        MoveArmToHome(); 
        IDPacket();        
      }
      else if(armlink.ext == 0x48){  //72
      // do something
      }
      else if(armlink.ext == 0x50){  //80
        MoveArmToHome(); 
        IDPacket();        
      }
      else if(armlink.ext == 0x58){  //88
        MoveArmTo90Home();
        IDPacket();
      }
      else if(armlink.ext == 0x60){  //96
        PutArmToSleep();
        IDPacket();        
      }
      else if(armlink.ext == 0x70){  //112
        IDPacket();
      }
      else if(armlink.ext == 0x80){  //128
        //IK value response
      }
      else if(armlink.ext == 0x81){  //129
         ReportServoRegister(armlink.ext,  armlink.Xaxis,  armlink.Yaxis,  armlink.Zaxis);
      }
      else if(armlink.ext == 0x82){  //130
         SetServoRegister(armlink.ext,  armlink.Xaxis,  armlink.Yaxis,  armlink.Zaxis, armlink.W_ang);
      }
      
      
      else if(armlink.ext >= 0xC8){  //200
        // read analogs
        ReportAnalog(armlink.ext, analogRead(armlink.ext - 0xC8));
      }
    }




//===================================================================================================
// ProcessUserInput3D: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInput3D(void) {
  
  // We Are in IK mode, so figure out what position we would like the Arm to move to.
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //
  boolean fChanged = false;
  int   sIKX;                  // Current X value in mm
  int   sIKY;                  //
  int   sIKZ;
  int   sIKGA;    

  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...

  if (g_bIKStatus == IKS_SUCCESS) {
    
// Keep IK values within limits
//
    sIKX = min(max((armlink.Xaxis-X_OFFSET), IK_MIN_X), IK_MAX_X);  
    sIKY = min(max(armlink.Yaxis, IK_MIN_Y), IK_MAX_Y);    
    sIKZ = min(max(armlink.Zaxis, IK_MIN_Z), IK_MAX_Z);
    sIKGA = min(max((armlink.W_ang-GA_OFFSET), IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords..
    sWristRot = min(max(armlink.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armlink.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armlink.dtime*16;
    
  }

  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  
  
  if (fChanged) {
    // report
    g_bIKStatus = doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 
  }
  return fChanged;

}


//===================================================================================================
// ProcessUserInput3D90: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInput3D90(void) {
  
  // We Are in IK mode, so figure out what position we would like the Arm to move to.
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //
  boolean fChanged = false;
  int   sIKX;                  // Current X value in mm
  int   sIKY;                  //
  int   sIKZ;
  int   sIKGA;    

  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...

  if (g_bIKStatus == IKS_SUCCESS) {
    
// Keep IK values within limits
//
    sIKX = min(max((armlink.Xaxis-X_OFFSET), IK_MIN_X_90), IK_MAX_X_90);  
    sIKY = min(max(armlink.Yaxis, IK_MIN_Y_90), IK_MAX_Y_90);    
    sIKZ = min(max(armlink.Zaxis, IK_MIN_Z_90), IK_MAX_Z_90);
    sIKGA = min(max((armlink.W_ang-GA_OFFSET), IK_MIN_GA_90), IK_MAX_GA_90);  // Currently in Servo coords..
    sWristRot = min(max(armlink.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armlink.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armlink.dtime*16;
    
  }

  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  
  
  if (fChanged) {
    // report
    g_bIKStatus = doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 
  }
  return fChanged;

}

//===================================================================================================
// ProcessUserInputCylindrical: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInputCylindrical() {
  // We Are in IK mode, so figure out what position we would like the Arm to move to.
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //
  boolean fChanged = false;
  int   sIKY;                  // Distance from base in mm
  int   sIKZ;
  int   sIKGA;

  // Will try combination of the other two modes.  Will see if I need to do the Limits on the IK values
  // or simply use the information from the Warning/Error from last call to the IK function...
  sIKY = g_sIKY;
  sIKZ = g_sIKZ;
  sIKGA = g_sIKGA;

  // The base rotate is real simple, just allow it to rotate in the min/max range...
  sBase = min(max(armlink.Xaxis, BASE_MIN), BASE_MAX);

  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...
  // Use Y for 2d distance from base
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKY > 0) && (armlink.Yaxis < 0)) || ((g_sIKY < 0) && (armlink.Yaxis > 0)))
    sIKY = min(max(armlink.Yaxis, IK_MIN_Y), IK_MAX_Y);

  // Now Z coordinate...
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKZ > 0) && (armlink.Zaxis < 0)) || ((g_sIKZ < 0) && (armlink.Zaxis > 0)))
    sIKZ = min(max(armlink.Zaxis, IK_MIN_Z), IK_MAX_Z);

  // And gripper angle.  May leave in Min/Max here for other reasons...   

  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKGA > 0) && (armlink.W_ang < 0)) || ((g_sIKGA < 0) && (armlink.W_ang > 0)))
    sIKGA = min(max((armlink.W_ang-GA_OFFSET), IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...

    sWristRot = min(max(armlink.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armlink.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armlink.dtime*16;
   
  fChanged = (sBase != g_sBase) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);


  if (fChanged) {
    g_bIKStatus = doArmIK(false, sBase, sIKY, sIKZ, sIKGA);
  }
  return fChanged;
}


//===================================================================================================
// ProcessUserInputCylindrical90: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInputCylindrical90() {
  // We Are in IK mode, so figure out what position we would like the Arm to move to.
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //
  boolean fChanged = false;
  int   sIKY;                  // Distance from base in mm
  int   sIKZ;
  int   sIKGA;

  // Will try combination of the other two modes.  Will see if I need to do the Limits on the IK values
  // or simply use the information from the Warning/Error from last call to the IK function...
  sIKY = g_sIKY;
  sIKZ = g_sIKZ;
  sIKGA = g_sIKGA;

  // The base rotate is real simple, just allow it to rotate in the min/max range...
  sBase = min(max(armlink.Xaxis, BASE_MIN), BASE_MAX);

  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...
  // Use Y for 2d distance from base
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKY > 0) && (armlink.Yaxis < 0)) || ((g_sIKY < 0) && (armlink.Yaxis > 0)))
    sIKY = min(max(armlink.Yaxis, IK_MIN_Y_90), IK_MAX_Y_90);

  // Now Z coordinate...
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKZ > 0) && (armlink.Zaxis < 0)) || ((g_sIKZ < 0) && (armlink.Zaxis > 0)))
    sIKZ = min(max(armlink.Zaxis, IK_MIN_Z_90), IK_MAX_Z_90);

  // And gripper angle.  May leave in Min/Max here for other reasons...   

  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKGA > 0) && (armlink.W_ang < 0)) || ((g_sIKGA < 0) && (armlink.W_ang > 0)))
    sIKGA = min(max((armlink.W_ang-GA_OFFSET), IK_MIN_GA_90), IK_MAX_GA_90);  // Currently in Servo coords...

    sWristRot = min(max(armlink.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armlink.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armlink.dtime*16;
   
  fChanged = (sBase != g_sBase) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);


  if (fChanged) {
    g_bIKStatus = doArmIK(false, sBase, sIKY, sIKZ, sIKGA);
  }
  return fChanged;
}

//===================================================================================================
// ProcessUserInputBackHoe: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInputBackHoe() {
  // lets update positions with the 4 joystick values
  // First the base
  boolean fChanged = false;
  sBase = min(max(armlink.Xaxis, BASE_MIN), BASE_MAX);
  // Now the Boom
  sShoulder = min(max(armlink.Yaxis, SHOULDER_MIN), SHOULDER_MAX);
  // Now the Dipper 
  sElbow = min(max(armlink.Zaxis, ELBOW_MIN), ELBOW_MAX);
  // Bucket Curl
  sWrist = min(max(armlink.W_ang, WRIST_MIN), WRIST_MAX);
    sWristRot = min(max(armlink.W_rot, WROT_MIN), WROT_MAX);
    sGrip = min(max(armlink.Grip, GRIP_MIN), GRIP_MAX);
    sDeltaTime = armlink.dtime*16;

  fChanged = (sBase != g_sBase) || (sShoulder != g_sShoulder) || (sElbow != g_sElbow) || (sWrist != g_sWrist) || (sWristRot != g_sWristRot) || (sGrip != g_sGrip);  
  return fChanged;
}

void ReportAnalog(unsigned char command, unsigned int value){
  unsigned char AH;
  unsigned char AL;
  AH = ((value & 0xFF00) >> 8);
  AL = (value & 0x00FF);
  Serial.write(0xff);
  Serial.write(command);
  Serial.write(AH);
  Serial.write(AL);
  Serial.write((unsigned char)(255 - (command+AH+AL)%256));
}


void ReportServoRegister(unsigned char command, int id, int registerNumber, int length)
{

  int registerValue = ax12GetRegister(id, registerNumber, length);


  unsigned char registerHigh;
  unsigned char registerLow;
  registerHigh = ((registerValue & 0xFF00) >> 8);
  registerLow = (registerValue & 0x00FF);
  Serial.write(0xff);
  Serial.write(command);
  Serial.write(registerHigh);
  Serial.write(registerLow);
  Serial.write((unsigned char)(255 - (command+registerHigh+registerLow)%256));

}
void SetServoRegister(unsigned char command, int id, int registerNumber, int length, int data)
{

  unsigned char registerHigh;
  unsigned char registerLow;
  
  if(length == 1)
  {
    ax12SetRegister(id, registerNumber, data);
  }
  else if (length == 2)
  {
    ax12SetRegister2(id, registerNumber, data);
  }
  


  registerHigh = ((data & 0xFF00) >> 8);
  registerLow = (data & 0x00FF);
  Serial.write(0xff);
  Serial.write(command); // should this be an error bit?
  Serial.write(registerHigh);
  Serial.write(registerLow);
  Serial.write((unsigned char)(255 - (command+registerHigh+registerLow)%256));

}





void IDPacket()  {
  Serial.write(0xFF);
  Serial.write((unsigned char) ARMID);
  Serial.write((unsigned char) g_bIKMode);
  Serial.write((unsigned char) 0);
  Serial.write((unsigned char)(255 - (ARMID+g_bIKMode+0)%256));
  
}



void DigitalOutputs(){
         // First bit = D1, 2nd bit = D2, etc. 
        int i;
        for(i=0;i<7;i++){
        unsigned char button = (armlink.buttons>>i)&0x01;
        if(button > 0){
          // button pressed, go high on a pin
          DDRB |= 0x01<<(i+1);
          PORTB |= 0x01<<(i+1);
        }
        else{
          DDRB &= 0xff - (0x01<<(i+1));
          PORTB &= 0xff - (0x01<<(i+1));
        } 
      } 
}



//=============================================================================
//=============================================================================
#endif
