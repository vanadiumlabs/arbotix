#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "GlobalArm.h"
extern BioloidController bioloid;

// Forward references
extern void MSound(byte cNotes, ...);

//=============================================================================
//=============================================================================
// IK Modes defined, 0-4
enum {
  IKM_IK3D_CARTESIAN, IKM_IK3D_CARTESIAN_90, IKM_CYLINDRICAL, IKM_CYLINDRICAL_90, IKM_BACKHOE};

// status messages for IK return codes..
enum {
  IKS_SUCCESS=0, IKS_WARNING, IKS_ERROR};


#define IK_FUDGE            5     // How much a fudge between warning and error


//=============================================================================
// Global Variables...
//=============================================================================
boolean         g_fArmActive = false;   // Is the arm logically on?
byte            g_bIKMode = IKM_IK3D_CARTESIAN;   // Which mode of operation are we in...
uint8_t         g_bIKStatus = IKS_SUCCESS;   // Status of last call to DoArmIK;
boolean         g_fServosFree = true;

// Current IK values
int            g_sIKGA;                  // IK Gripper angle..
int            g_sIKX;                  // Current X value in mm
int            g_sIKY;                  //
int            g_sIKZ;

// Values for current servo values for the different joints
int             g_sBase;                // Current Base servo value
int             g_sShoulder;            // Current shoulder target 
int             g_sElbow;               // Current
int             g_sWrist;               // Current Wrist value
int             g_sWristRot;            // Current Wrist rotation
int             g_sGrip;                // Current Grip position

int sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip;
int sDeltaTime = 100;




//===================================================================================================
// Convert radians to AX (10 bit goal address) servo position offset. 
//===================================================================================================
int radToAXServo(float rads){
  float val = (rads*100)/51 * 100;
  return (int) val;
}

//===================================================================================================
// Convert radians to MX (12 bit goal address) servo position offset. 
//===================================================================================================
int radToMXServo(float rads){
  float val = (rads*652);
  return (int) val;
}



//===================================================================================================
// Compute Arm IK for 3DOF+Mirrors+Gripper - was based on code by Michael E. Ferguson
// Hacked up by me, to allow different options...
//===================================================================================================
uint8_t doArmIK(boolean fCartesian, int sIKX, int sIKY, int sIKZ, int sIKGA)
{
  int t;
  int sol0;
  uint8_t bRet = IKS_SUCCESS;  // assume success

  if (fCartesian) {
    // first, make this a 2DOF problem... by solving baseAngle, converting to servo pos
#if ARMTYPE == WIDOWX
    sol0 = radToMXServo(atan2(sIKX,sIKY));
#else
    sol0 = radToAXServo(atan2(sIKX,sIKY));
#endif    
    // remove gripper offset from base
    t = sqrt(sq((long)sIKX)+sq((long)sIKY));

    // BUGBUG... Gripper offset support
//#define G 10   
 //   sol0 -= radToServo(atan2((G/2)-G_OFFSET,t));
 
  }
  else {
    // We are in cylindrical mode, probably simply set t to the y we passed in...
    t = sIKY;
  }
  // convert to sIKX/sIKZ plane, remove wrist, prepare to solve other DOF           
  float flGripRad = (float)(sIKGA)*3.14159/180.0;
  long trueX = t - (long)((float)WristLength*cos(flGripRad));   
  long trueZ = sIKZ - BaseHeight - (long)((float)WristLength*sin(flGripRad));

  long im = sqrt(sq(trueX)+sq(trueZ));        // length of imaginary arm
  float q1 = atan2(trueZ,trueX);              // angle between im and X axis
  long d1 = sq(ShoulderLength) - sq(ElbowLength) + sq((long)im);
  long d2 = 2*ShoulderLength*im;
  float q2 = acos((float)d1/float(d2));
  q1 = q1 + q2;

  d1 = sq(ShoulderLength)-sq((long)im)+sq(ElbowLength);
  d2 = 2*ElbowLength*ShoulderLength;
  q2 = acos((float)d1/(float)d2);

#if ARMTYPE == WIDOWX  //Use different radians equation for MX servos
  int sol1 = radToMXServo(q1-1.57);
  int sol2 = radToMXServo(3.14-q2);
  // solve for wrist rotate
  int sol3 = radToMXServo(3.2 + flGripRad - q1 - q2 );
  
#else  //Use different radians equation for AX servos
  int sol1 = radToAXServo(q1-1.57);
  int sol2 = radToAXServo(3.14-q2);
  // solve for wrist rotate
  int sol3 = radToAXServo(3.2 + flGripRad - q1 - q2 );
#endif  


#if ARMTYPE == PINCHER 
  // Lets calculate the actual servo values.
  if (fCartesian) {
    sBase = min(max(BASE_N - sol0, BASE_MIN), BASE_MAX);
  }
  sShoulder = min(max(SHOULDER_N - sol1, SHOULDER_MIN), SHOULDER_MAX);

  sElbow = min(max(ELBOW_N + sol2, ELBOW_MIN), ELBOW_MAX);

  sWrist = min(max(WRIST_N - sol3, WRIST_MIN), WRIST_MAX);

#else
  // Lets calculate the actual servo values.
  if (fCartesian) {
    sBase = min(max(BASE_N - sol0, BASE_MIN), BASE_MAX);
  }
  sShoulder = min(max(SHOULDER_N - sol1, SHOULDER_MIN), SHOULDER_MAX);

  sElbow = min(max(ELBOW_N - sol2, ELBOW_MIN), ELBOW_MAX);

  sWrist = min(max(WRIST_N + sol3, WRIST_MIN), WRIST_MAX);
#endif

  // Remember our current IK positions
  g_sIKX = sIKX; 
  g_sIKY = sIKY;
  g_sIKZ = sIKZ;
  g_sIKGA = sIKGA;
  // Simple test im can not exceed the length of the Shoulder+Elbow joints...

  if (im > (ShoulderLength + ElbowLength)) {
    if (g_bIKStatus != IKS_ERROR) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        Serial.println("IK Error");
      }
#endif
      MSound(2, 50, 3000, 50, 3000);
    }
    bRet = IKS_ERROR;  
  }
  else if(im > (ShoulderLength + ElbowLength-IK_FUDGE)) {
    if (g_bIKStatus != IKS_WARNING) {
#ifdef DEBUG
      if (g_fDebugOutput) {
        Serial.println("IK Warning");
      }
#endif
      MSound(1, 75, 2500);
    }
    bRet = IKS_WARNING;  
  }

  return bRet;
}



//===================================================================================================
// MoveArmTo
//===================================================================================================
void MoveArmTo(int sBase, int sShoulder, int sElbow, int sWrist, int sWristRot, int sGrip, int wTime, boolean fWait) {

  int sMaxDelta;
  int sDelta;

  // First make sure servos are not free...
  if (g_fServosFree) {
    g_fServosFree = false;

    for(uint8_t i=1; i <= CNT_SERVOS; i++) {
      TorqueOn(i);
    }  
    delay(100);    //required due to power dip on TorqueOn
    bioloid.readPose();
  }


  // Make sure the previous movement completed.
  // Need to do it before setNextPos calls as this
  // is used in the interpolating code...
  while (bioloid.interpolating > 0) {
    bioloid.interpolateStep();
    delay(3);
  }

  // Also lets limit how fast the servos will move as to not get whiplash.
  bioloid.setNextPose(SID_BASE, sBase);  

#if ARMTYPE == REACTOR  //double joints on reactor are handled differently
  sMaxDelta = abs(bioloid.getCurPose(SID_RSHOULDER) - sShoulder);
  bioloid.setNextPose(SID_RSHOULDER, sShoulder);
  bioloid.setNextPose(SID_LSHOULDER, 1024-sShoulder);

  sDelta = abs(bioloid.getCurPose(SID_RELBOW) - sElbow);
  if (sDelta > sMaxDelta)
    sMaxDelta = sDelta;
  bioloid.setNextPose(SID_RELBOW, sElbow);
  bioloid.setNextPose(SID_LELBOW, 1024-sElbow);
  
#else    //single joint config
  sMaxDelta = abs(bioloid.getCurPose(SID_SHOULDER) - sShoulder);
  bioloid.setNextPose(SID_SHOULDER, sShoulder);

  sDelta = abs(bioloid.getCurPose(SID_ELBOW) - sElbow);
  if (sDelta > sMaxDelta)
    sMaxDelta = sDelta;
  bioloid.setNextPose(SID_ELBOW, sElbow);
#endif

  sDelta = abs(bioloid.getCurPose(SID_WRIST) - sWrist);
  if (sDelta > sMaxDelta)
    sMaxDelta = sDelta;
  bioloid.setNextPose(SID_WRIST, sWrist);


#ifdef OPT_WRISTROT
  bioloid.setNextPose(SID_WRISTROT, sWristRot); 
#endif  
  bioloid.setNextPose(SID_GRIP, sGrip);


  // Save away the current positions...
  g_sBase = sBase;
  g_sShoulder = sShoulder;
  g_sElbow = sElbow;
  g_sWrist = sWrist;
  g_sWristRot = sWristRot;
  g_sGrip = sGrip;

  // Now start the move - But first make sure we don't move too fast.  
//  if (((long)sMaxDelta*wTime/1000L) > MAX_SERVO_DELTA_PERSEC) {
//    wTime = ((long)sMaxDelta*1000L)/ MAX_SERVO_DELTA_PERSEC;
//  }

  bioloid.interpolateSetup(wTime);

  // Do at least the first movement
  bioloid.interpolateStep();

  // And if asked to, wait for the previous move to complete...
  if (fWait) {
    while (bioloid.interpolating > 0) {
      bioloid.interpolateStep();
      delay(3);
    }
  }
}


//===================================================================================================
// MoveArmToHome
//===================================================================================================
void MoveArmToHome(void) {
//  if (g_bIKMode != IKM_BACKHOE) {
    g_bIKStatus = doArmIK(true, 0, (2*ElbowLength)/3+WristLength, BaseHeight+(2*ShoulderLength)/3, 0);
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 256, 2000, true);
     g_fArmActive = false;
//  }
//  else {
//    g_bIKStatus = IKS_SUCCESS;  // assume sucess soe we will continue to move...
//    MoveArmTo(2048, 2048, 2048, 2048, 512, 256, 2000, true);
//  }
}

//===================================================================================================
// MoveArmTo90Home
//===================================================================================================
void MoveArmTo90Home(void) {

//  if (g_bIKMode != IKM_BACKHOE) {
    g_bIKStatus = doArmIK(true, 0, 150, 30, -90);
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, 512, 256, 2000, true);
     g_fArmActive = false;
//  }
//  else {
//    g_bIKStatus = IKS_SUCCESS;  // assume sucess soe we will continue to move...
//    MoveArmTo(2048, 2048, 2048, 2048, 512, 256, 2000, true);
//  }
}

//===================================================================================================
// PutArmToSleep
//===================================================================================================
void PutArmToSleep(void) {
  g_fArmActive = false;
  
#if ARMTYPE == PINCHER  
  MoveArmTo(512, 400, 1000, 430, 512, 256, 2000, true);

#elif ARMTYPE == REACTOR
  MoveArmTo(512, 212, 212, 512, 512, 256, 2000, true);

#elif ARMTYPE == WIDOWX
  MoveArmTo(2048, 1024, 1024, 1700, 512, 256, 2000, true);
#endif

  // And Relax all of the servos...
  for(uint8_t i=1; i <= CNT_SERVOS; i++) {
    Relax(i);
  }
  g_fServosFree = true;
}



//===================================================================================================
// EmergencyStop
//===================================================================================================
void EmergencyStop(void) {
  g_fArmActive = false;

  // And Relax all of the servos...
  for(uint8_t i=1; i <= CNT_SERVOS; i++) {
    Relax(i);
  }
  g_fServosFree = true;
}





















//=============================================================================
//=============================================================================
#endif
