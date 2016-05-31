#ifndef GLOBALARM_H
#define GLOBALARM_H

//=============================================================================
// RG Snapper Arm Global Constraints & Work Area Definition
//=============================================================================

#define ARMID       5


//=============================================================================
//=============================================================================
// Normal Work Area
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //

#define IK_MAX_X  150
#define IK_MIN_X  -150

#define IK_MAX_Y  200
#define IK_MIN_Y  50

#define IK_MAX_Z  225
#define IK_MIN_Z  20

#define IK_MAX_GA  30
#define IK_MIN_GA  -30

// Arm dimensions(mm)
#define BASE_HGT 88.75      //base height (mm)
#define HUMERUS 96.00      //shoulder-to-elbow "humerus bone" (mm)
#define ULNA 90.6        //elbow-to-wrist "ulna bone" (mm)
#define GRIPPER 96      //wrist-gripper length (mm)



// offsets 
#define GA_OFFSET  90 //subtract this from GA to give us -90 - +90 angle
#define X_OFFSET  512 //offset value for 3D Cart mode on X axis

//////////////////////////////////////////////////////////////////////////////
// SERVO CONFIG  //
//////////////////////////////////////////////////////////////////////////////
// Declare servos
enum {
  BAS_SERVO=0, SHL_SERVO, ELB_SERVO, WRI_SERVO, GRI_SERVO};
//BAS_SERVO  - Base RobotGeek Servo (600-2400)
//SHL_SERVO  - Shoulder RobotGeek Servo (600-2400)
//ELB_SERVO  - Elbow RobotGeek Servo (600-2400)
//WRI_SERVO  - Wrist tilt RobotGeek Servo (600-2400)
//GRI_SERVO  - Grip 9G RobotGeek Servo

// Servo position limitations - limits in microseconds
#define BASE_MIN      600     //full counterclockwise for RobotGeek 180 degree servo
#define BASE_MAX      2400    //full clockwise for RobotGeek 180 degree servo
#define SHOULDER_MIN  600
#define SHOULDER_MAX  2400
#define ELBOW_MIN     600
#define ELBOW_MAX     2400
#define WRIST_MIN     600
#define WRIST_MAX     2400 
//mins and maxes depending on gripper type
#if GRIPPER_TYPE == ROBOT_GEEK_9G_GRIPPER
  #define GRIPPER_MIN   900    //full counterclockwise for 9g servo
  #define GRIPPER_MAX   2100   //full clockwise for 9g servo
#elif GRIPPER_TYPE == ROBOT_GEEK_PARALLEL_GRIPPER
  #define GRIPPER_MIN   750    //fully closed
  #define GRIPPER_MAX   2400   //fully open
#endif




// Define servo offsets in +/- uS. Adjust if your arm is not centering properly.
#define BAS_SERVO_ERROR 0 //(+ is CW, - is CCW)
#define SHL_SERVO_ERROR 0 //(+ is forward, - is backward)
#define ELB_SERVO_ERROR 0 //(+ is up, - is down)
#define WRI_SERVO_ERROR 0 //(+ is up, - is down)
#define GRI_SERVO_ERROR 0 //(+ is tighten grip, - is loosen grip) 

 //present positions of the servos 
float Base     =1500;    //holds the present position of the Base servo, starts at 1500 (centered)
float Shoulder =1500;    //holds the present position of the Shoulder servo, starts at 1500 (centered)
float Elbow    =1500;    //holds the present position of the Elbow servo, starts at 1500 (centered)
float Wrist    =1500;    //holds the present position of the wrist servo, starts at 1500 (centered)
int   Gripper  =1500;    //holds the present position of the gripper servo, starts at 1500 (centered)

 //present positions of the servos 
//float sBase     =1500;    //holds the present position of the Base servo, starts at 1500 (centered)

unsigned int sDeltaTime = 3000;

//=============================================================================
//=============================================================================






























//=============================================================================
//=============================================================================
#endif
