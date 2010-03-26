#include <ax12.h>
#include <BioloidController.h>
#include <WProgram.h>
#include <math.h>
#include "nuke.h"

/* min and max positions for each servo */                                          
@SERVO_MINS
@SERVO_MAXS

/* IK Engine */
BioloidController bioloid = BioloidController(1000000);
ik_req_t endpoints[LEG_COUNT];
float bodyRotX = 0;             // body roll (rad)
float bodyRotY = 0;             // body pitch (rad)
float bodyRotZ = 0;             // body rotation (rad)
int bodyPosX = 0;               // body offset (mm)
int bodyPosY = 0;               // body offset (mm)
int Xspeed;                     // forward speed (mm/s)
int Yspeed;                     // sideward speed (mm/s)
float Rspeed;                   // rotation speed (rad/s)

/* Gait Engine */
int gaitLegNo[LEG_COUNT];       // order to step through legs
ik_req_t gaits[LEG_COUNT];      // gait engine output
int pushSteps;                  // how much of the cycle we are on the ground
int stepsInCycle;               // how many steps in this cycle
int step;                       // current step 
int tranTime;
int liftHeight;
float cycleTime;                // cycle time in seconds (adjustment from speed to step-size)

/* Setup the starting positions of the legs. */
void setupIK(){
  endpoints[RIGHT_FRONT].x = @X_STANCE;
  endpoints[RIGHT_FRONT].y = @Y_STANCE;
  endpoints[RIGHT_FRONT].z = @Z_STANCE;
                    
  endpoints[RIGHT_REAR].x = -@X_STANCE;
  endpoints[RIGHT_REAR].y = @Y_STANCE;
  endpoints[RIGHT_REAR].z = @Z_STANCE;
@IF legs 6

  endpoints[RIGHT_MIDDLE].x = 0;
  endpoints[RIGHT_MIDDLE].y = @Y_STANCE;
  endpoints[RIGHT_MIDDLE].z = @Z_STANCE;

  endpoints[LEFT_MIDDLE].x = 0;
  endpoints[LEFT_MIDDLE].y = -@Y_STANCE;
  endpoints[LEFT_MIDDLE].z = @Z_STANCE;  
@END

  endpoints[LEFT_FRONT].x = @X_STANCE;
  endpoints[LEFT_FRONT].y = -@Y_STANCE;
  endpoints[LEFT_FRONT].z = @Z_STANCE;
    
  endpoints[LEFT_REAR].x = -@X_STANCE;
  endpoints[LEFT_REAR].y = -@Y_STANCE;
  endpoints[LEFT_REAR].z = @Z_STANCE;

  liftHeight = @LIFT_HEIGHT;
  stepsInCycle = 1;
  step = 0;
}

#include "gaits.h"

/* Convert radians to servo position offset. */
int radToServo(float rads){ 
  float val = (rads*100)/51 * 100;
  return (int) val; 
}

/* Body IK solver: compute where legs should be. */
ik_req_t bodyIK(int X, int Y, int Z, int Xdisp, int Ydisp, float Zrot){
    ik_req_t ans;
    
    float cosB = cos(bodyRotX);
    float sinB = sin(bodyRotX);
    float cosG = cos(bodyRotY);
    float sinG = sin(bodyRotY);
    float cosA = cos(bodyRotZ+Zrot);
    float sinA = sin(bodyRotZ+Zrot);
    
    int totalX = X + Xdisp + bodyPosX; 
    int totalY = Y + Ydisp + bodyPosY; 
    
    ans.x = totalX - int(totalX*cosG*cosA + totalY*sinB*sinG*cosA + Z*cosB*sinG*cosA - totalY*cosB*sinA + Z*sinB*sinA) + bodyPosX;
    ans.y = totalY - int(totalX*cosG*sinA + totalY*sinB*sinG*sinA + Z*cosB*sinG*sinA + totalY*cosB*cosA - Z*sinB*cosA) + bodyPosY;
    ans.z = Z - int(-totalX*sinG + totalY*sinB*cosG + Z*cosB*cosG);
    
    return ans;
}

/* Simple 3dof leg solver. X,Y,Z are the length from the Coxa rotate to the endpoint. */
ik_sol_t legIK(int X, int Y, int Z){
@LEG_IK
}

void doIK(){
    int servo;
    ik_req_t req, gait;    
    ik_sol_t sol;
    
    gaitSetup();

    // right front leg
    gait = gaitGen(RIGHT_FRONT);    
    req = bodyIK(endpoints[RIGHT_FRONT].x+gait.x, endpoints[RIGHT_FRONT].y+gait.y, endpoints[RIGHT_FRONT].z+gait.z, X_COXA, Y_COXA, gait.r);
    sol = legIK(endpoints[RIGHT_FRONT].x+req.x+gait.x,endpoints[RIGHT_FRONT].y+req.y+gait.y,endpoints[RIGHT_FRONT].z+req.z+gait.z);
    servo = @NEUTRAL_RF_COXA @SIGN_RF_COXA sol.coxa;
    if(servo < maxs[RF_COXA-1] && servo > mins[RF_COXA-1])
        bioloid.setNextPose(RF_COXA, servo);
    else{
        Serial.print("RF_COXA FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_RF_FEMUR @SIGN_RF_FEMUR sol.femur;
    if(servo < maxs[RF_FEMUR-1] && servo > mins[RF_FEMUR-1])
        bioloid.setNextPose(RF_FEMUR, servo);
    else{
        Serial.print("RF_FEMUR FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_RF_TIBIA @SIGN_RF_TIBIA sol.tibia;
    if(servo < maxs[RF_TIBIA-1] && servo > mins[RF_TIBIA-1])
        bioloid.setNextPose(RF_TIBIA, servo);
    else{
        Serial.print("RF_TIBIA FAIL: ");
        Serial.println(servo);
    }
    
    // right rear leg
    gait = gaitGen(RIGHT_REAR);    
    req = bodyIK(endpoints[RIGHT_REAR].x+gait.x,endpoints[RIGHT_REAR].y+gait.y, endpoints[RIGHT_REAR].z+gait.z, -X_COXA, Y_COXA, gait.r);
    sol = legIK(-endpoints[RIGHT_REAR].x-req.x-gait.x,endpoints[RIGHT_REAR].y+req.y+gait.y,endpoints[RIGHT_REAR].z+req.z+gait.z);
    servo = @NEUTRAL_RR_COXA @SIGN_RR_COXA sol.coxa;
    if(servo < maxs[RR_COXA-1] && servo > mins[RR_COXA-1])
        bioloid.setNextPose(RR_COXA, servo);
    else{
        Serial.print("RR_COXA FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_RR_FEMUR @SIGN_RR_FEMUR sol.femur;
    if(servo < maxs[RR_FEMUR-1] && servo > mins[RR_FEMUR-1])
        bioloid.setNextPose(RR_FEMUR, servo);
    else{
        Serial.print("RR_FEMUR FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_RR_TIBIA @SIGN_RR_TIBIA sol.tibia;
    if(servo < maxs[RR_TIBIA-1] && servo > mins[RR_TIBIA-1])
        bioloid.setNextPose(RR_TIBIA, servo);
    else{
        Serial.print("RR_TIBIA FAIL: ");
        Serial.println(servo);
    }
    
    // left front leg
    gait = gaitGen(LEFT_FRONT);
    req = bodyIK(endpoints[LEFT_FRONT].x+gait.x,endpoints[LEFT_FRONT].y+gait.y, endpoints[LEFT_FRONT].z+gait.z, X_COXA, -Y_COXA, gait.r);
    sol = legIK(endpoints[LEFT_FRONT].x+req.x+gait.x,-endpoints[LEFT_FRONT].y-req.y-gait.y,endpoints[LEFT_FRONT].z+req.z+gait.z);
    servo = @NEUTRAL_LF_COXA @SIGN_LF_COXA sol.coxa;
    if(servo < maxs[LF_COXA-1] && servo > mins[LF_COXA-1])
        bioloid.setNextPose(LF_COXA, servo);
    else{
        Serial.print("LF_COXA FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_LF_FEMUR @SIGN_LF_FEMUR sol.femur;
    if(servo < maxs[LF_FEMUR-1] && servo > mins[LF_FEMUR-1])
        bioloid.setNextPose(LF_FEMUR, servo);
    else{
        Serial.print("LF_FEMUR FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_LF_TIBIA @SIGN_LF_TIBIA sol.tibia;
    if(servo < maxs[LF_TIBIA-1] && servo > mins[LF_TIBIA-1])
        bioloid.setNextPose(LF_TIBIA, servo);
    else{
        Serial.print("LF_TIBIA FAIL: ");
        Serial.println(servo);
    }

    // left rear leg
    gait = gaitGen(LEFT_REAR);
    req = bodyIK(endpoints[LEFT_REAR].x+gait.x,endpoints[LEFT_REAR].y+gait.y, endpoints[LEFT_REAR].z+gait.z, -X_COXA, -Y_COXA, gait.r);
    sol = legIK(-endpoints[LEFT_REAR].x-req.x-gait.x,-endpoints[LEFT_REAR].y-req.y-gait.y,endpoints[LEFT_REAR].z+req.z+gait.z);
    servo = @NEUTRAL_LR_COXA @SIGN_LR_COXA sol.coxa;
    if(servo < maxs[LR_COXA-1] && servo > mins[LR_COXA-1])
        bioloid.setNextPose(LR_COXA, servo);
    else{
        Serial.print("LR_COXA FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_LR_FEMUR @SIGN_LR_FEMUR sol.femur;
    if(servo < maxs[LR_FEMUR-1] && servo > mins[LR_FEMUR-1])
        bioloid.setNextPose(LR_FEMUR, servo);
    else{
        Serial.print("LR_FEMUR FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_LR_TIBIA @SIGN_LR_TIBIA sol.tibia;
    if(servo < maxs[LR_TIBIA-1] && servo > mins[LR_TIBIA-1])
        bioloid.setNextPose(LR_TIBIA, servo);
    else{
        Serial.print("LR_TIBIA FAIL: ");
        Serial.println(servo);
    }
    
@IF legs 6
    // right middle leg
    gait = gaitGen(RIGHT_MIDDLE);    
    req = bodyIK(endpoints[RIGHT_MIDDLE].x+gait.x,endpoints[RIGHT_MIDDLE].y+gait.y, endpoints[RIGHT_MIDDLE].z+gait.z, 0, Y_COXA, gait.r);
    sol = legIK(endpoints[RIGHT_MIDDLE].x+req.x+gait.x,endpoints[RIGHT_MIDDLE].y+req.y+gait.y,endpoints[RIGHT_MIDDLE].z+req.z+gait.z);
    servo = @NEUTRAL_RM_COXA @SIGN_RM_COXA sol.coxa;
    if(servo < maxs[RM_COXA-1] && servo > mins[RM_COXA-1])
        bioloid.setNextPose(RM_COXA, servo);
    else{
        Serial.print("RM_COXA FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_RM_FEMUR @SIGN_RM_FEMUR sol.femur;
    if(servo < maxs[RM_FEMUR-1] && servo > mins[RM_FEMUR-1])
        bioloid.setNextPose(RM_FEMUR, servo);
    else{
        Serial.print("RM_FEMUR FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_RM_TIBIA @SIGN_RM_TIBIA sol.tibia;
    if(servo < maxs[RM_TIBIA-1] && servo > mins[RM_TIBIA-1])
        bioloid.setNextPose(RM_TIBIA, servo);
    else{
        Serial.print("RM_TIBIA FAIL: ");
        Serial.println(servo);
    }

    // left middle leg
    gait = gaitGen(LEFT_MIDDLE);
    req = bodyIK(endpoints[LEFT_MIDDLE].x+gait.x,endpoints[LEFT_MIDDLE].y+gait.y, endpoints[LEFT_MIDDLE].z+gait.z, 0, -Y_COXA, gait.r);
    sol = legIK(endpoints[LEFT_MIDDLE].x+req.x+gait.x,-endpoints[LEFT_MIDDLE].y-req.y-gait.y,endpoints[LEFT_MIDDLE].z+req.z+gait.z);
    servo = @NEUTRAL_LM_COXA @SIGN_LM_COXA sol.coxa;
    if(servo < maxs[LM_COXA-1] && servo > mins[LM_COXA-1])
        bioloid.setNextPose(LM_COXA, servo);
    else{
        Serial.print("LM_COXA FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_LM_FEMUR @SIGN_LM_FEMUR sol.femur;
    if(servo < maxs[LM_FEMUR-1] && servo > mins[LM_FEMUR-1])
        bioloid.setNextPose(LM_FEMUR, servo);
    else{
        Serial.print("LM_FEMUR FAIL: ");
        Serial.println(servo);
    }
    servo = @NEUTRAL_LM_TIBIA @SIGN_LM_TIBIA sol.tibia;
    if(servo < maxs[LM_TIBIA-1] && servo > mins[LM_TIBIA-1])
        bioloid.setNextPose(LM_TIBIA, servo);
    else{
        Serial.print("LM_TIBIA FAIL: ");
        Serial.println(servo);
    }
@END
    step = (step+1)%stepsInCycle;
}
