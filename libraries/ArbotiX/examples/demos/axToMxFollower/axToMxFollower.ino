/***************************
 * DYNAMIXEL Follower
 * Read servo 1 and write 
 * that value to servo 2
 * control servo is AX and 
 * output servo is MX
 * 
 * WARNING!!!!
 * The AX servo only has a 300 degree encoder.
 * While in the 60 degree deadzone the servo may report incorrect data, cand move the MX
 * servo incorrectly. DO NOT USE THIS CODE if the ax servo can be in the 60 degree deadzone. 
 ***************************/
#include <ax12.h> //import ax12 library to send DYNAMIXEL commands

const int CONTROL_SERVO = 1;
const int OUTPUT_SERVO = 2;

int servoPosition; // position from control servo

void setup()
{
  dxlInit(1000000);
  dxlTorqueOff(1); //make sure control servo torque is off.
}

void loop()
{
    servoPosition = dxlGetPosition(CONTROL_SERVO);//get AX servo position
    if(servoPosition > -1)//make sure we received valid data back from the servo
    {
      servoPosition = axToMxPosition(servoPosition); //convert AX data to MX data  
      dxlSetGoalPosition(OUTPUT_SERVO,servoPosition); //set the position of servo # 1 to '0'   
    }
    
    delay(20);//wait for servo to move   
 
}




