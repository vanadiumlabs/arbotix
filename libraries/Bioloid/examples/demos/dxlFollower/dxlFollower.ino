/***************************
 * DYNAMIXEL Follower
 * Read servo 1 and write 
 * that value to servo 2
 * control servo and output servo 
 * should be the same resolution
 ***************************/
#include <ax12.h> //import ax12 library to send DYNAMIXEL commands

const int CONTROL_SERVO = 1;
const int OUTPUT_SERVO = 2;

int servoPosition; // position from control servo

void setup()
{
  dxlInit(1000000);

}

void loop()
{
    servoPosition = dxlGetPosition(CONTROL_SERVO);
    dxlSetGoalPosition(OUTPUT_SERVO,servoPosition); //set the position of servo # 1 to '0'
    delay(10);//wait for servo to move
 
}




