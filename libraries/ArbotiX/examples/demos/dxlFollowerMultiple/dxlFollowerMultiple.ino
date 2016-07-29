/***************************
 * DYNAMIXEL Follower Multiple
 * Read servo 1, 2 and 3 then
 * write that value to servo 4, 5 and 6
 * control servo and output servo 
 * should be the same resolution
 ***************************/

#include <ax12.h> //import ax12 library to send DYNAMIXEL commands

const int NUMBER_CONTROL_SERVOS = 3;
int servoPosition; // position from control servo

const int controlServoIds[] = {1, 2, 3};  //ids for control servos
int writeServoData[NUMBER_CONTROL_SERVOS][2] = {
                                                  {4, 512},
                                                  {5, 512},
                                                  {6, 512}
                                                }; //id/position pairs for output servos

void setup()
{
  dxlInit(1000000);
  
  for(int i = 0; i < NUMBER_CONTROL_SERVOS; i++)
  {
    dxlTorqueOff(controlServoIds[i]); //make sure control servo torque is off.
  }

}

void loop()
{

  for(int i = 0; i < NUMBER_CONTROL_SERVOS; i++)
  {
    int tempPos = dxlGetPosition(controlServoIds[i]);
    if(tempPos > -1)
    {
      writeServoData[i][1] = dxlGetPosition(controlServoIds[i]);
    }
  }

  dxlSyncWritePosition(writeServoData,NUMBER_CONTROL_SERVOS);
  delay(20);//wait for servo to move
 
}




