/***************************
 * AXSimpleTest
 * This sketch sends positional commands to the AX servo 
 * attached to it - the servo must set to ID # 1
 * The sketch will send a value, i, to the servo.
 * 'For' loops are used to increment and decrement the value of 'i'
 ***************************/

//import ax12 library to send DYNAMIXEL commands
#include <ax12.h>

const int SERVO_ID = 1; //servo id to control

void setup()
{
    dxlInit(1000000); //start the DYNAMIXEL chain at 
  
    SetPosition(SERVO_ID,0); //set the position of servo # 1 to '0'
    delay(1000);//wait for servo to move
}

void loop()
{
  //increment from 0 to 1023
  for(int i=0; i <=1023; i++)
  {
    dxlSetGoalPosition(SERVO_ID,i); //set the position of servo #1 to the current value of 'i'
    delay(33);//wait for servo to move / send data at 30hz
  }
  //decrement from 1024 to 0
  for(int i=1023; i>0; i--)
  {
    dxlSetGoalPosition(SERVO_ID,i);//set the position of servo #1 to the current value of 'i'
    delay(33);//wait for servo to move / send data at 30hz
  }
}



