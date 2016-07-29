/***************************
 * DYNAMIXEL Analog Knob
 * Read Analog Sensor pin 0 and write 
 * that value to servo 1
 ***************************/
#include <ax12.h> //import ax12 library to send DYNAMIXEL commands

const int ANALOG_PIN = 0;
const int SERVO_ID = 1;

int rawAnalog;
int servoPosition; // position from control servo


void setup()
{
  dxlInit(1000000);

}

void loop()
{
    rawAnalog = analogRead(ANALOG_PIN); //analog read is 10-bit and returns a 0-1023 value
    servoPosition = map(rawAnalog, 0, 1023, 0, 4095); //map the sensor value to a 0-4095 range for the MX servo
    dxlSetGoalPosition(OUTPUT_SERVO,servoPosition); //set the position of servo # 1 to '0'
    delay(20);//wait for servo to move
 
}




