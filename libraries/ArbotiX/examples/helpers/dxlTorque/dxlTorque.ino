/***********************************************************************************
 *   ___________          DYNAMIXEL LED Helper
 *  *|  /    \  |*
 *  *| |      | |*
 *  *|  \ __ /  |*
 *  *|          |*
 *  *|  AX-12A  |*
 *  *|          |*
 *  *|__________|*
 *
 *
 *  Code Functionality:    
 *     
 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL blinks once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    This Sketch will display data to the Serial() port. 
 *    
 *
 * Compatible Servos:
 *    AX-12A / AX-12+
 *    AX-18A / AX-18F
 *    AX-12W
 *    MX-12W
 *    MX-28T / MX-28AT (previously known as RX-28M)
 *    MX-64T / MX-64AT
 *    MX-106T
 *
 *  Other Servos:    *  
 *    RX, EX and MX-R servos are code compatible but need additional circuitry to function
 *
 ********************************************************************************/

#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int SERVO_ID = 1; //number of servos in the chain
const int BLINK_TIME = 1000; //time foe each LED to be on 


void setup()
{
  dxlInit(1000000);  //start dynamixel library at 1mbps to communicate with the servos

  dxlSetGoalSpeed(SERVO_ID, 0); //set servo speed to full
  delay(33);
  dxlSetGoalPosition(SERVO_ID, 0); //move servo to 0
  delay(2000);  //wait to make sure servo has moved
  dxlSetGoalSpeed(SERVO_ID, 50); //set servo speed slow so we can see movement
  delay(33);
  dxlSetGoalPosition(SERVO_ID, 1023); //move servo to 1023
  delay(33);
  delay(1000);

  //torque is off, servo will stop 
  dxlTorqueOff(SERVO_ID);
  //dxlTorqueOffAll(); //use this to turn off torque to all servos
  //dxlTorqueOn(SERVO_ID);//turn servo back on
  //dxlTorqueOnAll();//turn torque on for all servos
}
void loop()
{ 
  //do nothing 
   
}



