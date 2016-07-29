/***********************************************************************************
 *   ___________          DYNAMIXEL MX Joint Mode
 *  *|  /    \  |*
 *  *| |      | |*
 *  *|  \ __ /  |*
 *  *|          |*
 *  *|  MX-28   |*
 *  *|          |*
 *  *|__________|*
 *
 *
 *  Code Functionality:    

 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL;s LED flashes once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    
 * Compatible Servos:
 *    MX-28T / MX-28AT (previously known as RX-28M)
 *    MX-64T / MX-64AT
 *    MX-106T
 *
 *  Other Servos:    *  
 *    RX, EX and MX-R servos are code compatible but need additional circuitry to function
 *
 ********************************************************************************/

#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int SERVO_ID = 1;

void setup()
{
  dxlInit(1000000);  //start dynamixel library at 1mbps to communicate with the servos
  
  int dxlMode = dxlGetMode(SERVO_ID); //get the mode for servo # SERVO_ID
  delay(33);
  //check if the servo's mode. Only set joint mode if the mode is not already in joint mode. This helps to preserve the lifespan of the EEPROM memory on the servo
  if(dxlMode != JOINT_MODE)
  {
    mxSetJointMode(SERVO_ID);
  }

  
}
void loop()
{
  //move servo positions up in increments of 10
  for (int i = 0; i <= MX_MAX_POSITION_VALUE; i = i + 10)
  {
    dxlSetGoalPosition(SERVO_ID, i); 
    delay(33);
  }
  
  //move servo positions down in increments of 10
  for (int i = MX_MAX_POSITION_VALUE; i >= 0; i = i - 10)
  {
    dxlSetGoalPosition(SERVO_ID, i); 
    delay(33);
  }
  //do nothing  
}



