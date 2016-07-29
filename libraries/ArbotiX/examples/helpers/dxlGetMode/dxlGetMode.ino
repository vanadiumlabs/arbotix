/***********************************************************************************
 *   ___________          DYNAMIXEL Get Mode
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
 *    Checks the current mode of the servo

 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL;s LED flashes once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    
 * Compatible Servos:
 *    AX-12A / AX-12+
 *    AX-18A / AX-18F
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
  Serial.begin(9600);
  Serial.println("DYNAMIXEL Mode");
  
  int dxlMode = dxlGetMode(SERVO_ID); //get the mode for servo # SERVO_ID

  
  Serial.print("Servo ");
  Serial.print(SERVO_ID);
  Serial.print(" mode: ");
  
switch (dxlMode) //check which mode the servo is based on the return value from dxlGetMode()
  {
    case JOINT_MODE:
      Serial.println("Joint Mode");
    break;
    
    case WHEEL_MODE:
      Serial.println("Wheel Mode");
    break;
    
    case MULTI_TURN_MODE:
      Serial.println("Mutli-Turn Mode");
    break;
    
    case TORQUE_MODE:
      Serial.println("Torque Control Mode");
    break;
    
  }
  
}
void loop()
{
  //do nothing  
}



