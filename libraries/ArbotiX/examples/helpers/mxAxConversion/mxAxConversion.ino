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
 *   

 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL;s LED flashes once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    
 *  variables:
 *    NEW_ID - set this to the ID that you want the servos to change to
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
  Serial.println("DYNAMIXEL AX/MX position conversion");
  
  int dxlMode = dxlGetMode(SERVO_ID); //broadcast message to set servo ID

  int axPos = 512;
  int mxPos = 2048;

  Serial.print("Original AX Position:");
  Serial.print(axPos);
  Serial.print(" New MX Position:");
  Serial.println(axToMxPosition(axPos));

  Serial.print("Original MX Position:");
  Serial.print(mxPos);
  Serial.print(" New MX Position:");  
  Serial.print(mxToAxPosition(mxPos));

  
}
void loop()
{
   
}



