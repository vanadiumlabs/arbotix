/***********************************************************************************
 *   ___________         DYNAMIXEL Ping
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
 *    dxlPing(id) checks for the prescense of a dynamixel servo at ID # id.
 *    The function returns a '1' if a servo is present and a '-1' if not.
 *    
 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL;s LED flashes once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    
 *    
 *        
 * Compatible Servos:
 *    AX-12A / AX-12+
 *    AX-18A / AX-18F
 *    MX-28T / MX-28AT (previously known as RX-28M)
 *    MX-64T / MX-64AT
 *    MX-106T
 *
 *  Other Servos:   
 *    RX, EX and MX-R servos are code compatible but need additional circuitry to function
 *
 ********************************************************************************/

#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int SERVO_ID = 1;

void setup()
{
  dxlInit(1000000);  //start dynamixel library at 1mbps to communicate with the servos
  pinMode(USER_LED, OUTPUT);  //set user LED to output
  Serial.begin(9600);

  if(dxlPing(SERVO_ID) == 1)
  {
    Serial.print("Servo ");
    Serial.print(SERVO_ID);
    Serial.println(" found!");
  }
  else
  {
    Serial.print("Servo ");
    Serial.print(SERVO_ID);
    Serial.println(" NOT found!");
    
  }
  

  
}
void loop()
{
   
}



