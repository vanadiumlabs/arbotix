/***********************************************************************************
 *   ___________          DYNAMIXEL Set Baud Broadcase
 *  *|  /    \  |*
 *  *| |      | |*
 *  *|  \ __ /  |*
 *  *|          |*
 *  *|  MX-28T  |*
 *  *|          |*
 *  *|__________|*
 *
 *
 *  Code Functionality:    
 *    This code will send a broadcast message to change the ID of ANY DYNAMIXEL connected
 *    to the DYNAMIXEL chain on the specified baud rate. This is handy for setting the ID
 *    for a servo when you are unsure of its current ID. Keep in mind that EVERY servo
 *    attached to the chain will change its ID. In this example all servos will be set to
 *    ID #1
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

void setup()
{
  
  dxlInit(1000000);  //start dynamixel library at 1mbps incase servo was already at 1mbps but has some other issue
  delay(100);
  dxlReset(DXL_BROADCAST); //broadcast message to set servo ID reset servo
  
  dxlInit(57142);  //start dynamixel library at 57142 to communicate with the MX servos
  delay(100);
  dxlReset(DXL_BROADCAST); //broadcast message to set servo ID reset servo
  delay(5000); //reset can take a little while
  dxlSetBaud(DXL_BROADCAST, 1); //broadcast message to set servo baud
  
  pinMode(USER_LED, OUTPUT);
  digitalWrite(USER_LED, HIGH); //operation is over, turn led on

  dxlInit(1000000);  //start dynamixel library at 1mbps to talk to MX servos that are now at 1mbps
}
void loop()
{
  for(int i = 0; i <= 4095; i++)
  {
    dxlSetGoalPosition(DXL_BROADCAST, i);
    delay(33);
  }
   
  for(int i = 4095; i >= 0; i--)
  {
    dxlSetGoalPosition(DXL_BROADCAST, i);
    delay(33);
  }
   
}



