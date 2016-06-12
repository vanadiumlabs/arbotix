/***********************************************************************************
 *   ___________          ArbotiX Set ID Broadcast
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

const int NEW_ID = 3;

void setup()
{
  dxlInit(1000000);  //start dynamixel library at 1mbps to communicate with the servos
  dxlSetId(DXL_BROADCAST, NEW_ID); //broadcast message to set servo ID
  pinMode(USER_LED, OUTPUT);
  digitalWrite(USER_LED, HIGH);
}
void loop()
{
   
}



