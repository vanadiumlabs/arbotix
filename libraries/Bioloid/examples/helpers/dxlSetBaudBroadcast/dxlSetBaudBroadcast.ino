/***********************************************************************************
 *   ___________          ArbotiX Set Baud Broadcast
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
 *    This code will send a broadcast message to change the baud of ANY DYNAMIXEL connected
 *    to the DYNAMIXEL chain on the specified baud rate.
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

const int NEW_BAUD = 1;

void setup()
{
  dxlInit(57600);  //start dynamixel library at 57600bps to communicate with the servos
  dxlSetBaud(DXL_BROADCAST, NEW_BAUD); //broadcast message to set servo ID
}
void loop()
{
   
}



