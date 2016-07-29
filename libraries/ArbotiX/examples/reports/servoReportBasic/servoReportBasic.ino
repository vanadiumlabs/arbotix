/***********************************************************************************
 *   ___________          ArbotiX Servo Report - Number of servos.
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
 *    This code will scan the DYNAMIXEL chain for servos and then issue a report on the
 *    Serial port. You will give the function a static number and it will scan for servos
 *    from ID 1 to ID n, where n is the number of servos you specified. 
 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL blinks once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    This Sketch will display data to the Serial() port. Open your Serial Monitor
 *    and set your baud rate to 9600
 *    
 *  variables:
 *    NUMBER_OF_SERVOS : Change this to match the number of servos on your chain. The
 *    scan will look for servos from 1 to NUMBER_OF_SERVOS
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

const int NUMBER_OF_SERVOS = 6;

void setup()
{
  dxlInit(1000000);  //start dynamixel library at 1mbps to communicate with the servos
  Serial.begin(9600); //start serial at 9600bps  
  dxlServoReport(NUMBER_OF_SERVOS); //scan for NUMBER_OF_SERVOS 
}
void loop()
{
   
}



