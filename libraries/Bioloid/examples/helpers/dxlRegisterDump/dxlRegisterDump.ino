/***********************************************************************************
 *   ___________          AX Register Dump
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
 *    T

 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL blinks once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    This Sketch will display data to the Serial() port. Open your Serial Monitor
 *    and set your baud rate to 38400
 *    
 * Compatible Servos:
 *    AX-12A / AX-12+
 *    AX-18A / AX-18F
 *
 *
 ********************************************************************************/
#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int NUMBER_OF_SERVOS = 6;

void setup()
{
  dxlInit(1000000);    //start dynamixel library at 1mbps to communicate with the servos
  Serial.begin(9600); //start serial at 38400bps for reporting data
  dxlRegisterReportMultiple(NUMBER_OF_SERVOS); 
}
void loop()
{
   
}



