/***********************************************************************************
 *   ___________          ArbotiX Servo Report - Servo List.
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
 *    Serial port. You will give the function an array of servo IDs, each which will be scanned. 
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
 *    servoList: integer array with servos to scan
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
const int NUMBER_OF_SERVOS = 4;
int servoList[NUMBER_OF_SERVOS] = {1,3,5,6}; //a custom list of servos

void setup()
{
  dxlInit(1000000);  //start dynamixel library at 1mbps to communicate with the servos
  Serial.begin(9600); //start serial at 9600bps  
  dxlServoReport(NUMBER_OF_SERVOS, servoList); //scan for NUMBER_OF_SERVOS 
}
void loop()
{
   
}



