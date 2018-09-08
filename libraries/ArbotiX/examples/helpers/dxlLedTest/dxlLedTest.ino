/***********************************************************************************
 *   ___________          DYNAMIXEL LED Test
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
 *    This sketch will turn the LEDs on/off for each servo servos from 1 to NUMBER_OF_SERVOS  *    
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
 *    LED_ON_TIME : time for each servo to be on for.
 *
 * Compatible Servos:
 *    AX-12A / AX-12+
 *    AX-18A / AX-18F
 *    AX-12W
 *    MX-12W
 *    MX-28T / MX-28AT (previously known as RX-28M)
 *    MX-64T / MX-64AT
 *    MX-106T
 *
 *  Other Servos:    *  
 *    RX, EX and MX-R servos are code compatible but need additional circuitry to function
 *
 ********************************************************************************/

#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int NUMBER_OF_SERVOS = 3; //number of servos in the chain
const int LED_ON_TIME = 5000; //time foe each LED to be on 


void setup()
{
  dxlInit(1000000);  //start dynamixel library at 1mbps to communicate with the servos
  dxlLedTest(NUMBER_OF_SERVOS, LED_ON_TIME); //scan for NUMBER_OF_SERVOS 
 
}
void loop()
{
   
}



