/***********************************************************************************
 *   ___________          DYNAMIXEL System Voltage List
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
 *    This sketch will use dxlGetSystemVoltage() to scan servos in the array SERVO_LIST for their 
 *    present voltage, than average  out all of those voltages. TThen the voltage 
 *    will be returned as a float variable for use in your code. dxlGetSystemVoltage() is meant as a basic
 *    debugging tool, so it will notreturn  errors if no servos are found. If individual 
 *    servos are missing, dxlGetSystemVoltage() will only average the servos that are present. A negative value
 *    means no servos were found.
 *    
 *    This function can be useful to get the voltage from a subset of non linearly IDed servos
 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL blinks once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    This Sketch will display data to the Serial() port. Open your Serial Monitor
 *    and set your baud rate to 9600
 *    
 *  variables:
 *    NUMBER_OF_SERVOS : Change this to match the number of servos on your chain. 
 *    SERVO_LIST : array of servos to scan
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

const int NUMBER_OF_SERVOS = 3;
const int SERVO_LIST[] = {3, 4 ,5};

float systemVoltage;



void setup()
{
  dxlInit(1000000);  //start dynamixel library at 1mbps to communicate with the servos
  Serial.begin(9600); //start serial at 9600bps  
  systemVoltage = dxlGetSystemVoltage(NUMBER_OF_SERVOS,SERVO_LIST); //scan for NUMBER_OF_SERVOS 
  Serial.print("System Voltage:");
  Serial.println(systemVoltage);
}
void loop()
{
   
}



