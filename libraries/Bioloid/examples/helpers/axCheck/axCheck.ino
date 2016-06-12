/***********************************************************************************
 *   ___________          AX Check
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

const int SERVO_ID = 1;

void setup()
{
  dxlInit(1000000);    //start dynamixel library at 1mbps to communicate with the servos
  Serial.begin(38400); //start serial at 38400bps for reporting data
  
  int axCheck = dxlIsAX(SERVO_ID); 

  if(axCheck == 1)
  {
    Serial.println("This Servo is an AX servo");
  }
  else if(axCheck == 0)
  {
    Serial.println("This Servo is not an AX servo");
  }
  else if (axCheck == -1)
  {
    Serial.println("There was a communication error");
  }
  
}
void loop()
{
   
}



