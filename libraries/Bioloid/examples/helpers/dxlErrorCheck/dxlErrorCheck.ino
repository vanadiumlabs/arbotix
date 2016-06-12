/***********************************************************************************
 *   ___________         DYNAMIXEL Error Check
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
  Serial.begin(38400);
  Serial.println("DYNAMIXEL ERROR CHECK");
  
  int dxlError = dxlGetError(SERVO_ID);
  dxlSetGoalPosition(SERVO_ID, 1023);
  dxlSetGoalPosition(SERVO_ID, 5000);
  delay(5000);
  if(dxlError == -1)
  {
    Serial.print("Servo ");
    Serial.print(SERVO_ID);
    Serial.println(" NOT found!");
    
  }
  else if(dxlError == 0)
  {
    Serial.print("Servo ");
    Serial.print(SERVO_ID);
    Serial.println(" has no errors");
  }
  else if(dxlError > 0)
  {
    Serial.print("Servo ");
    Serial.print(SERVO_ID);
      if(ERR_VOLTAGE & dxlError)
      {
          Serial.println("          Voltage Error");
      }
      if(ERR_ANGLE_LIMIT & dxlError)
      {
          Serial.println("          Angle Limit Error");
      }
      if(ERR_OVERHEATING & dxlError)
      {
          Serial.println("          Overheating Error");
      }
      if(ERR_RANGE & dxlError)
      {
          Serial.println("          Range Error");
      }
      if(ERR_CHECKSUM & dxlError)
      {
          Serial.println("          Checksum Error");
      }
      if(ERR_OVERLOAD & dxlError)
      {
          Serial.println("          Overload Error");
      }
      if(ERR_INSTRUCTION & dxlError)
      {
          Serial.println("          Instruction Error");
      }
      
      
  }


  
}
void loop()
{
   
}



