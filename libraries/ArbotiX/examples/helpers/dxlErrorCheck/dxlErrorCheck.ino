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
 *    This sketch uses dxlGetError() to check if a servo has any error conditions.
 *    This sketch will take the response and print out a serial statement with the
 *    individual errors. 
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
 *    AX-12W
 *    MX-12W
 *    MX-28T / MX-28AT (previously known as RX-28M)
 *    MX-64T / MX-64AT
 *    MX-106T
 *
 *
 *  Other Servos:   
 *    RX, EX and MX-R servos are code compatible but need additional circuitry to function
 *
 ********************************************************************************/

#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int SERVO_ID = 1;

void setup()
{
  dxlInit(1000000);           //start dynamixel library at 1mbps to communicate with the servos
  pinMode(USER_LED, OUTPUT);  //set user LED to output
  Serial.begin(9600);         //open the serial port at 9600
  Serial.println("DYNAMIXEL ERROR CHECK");
  
  int dxlError = dxlGetError(SERVO_ID); //check the servo for an error
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
      //the error byte is 0 if there are no errors. If the byte is non-zero, then each bit in the byte desginates a different error.
      //by using the & bitwise operator with the appropriate error mask (like ERR_VOLTAGE) we can isolate which bits are active and print an error 
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



