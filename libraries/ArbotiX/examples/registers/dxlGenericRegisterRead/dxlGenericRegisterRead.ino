/***********************************************************************************
 *   ___________          DYNAMIXEL Generic Register Read
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
 *    This is an example of using dxlGetRegister() to directlty write to a DYNAMIXEL   
 *    register. There are macros for each register available in the ArbotiX DYNAMIXEL
 *    library, though using dxlGetRegister() can be useful for lower level operations
 *    and for  array driven programming.
 *
 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL blinks once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    This Sketch will display data to the Serial() port. 
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
 ********************************************************************************/
#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int SERVO_ID = 1;    //number of servos we want to write to

int registerSingle; //holds a 1-byte register
int registerDouble; //holds a 2-byte register

void setup()
{
  dxlInit(1000000);    //start dynamixel library at 1mbps to communicate with the servos
  Serial.begin(9600);
  
  registerSingle = dxlGetRegister(SERVO_ID,AX_ID, 1 );
  //registerSingle = dxlGetRegister(SERVO_ID,3, 1 );  //equivalent function

  delay(33);  //for best results, do not exceed 30hz update rate, or wait ~33ms before making another dxl request

  registerDouble = dxlGetRegister(SERVO_ID,AX_PRESENT_POSITION_L,2);
  //registerDouble = dxlGetRegister(SERVO_ID,36,2); //equivalent function

  Serial.println("dxlGetRegister() read");
  Serial.print("Single byte register (ID) :");
  Serial.println(registerSingle);
  Serial.print("2 byte register (Present Position): ");
  Serial.println(registerDouble);
  

}
void loop()
{
   
}



