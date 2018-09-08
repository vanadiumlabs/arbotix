/***********************************************************************************
 *   ___________          DYNAMIXEL Generic Register Write
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
 *    This is an example of using dxlSetRegister() and dxlSetRegister2() to directlty 
 *    write to a DYNAMIXEL register. There are macros for each register available in the
 *    ArbotiX DYNAMIXEL library, though using dxlSetRegister()/dxlSetRegister2() 
 *    can be useful for lower level operations and for  array driven programming.
 *
 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL blinks once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
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

void setup()
{
  dxlInit(1000000);    //start dynamixel library at 1mbps to communicate with the servos
  

  
  dxlSetRegister(SERVO_ID,AX_LED, 1 );  //set a single byte register, turn the LED on
  //dxlGetRegister(SERVO_ID,25, 1 );  //equivalent function

  delay(33);  //for best results, do not exceed 30hz update rate, or wait ~33ms before making another dxl request

  dxlSetRegister2(SERVO_ID,AX_GOAL_POSITION_L,512); //set a 2-byte register, move the servo to 512 (centered for AX servos, 1/8 turn for MX servos)
  //dxlSetRegister2(SERVO_ID,30),512; //equivalent function


}
void loop()
{
   
}



