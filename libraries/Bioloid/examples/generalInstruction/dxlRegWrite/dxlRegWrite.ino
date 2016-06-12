/***********************************************************************************
 *   ___________         DYNAMIXEL Reg Write
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
 *    The DYNAMIXEL Reg Write command sends a command to the DYNAMIXEL, but the command
 *    will not be executed immediately. Instead the command is buffered in the servo.
 *    A follow up 'Action' command can be later sent, and then the buffered register  
 *    write will be executed.
 *    Because the 'Action' command can be sent as a broadcast command to all servos, you can 
 *    update multiple servos simultaneously by pre-seting the commands with dxlRegWrite and
 *    then send the action command.
 *    
 *    dxlRegWrite should be used for 1-byte register commands (like LED)
 *    dxlRegWrite2 should be used for 2-byte register commands (like position)
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

const int NEW_ID = 3;

void setup()
{
  dxlInit(1000000);  //start dynamixel library at 1mbps to communicate with the servos
  pinMode(USER_LED, OUTPUT);  //set user LED to output

  //regWrite commands wil be cached in each servo, and not executed until an 'dxlAction()' command is sent.
  dxlRegWrite2(1, AX_GOAL_POSITION_L, 512);  //reg write ID 1 to position 512
  dxlRegWrite2(2, AX_GOAL_POSITION_L, 512);  //reg write ID 2 to position 512
  dxlRegWrite2(3, AX_GOAL_POSITION_L, 512);  //reg write ID 3 to position 512
  dxlRegWrite(4, AX_LED, 0);                 //reg write ID 4 to be on

  delay(2000);  //wait to show the register write
  dxlAction();  //send the action command so that all servos write simultaneously 
  digitalWrite(USER_LED, HIGH); //turn LED on to show the the action command was sent
  
}
void loop()
{
   
}



