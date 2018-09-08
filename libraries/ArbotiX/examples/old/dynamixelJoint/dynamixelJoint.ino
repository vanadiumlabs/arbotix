
/***********************************************************************************
 *   ___________          ArbotiX Dynamixel Joint Mode Test
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
 *    This code will set a single servo to joint mode, then slowly move the 
 *    servo from completley clockwise to compleley counterclockwise, then back
 *    to completely clockwise.
 *    
 *  Physical Build:
 *    Connect your servo to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure the DYNAMIXEL blinks once when power
 *    is applied). This sketch will move the servo from its lower to upper limtis
 *    so you need to ensure that there is nothing connected to the servo horn, or 
 *    that anything connected will not obstruct its movement.
 *
 *  Defines:
 *    The default value for HIGHEST_POSTIION is set for AX servos. Set it to 4095
 *    for MX servos. 
 *    Change SERVO_ID to control a different servo. Set the value to 254 to control
 *    all servos.
 *    Increase SERVO_DELAY  to slow down the speed at which the servo moves.
 *
 * Compatible Servos:
 *    AX-12A / AX-12+
 *    AX-18A / AX-18F
 *    MX-28T / MX-28AT (previously known as RX-28M)
 *    MX-64T / MX-64AT
 *    MX-106T
 *
 *  Other Servos:   
 *    MX-12W and AX-12W can be used with this code, but it is NOT reccomended
 *    - these servos do not perform well in joint mode
 *  
 *    RX, EX and MX-R servos are code compatible but need additional circuitry to function
 *
 *  This code does not check for servo voltage, presense, or errors. 
 *
 ********************************************************************************/

//import ax12 library to send DYNAMIXEL commands
#include <ax12.h>

/*****DEFINES*********/
#define HIGHEST_POSTIION 1023   //Set the higest position for the servo to go to. This is 1023 for AX servos and 4095 for MX servos
//#define HIGHEST_POSTIION 4095
#define SERVO_ID 1              //ID of the servo to control
#define SERVO_DELAY 10          //how long to wait after setting a position


//setup runs once
void setup()
{
    ax12Init(1000000);        //initialize the dynamixel class. 
    dxlSetJointMode;          //sets the servo to joint mode - not needed if you know the servo is in joint mode
    SetPosition(SERVO_ID,0);  //set the position of servo # 1 to '0'
    delay(100);               //wait for servo to move to its position

  //increment from 0 to HIGHEST_POSTIION
  for(int i=0; i <= HIGHEST_POSTIION; i++)
  {
    SetPosition(SERVO_ID,i);  //set the position of servo #SERVO_ID to the current value of 'i'
    delay(SERVO_DELAY);       //wait for servo to move
  }
  //decrement from HIGHEST_POSTIION to 0
  for(int i=HIGHEST_POSTIION; i >= 0 ; i--)
  {
    SetPosition(SERVO_ID,i);   //set the position of servo #SERVO_ID to the current value of 'i'
    delay(SERVO_DELAY);         //wait for servo to move
  }

}

//loop runs indefinitley
void loop()
{
  //do nothing
}



