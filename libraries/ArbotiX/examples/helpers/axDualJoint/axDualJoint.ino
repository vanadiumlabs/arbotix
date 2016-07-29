/***********************************************************************************
 *   ___________         AX servo Dual Joint
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
 *    This function uses the sync write commands to move 2 dual joint servos
 *    simultaneously, like in the PhantomX Reactor Arm
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
 *    AX-12W (Not reccomended in joint mode)
 *
 *
 ********************************************************************************/
#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int SERVO_NUMBER = 2;    //number of servos we want to write to

//in this example we're going to move servos 2/3 like in the PhantomX Reactor arm
int servoData[SERVO_NUMBER][2] ={
                              {2,0},
                              {3,1023},
                            } ; //multi dimensional aray that holds the data to send to Sync Write. Each row represensts one servo, {servoID, data}

void setup()
{
  dxlInit(1000000);    //start dynamixel library at 1mbps to communicate with the servos
  
  dxlSyncWritePosition(servoData, SERVO_NUMBER); //sync write servoData array to servos

}

void loop()
{
  for (int i = 0; i <= 1023; i++)
  {
    servoData[0][1] = i;  //set servo data to position
    servoData[1][1] = 1023 - i;    //set servo data to inverse position because the 2 servos are physically inverted from each other. If you have a custom setup with servos that have the same orientation, than '1023 -' is not needed
    dxlSyncWritePosition(servoData, SERVO_NUMBER); //sync write servoData array to servos
    delay(10); //short movement delay
  }
}



