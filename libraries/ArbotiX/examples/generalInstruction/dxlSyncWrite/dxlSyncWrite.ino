/***********************************************************************************
 *   ___________          DYNAMIXEL Sync Write
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
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL blinks once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    This Sketch will display data to the Serial() port. Open your Serial Monitor
 *    and set your baud rate to 9600
 *    
 * Compatible Servos:
 *    MX-28T / MX-28AT (previously known as RX-28M)
 *    MX-64T / MX-64AT
 *    MX-106T
 *
 *
 ********************************************************************************/
#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int SERVO_NUMBER = 3;    //number of servos we want to write to
int servoData[SERVO_NUMBER][2] ={
                              {1,0},
                              {2,512},
                              {3,1023},
                            } ; //multi dimensional aray that holds the data to send to Sync Write. Each row represensts one servo, {servoID, data}

void setup()
{
  dxlInit(1000000);    //start dynamixel library at 1mbps to communicate with the servos
  dxlSyncWrite(servoData, SERVO_NUMBER, AX_GOAL_POSITION_L, 2); //syn write servoData array to   AX_GOAL_POSITION_L

}
void loop()
{
   
}



