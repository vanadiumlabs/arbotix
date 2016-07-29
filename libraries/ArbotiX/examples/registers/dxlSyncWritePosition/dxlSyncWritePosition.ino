/***********************************************************************************
 *   ___________          DYNAMIXEL Sync Write Position
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
 *    Sync Write is a DYNAMIXEL instruction that will allow you to write to multiple 
 *    DYNAMIXEL registers at the same time.  dxlSyncWritePosition() allows you to
 *    set multiple positions simultaneously. This is very useful in contrained joints 
 *    like dual joints, and as a method to reduce packet overhead as all the position
 *    data is sent in one packet. *
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

const int SERVO_NUMBER = 3;    //number of servos we want to write to
int servoData[SERVO_NUMBER][2] ={
                              {1,0},
                              {2,512},
                              {3,1023},
                            } ; //multi dimensional aray that holds the data to send to Sync Write. Each row represensts one servo, {servoID, data}

void setup()
{
  dxlInit(1000000);    //start dynamixel library at 1mbps to communicate with the servos
  dxlSyncWritePosition(servoData, SERVO_NUMBER); //sync write servoData array to servos

}
void loop()
{
   
}



