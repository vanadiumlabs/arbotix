/***********************************************************************************
 *   ___________          MX Bulk Read
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
 *    MX servos can respond to a bulk read command. This command will send out a single request packet, then wait for
 *    individual responses from each servo. This is more effecient than making multiple register read commands. 
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

const int MX_SERVOS_TO_READ = 3;    //number of MX servos we want to read from
int returnData[MX_SERVOS_TO_READ]; //array to hold return data
int requestParameters[MX_SERVOS_TO_READ][3] ={
                              {2,1,AX_PRESENT_POSITION_L},
                              {2,2,AX_PRESENT_SPEED_L},
                              {2,3,AX_PRESENT_LOAD_L},
                            } ; //multi dimensional aray that holds the request data. Each row represensts one request, {register length, servo ID, register to read
//{2,1,AX_PRESENT_POSITION_L} // servo ID 1, present position, 2 registers long
//{2,2,AX_PRESENT_SPEED_L}// servo ID 2, present speed, 2 registers long
//{2,3,AX_PRESENT_LOAD_L}// servo ID 3, present load, 2 registers long

void setup()
{
  dxlInit(1000000);    //start dynamixel library at 1mbps to communicate with the servos
  Serial.begin(9600); //start serial at 9600bps for reporting data
  mxBulkRead(requestParameters, 3, returnData); 
  
  for (int i = 0; i < MX_SERVOS_TO_READ; i++)
  {
    Serial.print("Servo ID ");
    Serial.print(requestParameters[i][1]);
    Serial.print("Returned Data:");
    Serial.println(returnData[i]);
  }
}
void loop()
{
   
}



