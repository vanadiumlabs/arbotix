/***********************************************************************************
 *   ___________          DYNAMIXEL MX Write Registers
 *  *|  /    \  |*
 *  *| |      | |*
 *  *|  \ __ /  |*
 *  *|          |*
 *  *|  MX-28   |*
 *  *|          |*
 *  *|__________|*
 *
 *
 *  Code Functionality:    
 *    This sketch will set all possible registers for the servo. The data in this
 *    sketch is the default data for the servo. This is meant as an example to show
 *    off all of the 'set' register macros. To reset a servo, we typically recoomend
 *    'dxlReset()'
 *    After all values have been set, this sketch will use  dxlRegisterReportSingle()
 *    to show the servo data
 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL blinks once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    This Sketch will display data to the Serial() port. Open your Serial Monitor
 *    and set your baud rate to 9600
 *    
 * Compatible Servos:
 *    MX-12W
 *    MX-28T / MX-28AT (previously known as RX-28M)
 *    MX-64T / MX-64AT
 *    MX-106T
 *
 *
 *
 ********************************************************************************/
#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int SERVO_ID = 1;

int regData;

void setup()
{
  dxlInit(1000000);    //start dynamixel library at 1mbps to communicate with the servos
  Serial.begin(9600); //start serial at 9600 for reporting data. 

  //set registers
  dxlSetId(SERVO_ID,SERVO_ID);                //set ID. In this case set the ID to the current ID, this is just an example
  delay(33);  //for best results, do not exceed 30hz update rate, or wait ~33ms before making another dxl request
  dxlSetBaud(SERVO_ID, 1);                    //set Baud. In this case set the Baud to the current Baud (1 mbps), this is just an example. See Manual for a list of baud rates
  delay(33);
  dxlSetReturnDelayTime(SERVO_ID, 250);       //set return delay time in milliseconds
  delay(33);
  dxlSetCWAngleLimit(SERVO_ID, 0);            //set clockwise angle limit
  delay(33);
  dxlSetCCWAngleLimit(SERVO_ID, 4095);        //set counter clockwise angle limit
  delay(33);
  dxlSetTempLimit(SERVO_ID, 80);               //set shutdown temperature limit in degrees celcius
  delay(33);
  dxlSetLowVoltage(SERVO_ID, 60);              //set low voltage limit, value/10 = voltage
  delay(33);
  dxlSetHighVoltage(SERVO_ID, 160);            //set high voltage limit, value/10 = voltage
  delay(33);
  dxlSetStartupMaxTorque(SERVO_ID, 1023);      //set max torque as a percent of 1023. See manual for more info. This value is loaded into RAM on startup and does not directly effect torque limit
  delay(33);
  dxlSetStatusReturnLevel(SERVO_ID, 2);        //set status return level, see manual for more info
  delay(33);
  dxlSetAlarmLED(SERVO_ID, 36);                //set which errors to turn alarm LED on for, see manual for more info
  delay(33);
  dxlSetAlarmShutdown(SERVO_ID, 36);           //set which errors to shutdown servo , see manual for more info
  delay(33);
  mxSetMultiTurnOffset(SERVO_ID, 0);           //set value for mx multi turn offset, see manual for more info
  delay(33);
  mxSetResolutionDivider(SERVO_ID, 1);         //set value for mx multi turn resolution divider, see manual for more info
  delay(33);
  dxlSetTorqueEnable(SERVO_ID, 0);             //set servo torque, 0=off 1 = on 
  delay(33);
  dxlSetLED(SERVO_ID, 0);                      //set led , 0 = off, 1 = on
  delay(33);
  mxSetD(SERVO_ID, 0);                          //set Derivative gain for PID, , see manual for more info
  delay(33);
  mxSetI(SERVO_ID, 0);                          //set Integral gain for PID, , see manual for more info
  delay(33);
  mxSetP(SERVO_ID, 32);                         //set Proportional gain for PID, , see manual for more info
  delay(33);
  dxlSetGoalPosition(SERVO_ID, 2048);           //set goal position , see manual for more info           
  delay(33);
  dxlSetGoalSpeed(SERVO_ID, 0);                 //set goal speed, see manual for more info
  delay(33);
  dxlSetRunningTorqueLimit(SERVO_ID, 1023);     //set running torque limit. This value is loaded from EEPROM and governs the torque limit while running
  delay(33);
  dxlSetEEPROMLock(SERVO_ID, 0);                //set eeprom lock, 0 = no lock, 1 = locked
  delay(33);
  dxlSetPunch(SERVO_ID, 0);                     //set servo punch, see manual for more info
  delay(33);
  mxSetTorqueControlMode(SERVO_ID, 0);          //set torque control mode , see manual for more info, MX-64/106 only
  delay(33);
  mxSetGoalTorque(SERVO_ID, 0);                 //set goal torque , see manual for more info, MX-64/106 only
  delay(33);
  mxSetGoalAcceleration(SERVO_ID, 0);           //set goal acceleration , see manual for more info
  delay(33);
  
  dxlRegisterReportSingle(SERVO_ID);//report register data for verification


}


void loop()
{
   
}



