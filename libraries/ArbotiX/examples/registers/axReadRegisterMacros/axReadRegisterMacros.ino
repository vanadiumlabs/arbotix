/***********************************************************************************
 *   ___________          DYNAMIXEL Register Dump
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
 *    This sketch will print out a register table on the serial monitor. This register
 *    will display the data for servo SERVO_ID.
 *    This sketch is meant as a demo to show the individual 'get' register calls. For
 *    creating a serial table, it is reccomended that you use The dxlRegisterReport() 
 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL blinks once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    This Sketch will display data to the Serial() port. Open your Serial Monitor
 *    and set your baud rate to 9600
 *    
 * Compatible Servos:
 *    AX-12A / AX-12+
 *    AX-18A / AX-18F
 *    AX-12W
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




  Serial.println("Manual Register Dump");
  Serial.print("Register Table Servos #");

  Serial.println(SERVO_ID);



    Serial.print("ID #\t\t\t");
    Serial.print(SERVO_ID);
    delay(33);

    Serial.println("");

    Serial.print("MODEL,\t\t\t");
    

        regData = dxlGetModel(SERVO_ID); 

        Serial.print(regData);
        delay(33);
    
    Serial.println("");



    Serial.print("FIRMWARE,\t\t");
    

        regData = dxlGetFirmwareVersion(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("ID,\t\t\t");
    

        regData = dxlGetId(SERVO_ID); 
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("BAUD RATE,\t\t");
    

        regData = dxlGetBaud(SERVO_ID); 
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("RETURN DELAY TIME,\t");
    

        regData = dxlGetReturnDelayTime(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("CW ANGLE LIMIT,\t\t");
    

        regData = dxlGetCWAngleLimit(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("CCW ANGLE LIMIT,\t");
    

        regData = dxlGetCCWAngleLimit(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("TEMPERATURE LIMIT,\t");
    

        regData = dxlGetTempLimit(SERVO_ID); 
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("LOW VOLTAGE LIMIT,\t");
    

        regData = dxlGetLowVoltage(SERVO_ID); 
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("HIGH VOLTAGE LIMIT,\t");
    

        regData = dxlGetHighVoltage(SERVO_ID); 
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("MAX TORQUE,\t\t");
    

        regData = dxlGetStartupMaxTorque(SERVO_ID); 
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("STATUS RETURN LEVEL,\t");
    

        regData = dxlGetStatusReturnLevel(SERVO_ID); 
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("ALARM LED,\t\t");
    

        regData = dxlGetAlarmLED(SERVO_ID); 
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("ALARM SHUTDOWN,\t\t");
    

        regData = dxlGetAlarmShutdown(SERVO_ID); 
        Serial.print(regData);
        delay(33);
    
    Serial.println("");




    Serial.print("TORQUE ENABLE,\t\t");
    

        regData = dxlGetTorqueEnable(SERVO_ID); 
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("LED,\t\t\t");
    

        regData = dxlGetLed(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");


    Serial.print("CW COMPLIANCE,\t\t");
    
  
        regData = axGetCWComplianceMargin(SERVO_ID); 
        Serial.print(regData);
        delay(33);
     
    Serial.println("");




    Serial.print("CCW COMPLIANCE,\t\t");
    
  
        regData = axGetCCWComplianceMargin(SERVO_ID); 
        Serial.print(regData);
        delay(33);
     
    Serial.println("");





    Serial.print("CW COMPLAINCE SLOPE,\t");
    
     
        regData = axGetCWComplianceSlope(SERVO_ID); 
        Serial.print(regData);
        delay(33);
      
    Serial.println("");

    Serial.print("CCW COMPLAINCE SLOPE,\t");
    
        regData = axGetCCWComplianceSlope(SERVO_ID); 
        Serial.print(regData);
        delay(33);
    
    Serial.println("");




    


    Serial.print("GOAL POSITION,\t\t");
    

        regData = dxlGetGoalPosition(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("MOVING SPEED,\t\t");
    

        regData = dxlGetGoalSpeed(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("TORQUE LIMIT,\t\t");
    

        regData = dxlGetTorqueLimit(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("PRESENT POSITION,\t");
    

        regData = dxlGetPosition(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("PRESENT SPEED,\t\t");
    

        regData = dxlGetSpeed(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("PRESENT LOAD,\t\t");
    

        regData = dxlGetTorque(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("PRESENT VOLTAGE,\t");
    

        regData = dxlGetVoltage(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("PRESENT TEMPERATURE,\t");
    

        regData = dxlGetTemperature(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("REGISTERED,\t\t");
    

        regData = dxlGetRegistered(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("MOVING,\t\t\t");
    

        regData = dxlGetMoving(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("LOCK,\t\t\t");
    

        regData = dxlGetLock(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");

    Serial.print("PUNCH,\t\t\t");
    

        regData = dxlGetPunch(SERVO_ID);
        Serial.print(regData);
        delay(33);
    
    Serial.println("");







}
void loop()
{
   
}



