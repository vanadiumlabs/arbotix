/***********************************************************************************
 *   ___________          register dump
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
 *    This is the same code as used in dxlRegisterReportMultiple()

 *    
 *  Physical Build / Instructions:
 *    Connect your servo(s) to the ArbotiX-M with a 3-pin cable. Power the ArbotiX-M
 *    with a 12v power supply (make sure each DYNAMIXEL blinks once when power
 *    is applied). The power jumper on the ArbotiX should be set to 'VIN'
 *    This Sketch will display data to the Serial() port. Open your Serial Monitor
 *    and set your baud rate to 38400
 *    
 * Compatible Servos:
 *    AX-12A / AX-12+
 *    AX-18A / AX-18F
 *
 *
 ********************************************************************************/
#include <ax12.h> //include the ArbotiX DYNAMIXEL library

const int SERVO_ID = 1;

void setup()
{
  dxlInit(1000000);    //start dynamixel library at 1mbps to communicate with the servos
  Serial.begin(9600); //start serial at 38400bps for reporting data

  int numberOfServos = 3;
  int servoList[] = {1,2,3};
  
  int regData;
  int modelData[numberOfServos];

  Serial.println("Register Table for All Servos #");

  Serial.print("Register,\t\t");

    for(int i = 0; i< numberOfServos; i++)
    {
        Serial.print("ID #");
        Serial.print(servoList[i]);
        Serial.print(",\t");
        delay(33);
    }



  

    // Serial.print("SERVO #,\t");
    // for(int i = 0; i< numberOfServos; i++)
    // {
    //     Serial.print(servoList[i]);
    //     Serial.print(",");
    //     delay(33);
    // }
    Serial.println("");

    Serial.print("MODEL,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetModel(servoList[i]); 

        modelData[i] = regData; //store register data forr later printout


        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");



    Serial.print("FIRMWARE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetFirmwareVersion(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("ID,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetId(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("BAUD RATE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetBaud(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("RETURN DELAY TIME,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetReturnDelayTime(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("CW ANGLE LIMIT,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetCWAngleLimit(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("CCW ANGLE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetCCWAngleLimit(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("TEMPERATURE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTempLimit(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("LOW VOLTAGE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetLowVoltage(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("HIGH VOLTAGE LIMIT,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetHighVoltage(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("MAX TORQUE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetStartupMaxTorque(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("STATUS RETURN LEVEL,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetStatusReturnLevel(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("ALARM LED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetAlarmLED(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("ALARM SHUTDOWN,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetAlarmShutdown(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");



    Serial.print("Multi Turn Offset,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetMultiTurnOffset(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("Resolution Divider,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetResolutionDivider(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");









    Serial.print("TORQUE ENABLE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTorqueEnable(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("LED,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetLed(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");


    Serial.print("CW COMPLIANCE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCWComplianceMargin(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");




    Serial.print("CCW COMPLIANCE,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCCWComplianceMargin(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");





    Serial.print("CW COMPLAINCE SLOPE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCWComplianceSlope(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");

    Serial.print("CCW COMPLAINCE SLOPE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelAX(modelData[i]) == true)
      {
        regData = axGetCCWComplianceSlope(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");




    Serial.print("D,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetD(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("I,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetI(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("P,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetP(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");







    Serial.print("GOAL POSITION,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetGoalPosition(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("MOVING SPEED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetGoalSpeed(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("TORQUE LIMIT,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTorqueLimit(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT POSITION,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetPosition(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT SPEED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetSpeed(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT LOAD,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTorque(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT VOLTAGE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetVoltage(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PRESENT TEMPERATURE,\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetTemperature(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("REGISTERED,\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetRegistered(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("MOVING,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetMoving(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("LOCK,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetLock(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");

    Serial.print("PUNCH,\t\t\t");
    for(int i = 0; i< numberOfServos; i++)
    {

        regData = dxlGetPunch(servoList[i]);
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
    }
    Serial.println("");




    Serial.print("Current(MX64/106),\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetCurrent(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("Torque Mode(MX64/106),\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetTorqueMode(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");


    Serial.print("Goal Torque(MX64/106),\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetGoalTorque(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");



    Serial.print("Goal acceleration,\t");
    for(int i = 0; i< numberOfServos; i++)
    {
      if(dxlIsModelMX(modelData[i]) == true)
      {
        regData = mxGetGoalAcceleration(servoList[i]); 
        Serial.print(regData);
        Serial.print(",\t");
        delay(33);
      }
      else
      {

        Serial.print("N/A");
        Serial.print(",\t");
      }
    }
    Serial.println("");






}
void loop()
{
   
}



