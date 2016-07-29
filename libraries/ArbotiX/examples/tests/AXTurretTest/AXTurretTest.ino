
/********************************************************
 * InterbotiX PhantomX Turret
 *  Test / Diagnostic Code
 *
 *  The following sketch will move each joint of the arm based on analog inputs.
 *
 *  Build Check / Test Guide 
 *  http://learn.trossenrobotics.com/18-interbotix/robot-arms/widowx-robot-arm/27-widowx-robot-arm-build-check
 *
 *  PhantomX Robot Turret Getting Started Guide
 *  http://learn.trossenrobotics.com/33-robotgeek-getting-started-guides/robotgeek-snapper-robot-arm/63-robotgeek-snapper-arm-getting-started-guide
 *  WIRING
 *      Power Jumper should be set to VIN    
 *      
 *      12v power supply should be connected to the ArbotiX
 * 
 *      2 AX-12As or 2 AX-18As should be daisy chained to the ArbotiX-M board
 *
 *      FTDI-USB cable should be connected to the ArbotiX board
 *
 *      No other hardware should be connected
 *  
 *  RUNNING THE TEST
 *        Once the firmware is loaded, open the Serial Monitor. Set the baud to 9600.
 *
 *        The arm will scan voltage and scan the servos. If everything is working correctly, 
 *          the program will begin to move the servos. If not it will report the problem.
 *        
 *
 ***********************************************************************************/

#include <ax12.h>               //include base library for DYNAMIXELs
#include <BioloidController.h>  //include bioloid libary for poses/movements
#include "poses.h"

BioloidController bioloid = BioloidController(1000000); //start the BIOLOID library at 1mbps. This will run the dxlInit() function internally, so we don't need to call it

const int SERVOCOUNT = 2;  //number of servos in this robot
int id;                    //temperary id for movement
int pos;                   //temporary position for movement
boolean runCheck = false;  //flag to see if we're running, so that we don't print out the menu options unnecessarily

void setup()
{
   pinMode(USER_LED,OUTPUT);  //user led as an output
   digitalWrite(USER_LED, HIGH); //set LED high to show that the test has started


  
   Serial.begin(9600); //open serial port

   Serial.println("######################################################");    
   Serial.println("Serial Communication Established.");    
   Serial.println("Starting PhantomX Turret Test.");    
   
  dxlVoltageReport(SERVOCOUNT);  //serial report for the system voltage  
  dxlServoReport(SERVOCOUNT);    //Scan Servos, return position and error (if there are any)
      
  MoveCenter();   //move all the servos to their centered position
  MoveTest();     //move each individual servo sequentially  
  MoveHome();     //move servos to home position
  
  MenuOptions();  //back to menu
 
  runCheck = true;
}

void loop(){
    int inByte = Serial.read(); //read data from serial port for menu option

    switch (inByte) 
    {

    case '1':    
      dxlServoReport(SERVOCOUNT);    //Scan Servos, return position and error 
      MenuOptions();                 //show menu options
      break;

    case '2':    
      MoveCenter();                   //move all the servos to their centered position
      MenuOptions();                 //show menu options
      break;
      
     case '3':    
      MoveHome();                    //move servos to home position
      MenuOptions();                 //show menu options
      break;     

     case '4':    
      dxlTorqueOffAll();             //turn the torque off to all servos
      MenuOptions();                 //show menu options
      break;     

    case '5':
      MoveCenter();                  //move all the servos to their centered position
      MoveTest();                    //move each individual servo sequentially  
      MenuOptions();                 //show menu options
      break;
      
    case '6':    
      
      dxlVoltageReport(SERVOCOUNT);   //get and report average voltage
      MenuOptions();                 //show menu options
      break;

    case '7':
      Serial.println("Start LED Test");
      dxlLedTest(SERVOCOUNT, 3000);       //turn on/off each LED sequentially for 3000ms each
      Serial.println("End LED Test");
      MenuOptions();                 //show menu options
      break;

    case '8':
      dxlRegisterReport(SERVOCOUNT); //print out all register data to the serial port
      MenuOptions();                 //show menu options
      break;

    } 
  
}


void MoveCenter(){
    delay(100);                    // recommended pause
    bioloid.loadPose(Center);      // load the pose from FLASH, into the nextPose buffer
    bioloid.readPose();            // read in current servo positions to the curPose buffer
    Serial.println("###########################");
    Serial.println("Moving servos to centered position");
    Serial.println("###########################");    
    delay(1000);
    bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1000ms
    while(bioloid.interpolating > 0){  // do this while we have not reached our new pose
        bioloid.interpolateStep();     // move servos, if necessary. 
        delay(3);
    }
    //display menu options if we're outside the first test sequence
    if (runCheck == true)
    {
      MenuOptions();
    }
}


void MoveHome(){
    delay(100);                    // recommended pause
    bioloid.loadPose(Home);   // load the pose from FLASH, into the nextPose buffer
    bioloid.readPose();            // read in current servo positions to the curPose buffer
    Serial.println("###########################");
    Serial.println("Moving servos to Home position");
    Serial.println("###########################");    
    delay(1000);
    bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1/2 a second
    while(bioloid.interpolating > 0){  // do this while we have not reached our new pose
        bioloid.interpolateStep();     // move servos, if necessary. 
        delay(3);
    }
    //display menu options if we're outside the first test sequence
    if (runCheck == true)
    {
      MenuOptions();
    }
}



void MoveTest()
{
  Serial.println("###########################");
  Serial.println("Initializing Movement Sign Test");  
  Serial.println("###########################");
  delay(500);  
  id = 1;
  pos = 512;
 
// Pans Servo Test
  Serial.println("Moving Servo ID: 1");
  
  while(pos >= 312)
  {  
    dxlSetGoalPosition(1, pos);
    pos = pos - 1;
    delay(10);
  }

  while(pos <= 512)
  {  
    dxlSetGoalPosition(1, pos);
    pos = pos + 1;
    delay(10);
  }
  
  delay(500);

// Tilt Servo Test  
  Serial.println("Moving Servo IDs: 2 "); 
  while(pos >= 312)
  {  
    dxlSetGoalPosition(2, pos);
    pos = pos - 1;
    delay(10);
  }

  while(pos <= 512)
  {  
    dxlSetGoalPosition(2, pos);
    pos = pos + 1;
    delay(10);
  }

  delay(500);
  

  
 
    //display menu options if we're outside the first test sequence
    if (runCheck == true)
    {
      MenuOptions();
    }
}



void MenuOptions()
{
  
    Serial.println("###########################"); 
    Serial.println("Please enter option 1-5 to run individual tests again.");     
    Serial.println("1) Servo Scanning Test");        
    Serial.println("2) Move Servos to Center"); 
    Serial.println("3) Move Servos to Home");     
    Serial.println("4) Relax Servos");            
    Serial.println("5) Perform Movement Sign Test");                
    Serial.println("6) Check System Voltage");   
    Serial.println("7) Perform LED Test");       
    Serial.println("8) Dump Servo Register Tables");       
    Serial.println("###########################"); 
}


