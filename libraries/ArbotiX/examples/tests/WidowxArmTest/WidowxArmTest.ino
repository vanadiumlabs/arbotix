
/********************************************************
 *  }--\     InterbotiX WidowX Robotic Arm     /--{
 *      |     Test / Diagnostix Code          |
 *   __/                                       \__
 *  |__|                                       |__|
 *
 *
 *  The following sketch will move each joint of the arm based on analog inputs.
 *
 *  Build Check / Tese Guide 
 *  http://learn.trossenrobotics.com/18-interbotix/robot-arms/widowx-robot-arm/27-widowx-robot-arm-build-check
 *
 *  WidowX Arm Getting Started Guide
 *  http://learn.trossenrobotics.com/33-robotgeek-getting-started-guides/robotgeek-snapper-robot-arm/63-robotgeek-snapper-arm-getting-started-guide
 *  WIRING
 *      Power Jumper should be set to VIN    
 *      
 *      12v power supply should be connected to the ArbotiX
 * 
 *      2 AX-12As, 2 MX-64s and 2 MX-28s should be daisy chained to the ArbotiX-M board
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

#include <ax12.h>
#include <BioloidController.h>
#include "poses.h"

BioloidController bioloid = BioloidController(1000000);

const int SERVOCOUNT = 6;  //number of servos in this robot
int id;
int pos;
//float voltage;
boolean RunCheck;          //

void setup()
{
  
   pinMode(0,OUTPUT);  //user led as an output
   
   //initialize variables 

   RunCheck = 0;
  //open serial port
   Serial.begin(9600);
   delay (500);   
    Serial.println("######################################################");    
   Serial.println("Serial Communication Established.");    
   Serial.println("Starting WidowX Robot Arm Test.");    
   
  dxlVoltageReport(SERVOCOUNT);
  
  dxlServoReport(SERVOCOUNT);    //Scan Servos, return position and error 
      
  MoveCenter();    //move all the servos to their centered position

  MoveTest();     //move servos accoring to poses
  
  MoveHome();     //move servos to home position
  
  MenuOptions();  //back to menu
 
  RunCheck = 1;
}

void loop(){
  // read the sensor:
  
    int inByte = Serial.read();

    switch (inByte) {

    case '1':    
       dxlServoReport(SERVOCOUNT);    //Scan Servos, return position and error 
      MenuOptions();
      break;

    case '2':    
      MoveCenter();
      MenuOptions();
      break;
      
     case '3':    
      MoveHome();
      MenuOptions();
      break;     

     case '4':    
      dxlTorqueOffAll();
      MenuOptions();
      break;     

    case '5':
      MoveCenter();
      MoveTest();
      MenuOptions();
      break;
      
    case '6':    
      
      dxlVoltageReport(SERVOCOUNT);
      MenuOptions();
      break;

    case '7':
      Serial.println("Start LED Test");
      dxlLedTest(SERVOCOUNT, 3000);
      Serial.println("End LED Test");
      MenuOptions();
      break;

    case '8':
      dxlRegisterReport(SERVOCOUNT);
      MenuOptions();
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
    bioloid.interpolateSetup(1000); // setup for interpolation from current->next over 1/2 a second
    while(bioloid.interpolating > 0){  // do this while we have not reached our new pose
        bioloid.interpolateStep();     // move servos, if necessary. 
        delay(3);
    }
    if (RunCheck == 1){
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
    if (RunCheck == 1){
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
  pos = 2048;
 
// Base Servo Test
  Serial.println("Moving Servo ID: 1");
  
  while(pos >= 1500)
  {  
    dxlSetGoalPosition(1, pos);
    pos = pos - 1;
    delay(10);
  }

  while(pos <= 2048)
  {  
    dxlSetGoalPosition(1, pos);
    pos = pos + 1;
    delay(10);
  }
  
  delay(500);

// Shoulder Servo Test  
  Serial.println("Moving Servo IDs: 2 "); 
  while(pos >= 1500)
  {  
    dxlSetGoalPosition(2, pos);
    pos = pos - 1;
    delay(10);
  }

  while(pos <= 2048)
  {  
    dxlSetGoalPosition(2, pos);
    pos = pos + 1;
    delay(10);
  }

  delay(500);
  
// Elbow Servo Test  

  Serial.println("Moving Servo IDs: 3 "); 
  while(pos <= 2400)
  {  
    dxlSetGoalPosition(3, pos);
    pos = pos + 1;
    delay(10);
  }

  while(pos >= 2048)
  {  
    dxlSetGoalPosition(3, pos);
    pos = pos - 1;
    delay(10);
  }

  delay(500);


  //Wrist Servo Test
  
  Serial.println("Moving Servo ID: 4");
  
  while(pos <= 2500)
  {  
    dxlSetGoalPosition(4, pos);
    pos = pos + 1;
    delay(10);
  }

  while(pos >= 2048)
  {  
    dxlSetGoalPosition(4, pos);
    pos = pos - 1;
    delay(10);
  }
  
  delay(500);  
 
 pos = 512; 
  
  //Wrist Rotate Servo Test  

  Serial.println("Moving Servo ID: 5");
  
  while(pos >= 312)
  {  
    dxlSetGoalPosition(5, pos);
    pos = pos - 1;
    delay(10);
  }

  while(pos <= 512)
  {  
    dxlSetGoalPosition(5, pos);
    pos = pos + 1;
    delay(10);
  }
  
  delay(500);   
  
  //Gripper Servo Test  
  
    Serial.println("Moving Servo ID: 6");
  
  while(pos >= 312)
  {  
    dxlSetGoalPosition(6, pos);
    pos = pos - 1;
    delay(10);
  }

  while(pos <= 512)
  {  
    dxlSetGoalPosition(6, pos);
    pos = pos + 1;
    delay(10);
  }
  
  delay(500);   
 
    if (RunCheck == 1){
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

void RelaxServos(){
  id = 1;
  Serial.println("###########################");
  Serial.println("Relaxing Servos.");
  Serial.println("###########################");    
  while(id <= SERVOCOUNT){
    Relax(id);
    id = (id + 1)%SERVOCOUNT;
    delay(50);
  }
   if (RunCheck == 1){
      MenuOptions();
  }
}


