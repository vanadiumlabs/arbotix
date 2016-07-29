
/********************************************************
 * InterbotiX PhantomX Quadruped  Test / Diagnostic Code                                           
 *
 *
 *  The following sketch will scan / check servos and then
 *  move each servo. Your robot should move to the same positions
 *  as in the Build Check Guide
 *  
 *  Build Check / Tese Guide 
 *  http://learn.trossenrobotics.com/interbotix/robot-crawlers/11-phantomx-quadruped/28-phantomx-quadruped-build-check
 *  
 *  Robot  Getting Started Guide
 * http://learn.trossenrobotics.com/interbotix/robot-crawlers/phantomx-quadruped
 *  WIRING
 *      Power Jumper should be set to VIN    
 *      
 *      12v power supply should be connected to the ArbotiX
 * 
 *      12 AX-12As  should be daisy chained to the ArbotiX-M board
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

const int SERVOCOUNT = 12;  //number of servos in this robot
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
   Serial.println("Starting Robot Test.");    

   
  dxlVoltageReport(SERVOCOUNT);  //serial report for the system voltage  
  dxlServoReport(SERVOCOUNT);    //Scan Servos, return position and error (if there are any)
      
  MoveCenter();   //move all the servos to their centered position
  LeftLegTest(); //move each individual servo sequentially  
  RightLegTest();//move each individual servo sequentially  
  
  MenuOptions();  //back to menu
 
  runCheck = true;  //any function calls will show the menu after this 
}

void loop()
{    
  int inByte = Serial.read(); //read data from serial port for menu option

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
      
      dxlTorqueOffAll();
      MenuOptions();
      break;     

    case '4':    
      LeftLegTest();
      break;
      
    case '5':    
      RightLegTest();
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
    if (runCheck == 1)
    {
      MenuOptions();
    }
}





void LeftLegTest(){
  Serial.println("###########################");
  Serial.println("Initializing Left Leg Tests");  
  Serial.println("###########################");
  delay(500);  
  id = 1;
  while(id <= (SERVOCOUNT-1)){
    
  Serial.print("Moving Servo ID: ");
  Serial.println(id);    
  dxlSetGoalPosition(id, 412);
  delay(1000);
  dxlSetGoalPosition(id, 512);
  delay(1000);
//iterate to next servo ID
  id = id + 2;


  Serial.print("Moving Servo ID: ");
  Serial.println(id);
  dxlSetGoalPosition(id, 712);
  delay(1000);
  dxlSetGoalPosition(id, 512);
  delay(1000);
//iterate to next servo ID
  id = id + 2;  


  Serial.print("Moving Servo ID: ");
  Serial.println(id);
  dxlSetGoalPosition(id, 712);
  delay(1000);
  dxlSetGoalPosition(id, 512);
  delay(1000);
  //iterate to next servo ID
  id = id + 2;  
  }
    if (runCheck == 1){
   MenuOptions();
  }
  
}




void RightLegTest(){

  Serial.println("###########################");
  Serial.println("Initializing Right Leg Tests"); 
  Serial.println("###########################");
  delay(500);  
  id = 2;
  while(id <= SERVOCOUNT){

  Serial.print("Moving Servo ID: ");
  Serial.println(id);  
  dxlSetGoalPosition(id, 612);
  delay(1000);
  dxlSetGoalPosition(id, 512);
  delay(1000);
//iterate to next servo ID
  id = id + 2;


  Serial.print("Moving Servo ID: ");
  Serial.println(id);
  dxlSetGoalPosition(id, 312);
  delay(1000);
  dxlSetGoalPosition(id, 512);
  delay(1000);
//iterate to next servo ID
  id = id + 2;  


  Serial.print("Moving Servo ID: ");
  Serial.println(id);
  dxlSetGoalPosition(id, 312);
  delay(1000);
  dxlSetGoalPosition(id, 512);
  delay(1000);
  //iterate to next servo ID
  id = id + 2;  
  }
      if (runCheck == 1){
      MenuOptions();
  }
  
}


void MenuOptions()
{
  
    Serial.println("###########################"); 
    Serial.println("Please enter option 1-8 to run individual tests again.");     
    Serial.println("1) Servo Scanning Test");        
    Serial.println("2) Move Servos to Center");   
    Serial.println("3) Relax Servos");             
    Serial.println("4) Perform Left Leg Sign Test");  
    Serial.println("5) Perform Right Leg Sign Test");                
    Serial.println("6) Check System Voltage");   
    Serial.println("7) Perform LED Test");       
    Serial.println("8) Dump Servo Register Tables");       
    Serial.println("###########################"); 
}




