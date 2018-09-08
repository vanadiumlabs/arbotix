
/********************************************************
 * 


 ***********************************************************************************/

#include <ax12.h>               //include base library for DYNAMIXELs
#include <BioloidController.h>  //include bioloid libary for poses/movements
#include "poses.h"              

BioloidController bioloid = BioloidController(1000000); //start the BIOLOID library at 1mbps. This will run the dxlInit() function internally, so we don't need to call it

const int SERVOCOUNT = 6;  //number of servos in this robot
int id;                    //temperary id for movement
int pos;                   //temporary position for movement
boolean runCheck = false;  //flag to see if we're running, so that we don't print out the menu options unnecessarily

uint16_t dynamic[] = {5, 512, 512, 512, 512, 512};

void setup()
{
 
pinMode(0, OUTPUT);
pinMode(1, OUTPUT);
Serial.begin(38400);

Serial.println("Begin");

                 // read in current servo positions to the curPose buffer
//  while(bioloid.readPose() == false)
//  {
//    delay(500);
//  }

Serial.println("done read pose");
  digitalWrite(1, HIGH);
  delay(2000);
  digitalWrite(1, LOW);



}

void loop()
{    
    digitalWrite(0, LOW);
 
    delay(100);                      // recommended pause
    bioloid.loadPose(center);        // load the pose from FLASH, into the nextPose buffer
   if(bioloid.readPose() == false)              // read in current servo positions to the curPose buffer

    {

      Serial.println("SUPER BAD READ");
    }
    
    bioloid.interpolateSetup(2000);  // setup for interpolation from current->next over 1 second
    while(bioloid.interpolating > 0) // do this while we have not reached our new pose
    {  
        bioloid.interpolateStep();     // move servos, if necessary. 
        delay(3);                      //movement delay
    }
      dxlServoReport(SERVOCOUNT); //scan for NUMBER_OF_SERVOS 

    digitalWrite(0, HIGH);//test

    delay(100);                      // recommended pause
    bioloid.loadPose(pose1);        // load the pose from FLASH, into the nextPose buffer
    bioloid.readPose();              // read in current servo positions to the curPose buffer
    bioloid.interpolateSetup(2000);  // setup for interpolation from current->next over 1 second
    while(bioloid.interpolating > 0) // do this while we have not reached our new pose
    {  
        bioloid.interpolateStep();     // move servos, if necessary. 
        delay(3);                      //movement delay
    }
    

  dxlServoReport(SERVOCOUNT); //scan for NUMBER_OF_SERVOS 

  
}








