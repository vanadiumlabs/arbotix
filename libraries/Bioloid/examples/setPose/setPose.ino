
/********************************************************
 * 


 ***********************************************************************************/

#include <ax12.h>               //include base library for DYNAMIXELs
#include <BioloidController.h>  //include bioloid libary for poses/movements
#include "poses.h"              

BioloidController bioloid = BioloidController(1000000); //start the BIOLOID library at 1mbps. This will run the dxlInit() function internally, so we don't need to call it

const int SERVOCOUNT = 5;  //number of servos in this robot
int id;                    //temperary id for movement
int pos;                   //temporary position for movement
boolean runCheck = false;  //flag to see if we're running, so that we don't print out the menu options unnecessarily

uint16_t dynamic[] = {5, 512, 512, 512, 512, 512};

void setup()
{
 

    delay(100);                      // recommended pause
    bioloid.loadPose(center);        // load the pose from FLASH, into the nextPose buffer
    bioloid.readPose();              // read in current servo positions to the curPose buffer
    bioloid.interpolateSetup(1000);  // setup for interpolation from current->next over 1 second
    while(bioloid.interpolating > 0) // do this while we have not reached our new pose
    {  
        bioloid.interpolateStep();     // move servos, if necessary. 
        delay(3);                      //movement delay
    }

    delay(100);                      // recommended pause
    bioloid.loadPose(pose1);        // load the pose from FLASH, into the nextPose buffer
    bioloid.readPose();              // read in current servo positions to the curPose buffer
    bioloid.interpolateSetup(1000);  // setup for interpolation from current->next over 1 second
    while(bioloid.interpolating > 0) // do this while we have not reached our new pose
    {  
        bioloid.interpolateStep();     // move servos, if necessary. 
        delay(3);                      //movement delay
    }


}

void loop()
{    
 

  
}








