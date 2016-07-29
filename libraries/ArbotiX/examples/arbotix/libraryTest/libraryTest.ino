/* Include several libraries to check if the InterbotiX Tools and Libraries have been installed correctly*/
#include <ArmLink.h>


#include <ax12.h>               //include base library for DYNAMIXELs
#include <BioloidController.h>  //include bioloid libary for poses/movements
#include <Commander.h>  //include bioloid libary for poses/movements

BioloidController bioloid = BioloidController(1000000); //start the BIOLOID library at 1mbps. This will run the dxlInit() function internally, so we don't need to call it

ArmLink armlink = ArmLink();



void setup()
{

}
void loop()
{
}
