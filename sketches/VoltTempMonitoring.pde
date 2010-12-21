/*
 * Example of how to soft-monitor your lipos
 */

#include <ax12.h>
#include <BioloidController.h>
#include "nuke.h"

#define SERVO_COUNT     12

float volts;
int temps[SERVO_COUNT];
int servo;  // which servo to measure

void setup(){
  // setup IK
  setupIK();
  gaitSelect(RIPPLE);
  // setup serial
  Serial.begin(38400);

  // wait, then check the voltage (LiPO safety)
  delay (1000);
  float volts = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
  Serial.print ("System Voltage: ");
  Serial.print (volts);
  Serial.println (" volts.");
  if(volts < 10.0)
    while(1);

  // stand up slowly
  bioloid.poseSize = SERVO_COUNT;
  bioloid.readPose();
  doIK();
  bioloid.interpolateSetup(1000);
  while(bioloid.interpolating > 0){
    bioloid.interpolateStep();
    delay(3);
  }
}

void loop(){
  // Whatever code here to control the bot
  
  // update IK if needed
  if(bioloid.interpolating == 0){
    // setup our next move
    doIK();
    bioloid.interpolateSetup(tranTime);
    // get voltage, we use a small amount of filtering here
    volts = (0.8*volts)+(0.2*((ax12GetRegister(1+servo,AX_PRESENT_VOLTAGE,1)/10.0)));
    // and temperatures
    temps[servo] = ax12GetRegister(1+servo,AX_PRESENT_TEMPERATURE,1);
    servo++;
    // finished looping? if so, output our data
    if(servo == 12){
      Serial.print("Volts:");
      Serial.println(volts);
      for(servo=0;servo<12;servo++){
        Serial.print("S");
        Serial.print(servo+1);
        Serial.print(":");
        Serial.println(temps[servo]);
      }  
      servo = 0;
      if(volts < 9.6){
        // battery depleted, turn off servo torque.
        int i;
        for(i =1; i<=SERVO_COUNT;i++)
           Relax(i);
        while(1);
      }
    }
  }
  
  // update joints
  bioloid.interpolateStep();
}  

