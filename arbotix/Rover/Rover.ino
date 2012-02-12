/*
 * Rover: A Simple 4WD Rover
 * 
 * Walk joystick drives robot
 */

#include <Motors2.h>
#include <Commander.h>

Commander command = Commander();
Motors2 motors = Motors2(); 

void setup(){
  motors.init();
  // make our LED output
  pinMode(0,OUTPUT);
  // setup commander
  command.begin(38400);
}

void loop(){
  // take commands
  if(command.ReadMsgs() > 0){
    digitalWrite(0,HIGH-digitalRead(0));
    // set speeds
    int left = 2*command.walkV + 2*command.walkH;
    int right = 2*command.walkV - 2*command.walkH;
    motors.set(left, right); 
  }
}
