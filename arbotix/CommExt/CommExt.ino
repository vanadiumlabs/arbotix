/* 
  ArbotiX Firmware - Commander Extended Instruction Set Example
  Copyright (c) 2008-2012 Vanadium Labs LLC.  All right reserved.
 
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/ 
 
#include <ax12.h>
#include <BioloidController.h>
#include <Commander.h>
#include <Motors2.h>

/* Hardware Constructs */
BioloidController bioloid = BioloidController(1000000);
Motors2 drive = Motors2();
Commander command = Commander();
#define PAN    1
#define TILT   2
int pan;
int tilt;

/* Modes - pure commander, or extended instruction set */
#define MODE_COMMANDER    0
#define MODE_EXTENDED     1
int mode = MODE_COMMANDER;

void setup(){
  // setup LED
  pinMode(0,OUTPUT);
  // setup serial
  Serial.begin(38400);
  // setup interpolation, slowly raise turret
  pan = 512;
  tilt = 512;
  delay(1000);
  bioloid.poseSize = 2;
  bioloid.readPose();
  bioloid.setNextPose(PAN,pan);
  bioloid.setNextPose(TILT,tilt);
  bioloid.interpolateSetup(2000);
  while(bioloid.interpolating > 0){
    bioloid.interpolateStep();
    delay(3);
  }
}

void Response(unsigned char command, unsigned char value){
  Serial.write(0xff);
  Serial.write(command);
  Serial.write(value);
  Serial.write((unsigned char)(255 - (command+value)%256));
}
 
void loop(){
  int i;
  
  // take commands
  if(command.ReadMsgs() > 0){
    // advanced commands? 
    if(command.ext > 0x07){
      mode = MODE_EXTENDED;
      if(command.ext < 0x10){
        // no action
      }else if(command.ext < 0x1B){
        // read analogs
        Response(command.ext, analogRead(command.ext & 0x0F) >> 2);
      }else if(command.ext == 0x1B){
        Response(command.ext, (unsigned char) PINB);
      }else if(command.ext < 0x40){
        // error?
      }else if(command.ext == 0x40){
        drive.set(0,0);  // motors OFF
      }else if(command.ext < 0x60){
        drive.left((((int)command.ext - 0x50) * 255)/10); 
      }else if(command.ext < 0x80){
        drive.right((((int)command.ext - 0x70) * 255)/10);
      }else if(command.ext < 0xA0){
        // set digital 
        int pin = (command.ext - 0x80)/4;
        int dir = ((command.ext - 0x80) >> 1)&0x01;
        PORTB = (PORTB & (0xff - (0x01<<pin))) | (((command.ext-0x80)%2)<<pin);
        DDRB = (DDRB & (0xff - (0x01<<pin))) | (dir<<pin);
      }
    }else{
      // toggle LEDs
      digitalWrite(0,HIGH-digitalRead(0));
      mode = MODE_COMMANDER;
    }
      
    // set servo positions, handle other IO
    if(mode == MODE_EXTENDED){
      pan = command.pan;
      tilt = command.tilt;
    }else{
      pan = (pan/2) + (256 - (command.walkH*2));
      tilt = (tilt/2) + (256 + (command.walkV*2));
      // 6 primary buttons control first digital IO 1-6
      for(i=0;i<6;i++){
        unsigned char button = (command.buttons>>i)&0x01;
        if(button > 0){
          // button pressed, go high on a pin
          DDRB |= 0x01<<(i+1);
          PORTB |= 0x01<<(i+1);
        }else{
          DDRB &= 0xff - (0x01<<(i+1));
          PORTB &= 0xff - (0x01<<(i+1));
        } 
      }    
      // top buttons control motors
      if(command.buttons & BUT_RT){
        drive.right(255); 
      }else{
        drive.right(0);
      }
      if(command.buttons & BUT_LT){
        drive.left(255); 
      }else{
        drive.left(0);
      }
    }
    
    SetPosition(PAN,pan);
    SetPosition(TILT,tilt);
  }    
}

