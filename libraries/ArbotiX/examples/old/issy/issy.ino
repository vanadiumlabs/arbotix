/* 
  IssyDunnYet reworked code for arbotiX controller using Sanguino environment.
  This sketch will allow you to drive around a 4-legged, 3DOF per leg, quad, using
  the PyMech program. 
  
  Copyright (c) 2008-2012 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Motors2.h>
#include <ax12.h>
#include <BioloidController.h>
#include "poses.h"

BioloidController bioloid = BioloidController(1000000);
Motors2 guns = Motors2();

#define VOLTAGE_READ    7

// servo numbers
#define RIGHT_FRONT     5
#define LEFT_FRONT      6
#define RIGHT_REAR      11
#define LEFT_REAR       12
#define TILT_SERVO      13

// we use a register based write algorithm, similar to the Dynamixel bus. 
#define REG_FORWARD     0x10
#define REG_TURN        0x11    // -128..128    
#define REG_PAN         0x12    // turret pan servo, value in degrees, 0=forward, -90=left
#define REG_TILT        0x13
#define REG_FIRE        0x14
#define REG_STATUS      0x20

int mode = 0; // where are we in our walk cycle?
int addr = 0; // what register address are we writing to?
int count = 0; 
int checksum = 0;
int iter = 0;

/* temporary holders during a register packet write. */
int nForward = 0;
int nTurn = 0;
int nPan = 0;
int nTilt = 0;
int nFire = 0;
int nStatus = 0;
unsigned char data;

/* Movement parameters */
int rstep, lstep;
int pan, tilt;
int walkMode; // where are we in our step sequence of 0-3, -1 = relaxed

/* Power variables */
int vmain = 1200;
unsigned long vtime = 0; // last time we sent out voltages
    
void setup(){
    int i;
    Serial.begin(38400);    
    pinMode(0,OUTPUT);
    delay(1000);
    
    // stand up slowly
    bioloid.loadPose(walkend);
    bioloid.readPose();    
    bioloid.interpolateSetup(1500); // from current position to standing in 1.5 seconds
    while(bioloid.interpolating > 0){
        bioloid.interpolateStep();
        delay(3);
    }
    delay(500);
    
    Serial.println("Issy Alive");
    // walking parameters
    walkMode = 0;
    rstep = 0;
    lstep = 0;
    mode = 0;
}

/* main loop, read in serial data and walk. */
void loop(){
    int i;
    
    // process messages coming in from PyMech
    while(Serial.available() > 0){
        // We need to 0xFF at start of packet
        if(mode == 0){         // start of new packet
            if(Serial.read() == 0xff)
                mode = 1;
            digitalWrite(0,HIGH-digitalRead(0));
        }else if(mode == 1){   // next byte is address
            addr = (int) Serial.read();    
            if(addr != 0xff)
                mode = 2;
        }else if(mode == 2){   // next byte is byte count to write
            count = (int) Serial.read();
            checksum = addr + count;
            iter = 0;
            mode = 3;
        }else if(mode == 3){   // read data in 
            data = (unsigned char) Serial.read();
            checksum += (int) data;
            if(iter == count){
                // process request
                if((checksum%256) != 255){ 
                    Serial.println("Packet Error");
                }else{
                    // deal with status/relax updates
                    if(nStatus > 0){
                        bioloid.loadPose(walkend); //stand);
                        bioloid.writePose();
                        nStatus = 0;
                    }else if(nStatus < 0){
                        bioloid.interpolating = 0;
                        for(data=0;data<12;data++)
                            Relax(data);
                        lstep=0;rstep=0;
                        nStatus = 0;
                    }else{  //  if not paused, or just standing, lets update stuff
                        lstep = nForward+nTurn;
                        rstep = nForward-nTurn;
                        // issy has no pan servo, instead we shift the body
                        if(nPan != 0){ // note that shifting only works when not walking. 
                            bioloid.loadPose(walkend);
                            // left front back, rightfront forward
                            bioloid.setNextPose(LEFT_FRONT, bioloid.getNextPose(LEFT_FRONT) + nPan);
                            bioloid.setNextPose(RIGHT_FRONT, bioloid.getNextPose(RIGHT_FRONT) + nPan);
                            // left rear back, right rear back
                            bioloid.setNextPose(LEFT_REAR, bioloid.getNextPose(LEFT_REAR) + nPan);
                            bioloid.setNextPose(RIGHT_REAR, bioloid.getNextPose(RIGHT_REAR) + nPan);
                            bioloid.writePose();                            
                        }                        
                        SetPosition(TILT_SERVO, 512 + nTilt*2);
                    }
                    // fire guns, or stop firing
                    if(nFire > 0) guns.right(170);
                        else guns.right(0);
                }
                mode = 0;
            }else{
                if(addr == REG_FORWARD) nForward = (int) (data-128);
                else if(addr == REG_TURN) nTurn = (int) (data-128);
                else if(addr == REG_PAN) nPan = (int) (data-128);                        
                else if(addr == REG_TILT) nTilt = (int) (data-128);
                else if(addr == REG_FIRE) nFire = (int) (data-128);
                else if(addr == REG_STATUS) nStatus = (int) (data-128);
            }                  
            iter++;addr++;
        }
    }
    
    // update parameters
    if(bioloid.interpolating == 0){
        if((rstep != 0) || (lstep != 0)){    // do nothing if we are stopped
        if(walkMode == 0){
            bioloid.loadPose(walk0);
            bioloid.setNextPose(RIGHT_FRONT,bioloid.getNextPose(RIGHT_FRONT) - rstep);
            bioloid.setNextPose(LEFT_FRONT,bioloid.getNextPose(LEFT_FRONT) - lstep);
            bioloid.setNextPose(RIGHT_REAR,bioloid.getNextPose(RIGHT_REAR) + rstep);
            bioloid.setNextPose(LEFT_REAR,bioloid.getNextPose(LEFT_REAR) + lstep);
            walkMode = 1;
            bioloid.interpolateSetup(75);   
        }else if(walkMode == 1){
            bioloid.loadPose(walkend);
            bioloid.setNextPose(RIGHT_FRONT,bioloid.getCurPose(RIGHT_FRONT));
            bioloid.setNextPose(LEFT_FRONT,bioloid.getCurPose(LEFT_FRONT));
            bioloid.setNextPose(RIGHT_REAR,bioloid.getCurPose(RIGHT_REAR));
            bioloid.setNextPose(LEFT_REAR,bioloid.getCurPose(LEFT_REAR));
            walkMode = 2;
            bioloid.interpolateSetup(30);   
        }else if(walkMode == 2){
            bioloid.loadPose(walk1);
            bioloid.setNextPose(RIGHT_FRONT,bioloid.getNextPose(RIGHT_FRONT) + rstep);
            bioloid.setNextPose(LEFT_FRONT,bioloid.getNextPose(LEFT_FRONT) + lstep);
            bioloid.setNextPose(RIGHT_REAR,bioloid.getNextPose(RIGHT_REAR) - rstep);
            bioloid.setNextPose(LEFT_REAR,bioloid.getNextPose(LEFT_REAR) - lstep);
            walkMode = 3;
            bioloid.interpolateSetup(75);   
        }else{
            bioloid.loadPose(walkend);  
            bioloid.setNextPose(RIGHT_FRONT,bioloid.getCurPose(RIGHT_FRONT));
            bioloid.setNextPose(LEFT_FRONT,bioloid.getCurPose(LEFT_FRONT));
            bioloid.setNextPose(RIGHT_REAR,bioloid.getCurPose(RIGHT_REAR));
            bioloid.setNextPose(LEFT_REAR,bioloid.getCurPose(LEFT_REAR));
            walkMode = 0;
            bioloid.interpolateSetup(30);   
        }
        }else{
            // stopped! put feet down!
            bioloid.loadPose(walkend);  
            bioloid.setNextPose(RIGHT_FRONT,bioloid.getCurPose(RIGHT_FRONT));
            bioloid.setNextPose(LEFT_FRONT,bioloid.getCurPose(LEFT_FRONT));
            bioloid.setNextPose(RIGHT_REAR,bioloid.getCurPose(RIGHT_REAR));
            bioloid.setNextPose(LEFT_REAR,bioloid.getCurPose(LEFT_REAR));
            bioloid.interpolateSetup(30);   
        }   
    }
    
    // voltage monitoring loop
    if((millis() - vtime) > 3000){
        int v = analogRead(VOLTAGE_READ);
        // convert V to voltage (since it is scaled), and then add to running average
        vmain = ((vmain * 5)/10) + ((((v * 14)/10) * 5)/10);
        if(vmain < 980){ // 9.8V is fairly conservative, but shutdown!
            Serial.println("Battery Expired!");
            guns.right(0); // kill guns
            for(mode=0;mode<13;mode++) // kill servos
                Relax(mode);
            while(1);
        }else{ // print out voltage 
            float x = vmain/100;
            Serial.print("Voltage:");
            Serial.println(x);
            vtime = millis();
        }
        // handle temp readback
        for(v=1;v<13;v++){    
            int data = ax12GetRegister(v, AX_PRESENT_TEMPERATURE, 1);                
            Serial.print("S");
            Serial.print((v/100));
            Serial.print((v/10)%10);
            Serial.print(v%10);
            Serial.print("TEMP:");
            Serial.print((data/100));
            Serial.print((data/10)%10);
            Serial.println(data%10);
        }
    }
    
    // update joints
    bioloid.interpolateStep();
}
