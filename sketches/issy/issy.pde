/* 
  IssyDunnYet reworked code for arbotiX controller using Sanguino environment.
  This sketch will allow you to drive around a 4-legged, 3DOF per leg, quad, using
  the PyMech program. 
  
  Copyright (c) 2008,2009 Michael E. Ferguson.  All right reserved.

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
int walkMode;

void setup(){
    int i;
    Serial.begin(38400);    
    pinMode(0,OUTPUT);
    delay(1000);
    
    // stand up slowly
    bioloid.loadPose(stand);
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
                    //Print("Packet Error\n");
                }else{
                    lstep = nForward+nTurn;
                    rstep = nForward-nTurn;
                    pan = nPan;
                    tilt = nTilt;
                    if(nFire > 0) guns.right(170);
                        else guns.right(0);
                    if(nStatus > 0){
                        bioloid.loadPose(walkend);//stand);
                        bioloid.writePose();
                    }else{
                        bioloid.interpolating = 0;
                        for(data=0;data<12;data++)
                            Relax(data);
                        lstep=0;rstep=0;
                    }           
                }
                mode = 0;
            }else{
                if(addr == REG_FORWARD) nForward = (int) (data-128);
                else if(addr == REG_TURN) nTurn = (int) (data-128);
                else if(addr == REG_PAN) nPan = (int) (data-128);                        
                else if(addr == REG_TILT) nTilt = (int) (data-128);
                else if(addr == REG_FIRE) nFire = (int) data;
                else if(addr == REG_STATUS) nStatus = data;
            }                  
            iter++;addr++;
        }
    }
    
    // update parameters
    if(bioloid.interpolating == 0){
        if((rstep != 0) || (lstep != 0)){    // do nothing if we are stopped
        if(walkMode == 0){
            bioloid.loadPose(walk0);
            bioloid.setNextPose(5,bioloid.getCurPose(5) - rstep);
            bioloid.setNextPose(6,bioloid.getCurPose(6) - lstep);
            bioloid.setNextPose(11,bioloid.getCurPose(11) + rstep);
            bioloid.setNextPose(12,bioloid.getCurPose(12) + lstep);
            walkMode = 1;
            bioloid.interpolateSetup(75);   
        }else if(walkMode == 1){
            bioloid.loadPose(walkend);
            bioloid.setNextPose(5,bioloid.getCurPose(5));
            bioloid.setNextPose(6,bioloid.getCurPose(6));
            bioloid.setNextPose(11,bioloid.getCurPose(11));
            bioloid.setNextPose(12,bioloid.getCurPose(12));
            walkMode = 2;
            bioloid.interpolateSetup(30);   
        }else if(walkMode == 2){
            bioloid.loadPose(walk1);
            bioloid.setNextPose(5,bioloid.getCurPose(5) + rstep);
            bioloid.setNextPose(6,bioloid.getCurPose(6) + lstep);
            bioloid.setNextPose(11,bioloid.getCurPose(11) - rstep);
            bioloid.setNextPose(12,bioloid.getCurPose(12) - lstep);
            walkMode = 3;
            bioloid.interpolateSetup(75);   
        }else{
            bioloid.loadPose(walkend);  
            bioloid.setNextPose(5,bioloid.getCurPose(5));
            bioloid.setNextPose(6,bioloid.getCurPose(6));
            bioloid.setNextPose(11,bioloid.getCurPose(11));
            bioloid.setNextPose(12,bioloid.getCurPose(12));
            walkMode = 0;
            bioloid.interpolateSetup(30);   
        }
        }else{
            // stopped! put feet down!
            bioloid.loadPose(walkend);  
            bioloid.setNextPose(5,bioloid.getCurPose(5));
            bioloid.setNextPose(6,bioloid.getCurPose(6));
            bioloid.setNextPose(11,bioloid.getCurPose(11));
            bioloid.setNextPose(12,bioloid.getCurPose(12));
            bioloid.interpolateSetup(30);   
        }   
    }
    
    // update joints
    bioloid.interpolateStep();
}
