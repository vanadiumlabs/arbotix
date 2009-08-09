/* 
  ArbotiX Test Program for use with PyPose V1.0
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
 
#include <ax12.h>
#include <BioloidController.h>

BioloidController bioloid = BioloidController(1000000);

#define ARB_SIZE_POSE   7  // also initializes
#define ARB_LOAD_POSE   8
#define ARB_PLAY_POSE   9
#define ARB_COMP_POSE   10

int mode = 0;              // where we are in the frame

unsigned char id = 0;      // id of this frame
unsigned char length = 0;  // length of this frame
unsigned char ins = 0;     // instruction of this frame

unsigned char params[30];  // parameters
unsigned char index = 0;   // index in param buffer

int checksum;              // checksum

void setup(){
    Serial.begin(38400);    
    pinMode(0,OUTPUT);     // status LED
}

/* 
 * packet: ff ff id length ins params checksum
 *   same as ax-12 table, except, we define new instructions for Arbotix
 *
 * ID = 253 for these special commands!
 * Pose Size = 7, followed by single param: size of pose
 * Load Pose = 8, followed by pose positions (# of param = 2*pose_size)
 * Play Pose = 9, followed by 2-byte param: #ms to interpolate
 * Moving = 10, returns whether we are interpolating or not
 */

void loop(){
    int i;
    
    // process messages
    while(Serial.available() > 0){
        // We need to 0xFF at start of packet
        if(mode == 0){         // start of new packet
            if(Serial.read() == 0xff){
                mode = 2;
                digitalWrite(0,HIGH-digitalRead(0));
            }
        //}else if(mode == 1){   // another start byte
        //    if(Serial.read() == 0xff)
        //        mode = 2;
        //    else
        //        mode = 0;
        }else if(mode == 2){   // next byte is index of servo
            id = Serial.read();    
            if(id != 0xff)
                mode = 3;
        }else if(mode == 3){   // next byte is length
            length = Serial.read();
            checksum = id + length;
            mode = 4;
        }else if(mode == 4){   // next byte is instruction
            ins = Serial.read();
            checksum += ins;
            index = 0;
            mode = 5;
        }else if(mode == 5){   // read data in 
            params[index] = Serial.read();
            checksum += (int) params[index];
            index++;
            if(index + 1 == length){  // we've read params & checksum
                mode = 0;
                if((checksum%256) != 255){ 
                    //Print("Packet Error\n");
                    Serial.print("PE");
                }else{
                    if(id == 253){
                        // special ArbotiX instructions
                        if(ins == ARB_SIZE_POSE){
                            bioloid.poseSize = params[0];
                            bioloid.readPose();    
                        }else if(ins == ARB_LOAD_POSE){
                            int i;
                            for(i=0;i<bioloid.poseSize;i++){
                                bioloid.setNextPose(i+1,params[i]);   
                            }
                        }else if(ins == ARB_PLAY_POSE){
                            int t = params[0] + (params[1]<<8);
                            bioloid.interpolateSetup(t);   
                        }else if(ins == ARB_COMP_POSE){
                            if(bioloid.interpolating){
                                
                            }else{
                                
                            }
                        }   
                    }else{
                        // pass thru
                        if(ins == AX_READ_DATA){
                            int i;
                            ax12GetRegister(id, params[0], params[1]);
                            // return a packet: FF FF id Len Err params check
                            for(i=0;i<ax_rx_buffer[3]+4;i++)
                                Serial.print(ax_rx_buffer[i],BYTE);
                        }else if(ins == AX_WRITE_DATA){
                            if(length == 4){
                                ax12SetRegister(id, params[0], params[1]);
                            }else{
                                int x = params[1] + (params[2]<<8);
                                ax12SetRegister2(id, params[0], x);
                            }
                        }
                    }
                }
            }
        }
    }
    
    // update joints
    bioloid.interpolateStep();
}

