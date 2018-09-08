/*
  ArmLink.cpp - Library for interfacing with ArbotiX based Robotic Arms.
  Based on Commander Libraries written by:
  Copyright (c) 2009-2012 Michael E. Ferguson.  All right reserved.
  Copyright (c) 2013 Trossen Robotics. All right reserved.

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

#include <Arduino.h>
#include "ArmLink.h"

/* Constructor */
ArmLink::ArmLink(){
    index = -1;
    status = 0;


}

void ArmLink::begin(int baud){
    Serial.begin(baud);
}


/* process messages coming from ArmLink 
 *  format = 0xFF Xaxis Yaxis Zaxis W_angle W_rot Gripper BUTTONS EXT CHECKSUM */
int ArmLink::ReadMsgs(){

   while(Serial.available() > 0){


        if(index == -1){         // looking for new packet
            if(Serial.read() == 0xff){

                index = 0;
                checksum = 0;
            }
        }else if(index == 0){
            vals[index] = (unsigned char) Serial.read();
            if(vals[index] != 0xff){            
                checksum += (int) vals[index];
                index++;
            }
        }else{
            vals[index] = (unsigned char) Serial.read();
            checksum += (int) vals[index];
            index++;
            if(index == 16){ // packet complete
                if(checksum%256 != 255){
                    // packet error!
                    index = -1;

                //flush serial
                while(Serial.available())
                {
                  Serial.read();
                }
                    return 0;
                }else{
                    Xaxis = ((vals[0]<<8) + vals[1]);
                    Yaxis = (vals[2]<<8) + vals[3];
					Zaxis = (vals[4]<<8) + vals[5];
					W_ang = ((vals[6]<<8) + vals[7]);
					W_rot = (vals[8]<<8) + vals[9];
					Grip = (vals[10]<<8) + vals[11];
					dtime = vals[12];
                    buttons = vals[13];
                    ext = vals[14];
                }
                index = -1;
                //flush serial
                while(Serial.available())
                {
                  Serial.read();
                }
               // Serial.flush();


                return 1;
            }
        }
    }
    return 0;
}
