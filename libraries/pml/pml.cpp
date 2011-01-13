/* 
  pml.cpp - Planar Meta-Laser Library
  Copyright (c) 2010 Vanadium Labs LLC.  All right reserved.
 
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

#include <WProgram.h>
#include "pml.h"

/* setup default parameters */
PML::PML(){}
void PML::init(){
  enabled = 0;
  servo_id = -1;
  sensor_id = 0;
  start = 209;
  ticks = 21;
  steps = MAX_READINGS;
  direction = UP_SCAN;
  time = millis() + PML_TIME_FRAME;
}

/* select servo, center it */
void PML::setServo(int id){
  servo_id = id;
  SetPosition(servo_id, 512);
}

/* start, stop scanning */
void PML::enable(){
  if(servo_id > 0){
    index = steps/2;
    position = start + (index*ticks); 
    direction = UP_SCAN;
    shutdown = 0;
    enabled = 1;
  }
}
void PML::disable(){
  shutdown = 1;
}

/* adjust the scan range parameters */
void PML::setupStep(int step_start, int step_value, int step_count){
  start = step_start;
  ticks = step_value;  
  steps = step_count;
  if( index >= step_count){
    index = step_count-1;
  }else if( position < start){
    index = 0;
  }else{
    index = (position - start)/ticks;
  }
  position = start + ticks*index;
}

/* take our next reading */
void PML::step(){
  if(enabled > 0){
    unsigned long j = millis();
    if(j > time){
      time = j+PML_TIME_FRAME;
      // reading from IR sensor
      int v = analogRead(sensor_id);
      // save reading
      if(direction == UP_SCAN){
        data_up[index*2] = v % 256;
        data_up[(index*2)+1] = v>>8;
      }else{
        data_dn[index*2] = v % 256;
        data_dn[(index*2)+1] = v>>8;
      }
      // shutdown head if needed
      if((shutdown > 0) && (position > 482) && (position < 542)){
        enabled = 0;
        SetPosition(servo_id,512);
      }else{
        // otherwise, move head
        if(direction == UP_SCAN){
          index++;
          if(index >= steps){
            index--;
            direction = DN_SCAN;
            data_dn_time = time;
          }else{
            position += ticks;
            SetPosition(servo_id, position); 
          }
        }else{
          index--;
          if(index < 0){
            index++;
            direction = UP_SCAN;
            data_up_time = time;
          }else{
            position -= ticks;
            SetPosition(servo_id, position);
          }
        } // end move head  
      }
    }    
  }
}

/* Sets pointer to readings, time read began.
   returns the direction of scanning */
int PML::getScanID(){
  if(direction == UP_SCAN){
    // return the down scan
    scan_time = millis() - data_dn_time;
    return DN_SCAN;
  }else{
    scan_time = millis() - data_up_time;
    return UP_SCAN;
  }
}

