/* 
  Example code for the Walking Droid in the Bioloid Beginner kit 
  Copyright (c) 2009 Michael E. Ferguson.  All right reserved.

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
#include "poses.h"

#define frames 6

BioloidController bioloid = BioloidController(1000000);

int idx = 0;

void setup(){
    Serial.begin(38400);    
    pinMode(0,OUTPUT);
    delay(1000);
    
    // stand up slowly
    bioloid.loadPose(plantLeft);
    bioloid.readPose();
    bioloid.interpolateSetup(500);
    while(bioloid.interpolating > 0){
        bioloid.interpolateStep();
        delay(3);
    }
}

void loop(){
    int i;
    
    // if finished interpolating, which pose do we go to next?
    if(bioloid.interpolating == 0){
        idx++;
        if(idx > 18) while(1);   // this demo goes 3 cycles, then stops
        if(idx % frames == 0){
            bioloid.loadPose(plantLeft);
        }else if(idx % frames == 1){
            bioloid.loadPose(liftRight);   
        }else if(idx % frames == 2){
            bioloid.loadPose(swingRight);   
        }else if(idx % frames == 3){
            bioloid.loadPose(plantRight);   
        }else if(idx % frames == 4){
            bioloid.loadPose(liftLeft);
        }else if(idx % frames == 5){
            bioloid.loadPose(swingLeft);
        }
        bioloid.interpolateSetup(1000);   
    }
    
    // update joints, this should be called faster than 30hz.
    bioloid.interpolateStep();
}
