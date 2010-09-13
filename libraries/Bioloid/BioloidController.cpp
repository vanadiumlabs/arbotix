/*
  BioloidController.cpp - arbotiX Library for Bioloid Pose Engine
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

#include "wiring.h" // we need this for the serial port defines

#include <avr/pgmspace.h>
#include "BioloidController.h"

/* initializes serial1 transmit at baud, 8-N-1 */
BioloidController::BioloidController(long baud){
    int i;
    for(i=0;i<AX12_MAX_SERVOS;i++)
        pose[i] = 512;
    interpolating = 0;
    playing = 0;
    lastframe = millis();
    ax12Init(baud);  
}

/* load a named pose from FLASH into nextpose. */
void BioloidController::loadPose( const unsigned int * addr ){
    int i;
    poseSize = pgm_read_word_near(addr); // number of servos in this pose
    for(i=0; i<poseSize; i++)
        nextpose[i] = pgm_read_word_near(addr+1+i) << BIOLOID_SHIFT;
}
/* read in current servo positions to the pose. */
void BioloidController::readPose(){
    for(int i=0;i<poseSize;i++){
        pose[i] = ax12GetRegister(i+1,AX_PRESENT_POSITION_L,2)<<BIOLOID_SHIFT;
        delay(25);   
    }
}
/* write pose out to servos using sync write. */
void BioloidController::writePose(){
    int temp;
    int count = poseSize;
    int length = 4 + (poseSize * 3);   // 3 = id + pos(2byte)
    int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_POSITION_L;
    setTXall();
    ax12write(0xFF);
    ax12write(0xFF);
    ax12write(0xFE);
    ax12write(length);
    ax12write(AX_SYNC_WRITE);
    ax12write(AX_GOAL_POSITION_L);
    ax12write(2);
    while(count > 0){
        count--;
        temp = pose[count] >> BIOLOID_SHIFT;
        checksum += (temp&0xff) + (temp>>8) + count + 1;
        ax12write(count + 1);
        ax12write(temp&0xff);
        ax12write(temp>>8);
    } 
    ax12write(0xff - (checksum % 256));
    setRX(0);
}

/* set up for an interpolation from pose to nextpose over TIME 
    milliseconds by setting servo speeds. */
void BioloidController::interpolateSetup(int time){
    int i;
    int frames = (time/BIOLOID_FRAME_LENGTH) + 1;
    lastframe = millis();
    // set speed each servo...
    for(i=0;i<poseSize;i++){
        if(nextpose[i] > pose[i]){
            speed[i] = (nextpose[i] - pose[i])/frames + 1;
        }else{
            speed[i] = (pose[i]-nextpose[i])/frames + 1;
        }
    }
    interpolating = 1;
}
/* interpolate our pose, this should be called at about 30Hz. */
void BioloidController::interpolateStep(){
    if(interpolating == 0) return;
    int i;
    int complete = poseSize;
    while(millis() - lastframe < BIOLOID_FRAME_LENGTH);
    lastframe = millis();
    // update each servo
    for(i=0;i<poseSize;i++){
        int diff = nextpose[i] - pose[i];
        if(diff == 0){
            complete--;
        }else{
            if(diff > 0){
                if(diff < speed[i]){
                    pose[i] = nextpose[i];
                    complete--;
                }else
                    pose[i] += speed[i];
            }else{
                if((-diff) < speed[i]){
                    pose[i] = nextpose[i];
                    complete--;
                }else
                    pose[i] -= speed[i];                
            }       
        }
    }
    if(complete <= 0) interpolating = 0;
    writePose();      
}

/* get a servo value in the current pose */
int BioloidController::getCurPose(int id){
    return ((pose[id-1]) >> BIOLOID_SHIFT);
}
/* get a servo value in the next pose */
int BioloidController::getNextPose(int id){
    return ((nextpose[id-1]) >> BIOLOID_SHIFT);
}
/* set a servo value in the next pose */
void BioloidController::setNextPose(int id, int pos){
    nextpose[id-1] = (pos << BIOLOID_SHIFT);
}

/* play a sequence. */
void BioloidController::playSeq( const transition_t  * addr ){
    sequence = (transition_t *) addr;
    // number of transitions left to load
    transitions = pgm_read_word_near(&sequence->time);
    sequence++;    
    // load a transition
    loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
    interpolateSetup(pgm_read_word_near(&sequence->time));
    transitions--;
    playing = 1;
}
/* keep playing our sequence */
void BioloidController::play(){
    if(playing == 0) return;
    if(interpolating > 0){
        interpolateStep();
    }else{  // move onto next pose
        sequence++;   
        if(transitions > 0){
            loadPose((const unsigned int *)pgm_read_word_near(&sequence->pose));
            interpolateSetup(pgm_read_word_near(&sequence->time));
            transitions--;
        }else{
            playing = 0;
        }
    }
}

