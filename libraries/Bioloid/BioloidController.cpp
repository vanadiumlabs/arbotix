/*
  BioloidController.cpp - ArbotiX Library for Bioloid Pose Engine
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

#include "BioloidController.h"

/* initializes serial1 transmit at baud, 8-N-1 */
BioloidController::BioloidController(long baud){
    int i;
    // setup storage
    id_ = (unsigned char *) malloc(AX12_MAX_SERVOS * sizeof(unsigned char));
    pose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
    nextpose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
    speed_ = (int *) malloc(AX12_MAX_SERVOS * sizeof(int));
    // initialize
    for(i=0;i<AX12_MAX_SERVOS;i++){
        id_[i] = i+1;
        pose_[i] = 512;
        nextpose_[i] = 512;
    }
    interpolating = 0;
    playing = 0;
    lastframe_ = millis();
    ax12Init(baud);  
}

/* new-style setup */
void BioloidController::setup(int servo_cnt){
    int i;
    // setup storage
    id_ = (unsigned char *) malloc(servo_cnt * sizeof(unsigned char));
    pose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
    nextpose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
    speed_ = (int *) malloc(servo_cnt * sizeof(int));
    // initialize
    poseSize = servo_cnt;
    for(i=0;i<poseSize;i++){
        id_[i] = i+1;
        pose_[i] = 512;
        nextpose_[i] = 512;
    }
    interpolating = 0;
    playing = 0;
    lastframe_ = millis();
}
void BioloidController::setId(int index, int id){
    id_[index] = id;
}
int BioloidController::getId(int index){
    return id_[index];
}

/* load a named pose from FLASH into nextpose. */
void BioloidController::loadPose( const unsigned int * addr ){
    int i;
    poseSize = pgm_read_word_near(addr); // number of servos in this pose
    for(i=0; i<poseSize; i++)
        nextpose_[i] = pgm_read_word_near(addr+1+i) << BIOLOID_SHIFT;
}
/* read in current servo positions to the pose. */
/* read in current servo positions to the pose. */
bool BioloidController::readPose(){
    bool errorFound = false;
    Serial.print("Pose Reads:");
    Serial.print(poseSize);
    for(int i=0;i<poseSize;i++)
    {
        
        int temp = ax12GetRegister(id_[i],AX_PRESENT_POSITION_L,2);
        if(temp < 0 || temp > 4096)
        {
            Serial.print("BAD 1:");
            Serial.print(temp);
            delay(33);
            temp = ax12GetRegister(id_[i],AX_PRESENT_POSITION_L,2); 
            
            if(temp < 0 || temp > 4096)
            {   
                Serial.print("BAD 2:");
                Serial.print(temp);
                delay(33);
                temp = ax12GetRegister(id_[i],AX_PRESENT_POSITION_L,2);
            } 
        }

        if(temp >= 0 && temp <= 4096)
        {
            pose_[i] = temp << BIOLOID_SHIFT;
            Serial.print(" ");
            Serial.print(temp);


        }
        else
        {

            Serial.print("BAD 3:");
            Serial.print(temp);

            Serial.print(" ");
            Serial.print(temp);
            
            errorFound = true;
        }

    }

            delay(25);   
            Serial.println(" ");

    return(!errorFound);





}

/* write pose out to servos using sync write. */
void BioloidController::writePose(){
    int temp;
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
    for(int i=0; i<poseSize; i++)
    {
        temp = pose_[i] >> BIOLOID_SHIFT;
        checksum += (temp&0xff) + (temp>>8) + id_[i];
        ax12write(id_[i]);
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
    lastframe_ = millis();
    // set speed each servo...
    for(i=0;i<poseSize;i++){
        if(nextpose_[i] > pose_[i]){
            speed_[i] = (nextpose_[i] - pose_[i])/frames + 1;
        }else{
            speed_[i] = (pose_[i]-nextpose_[i])/frames + 1;
        }
    }
    interpolating = 1;
}
/* interpolate our pose, this should be called at about 30Hz. */
void BioloidController::interpolateStep(){
    if(interpolating == 0) return;
    int i;
    int complete = poseSize;
    while(millis() - lastframe_ < BIOLOID_FRAME_LENGTH);
    lastframe_ = millis();
    // update each servo
    for(i=0;i<poseSize;i++){
        int diff = nextpose_[i] - pose_[i];
        if(diff == 0){
            complete--;
        }else{
            if(diff > 0){
                if(diff < speed_[i]){
                    pose_[i] = nextpose_[i];
                    complete--;
                }else
                    pose_[i] += speed_[i];
            }else{
                if((-diff) < speed_[i]){
                    pose_[i] = nextpose_[i];
                    complete--;
                }else
                    pose_[i] -= speed_[i];                
            }       
        }
    }
    if(complete <= 0) interpolating = 0;
    writePose();      
}

/* get a servo value in the current pose */
int BioloidController::getCurPose(int id){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id )
            return ((pose_[i]) >> BIOLOID_SHIFT);
    }
    return -1;
}
/* get a servo value in the next pose */
int BioloidController::getNextPose(int id){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id )
            return ((nextpose_[i]) >> BIOLOID_SHIFT);
    }
    return -1;
}
/* set a servo value in the next pose */
void BioloidController::setNextPose(int id, int pos){
    for(int i=0; i<poseSize; i++){
        if( id_[i] == id ){
            nextpose_[i] = (pos << BIOLOID_SHIFT);
            return;
        }
    }
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

