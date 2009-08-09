/*
  BioloidController.h - arbotiX Library for Bioloid Pose Engine
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

#ifndef BioloidController_h
#define BioloidController_h

/* Poses are typically stored in a file called poses.h:
 * PROGMEM prog_uint16_t stand[ ]  =    {0x8C,512,512,482,542,662,362,512,512,542,482,362,662,
 *                                        30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30}; 
 * The first number = #of servos in pose (max 32). We add 0x80 if we are doing speed control, 
 *   or 0x40 if we want to define an end time... poses.h can easily be made using pose and
 *   capture in PyPose.
 *
 * details of new pose-based engine: (SUBJECT TO CHANGE)
 *   (30 frames per second update rate)
 *   we load a pose into NEXTPOSE (unless it's going in pose, i.e. its immediate)
 *     a pose is composed of end positions and servo speeds
 *   once we've adjusted NEXTPOSE to suit, we call interpolateSetup(time) to tell 
 *     
 *   the pose system interrupts at 30Hz, it updates the outputs and sends a sync
 *     write to the servos. If we've arrived at our final destination, we stop 
 *     pose system. 
 *  
 */

#include "ax12.h"

#define BIOLOID_FRAME_LENGTH      33    // 33 ms between frames
#define BIOLOID_SHIFT 3

/** Bioloid Controller Class for mega324p/644p clients. **/
class BioloidController
{
  public:
    /* YOU SHOULD NOT USE Serial1 */
	BioloidController(long baud);               // baud usually 1000000

    /* Pose Manipulation */
    void loadPose( const unsigned int * addr ); // load a named pose from FLASH  
    void readPose();                            // read a pose in from the servos  
    void writePose();                           // write a pose out to the servos
    void interpolateSetup(int time);            // calculate speeds for smooth transition
    void interpolateStep();                     // move forward one step in current interpolation  
    unsigned char interpolating;                // are we in an interpolation? 0=No, 1=Yes
 
    int getCurPose(int id);                     // get a servo value in the current pose
    int getNextPose(int id);                    // get a servo value in the next pose
    void setNextPose(int id, int pos);          // set a servo value in the next pose    
    
    /* Pose Engine */
    
    int poseSize;                               // how many servos are in this pose, used by Sync()

  private:  
    unsigned int pose[AX12_MAX_SERVOS];         // the current pose, updated by Step(), set out by Sync()
    unsigned int nextpose[AX12_MAX_SERVOS];     // the destination pose, where we put on load
    int speed[AX12_MAX_SERVOS];                 // speeds for interpolation 

    unsigned long lastframe;                    // time last frame was sent out  

};

#endif
