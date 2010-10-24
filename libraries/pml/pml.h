/* 
  pml.h - Planar Meta-Laser Library
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

#ifndef PML_DRIVER
#define PML_DRIVER

#include <ax12.h>

/* Definitions */
#define MAX_READINGS     30   // max readings in scan (to keep buffer size down)
#define PML_TIME_FRAME   50   // how long between movements
#define UP_SCAN 0
#define DN_SCAN 1

/* A class for the PML */
class PML
{
  public:
    PML();
    void init();
    void enable();
    void disable();
    void setServo(int id);
    void setSensor(int id){sensor_id = id;};
    void step();
    void setupStep(int step_start, int step_value, int step_count);
    int getScanID(); // returns which scan buffer is complete and should be read

    unsigned char data_up[2*MAX_READINGS];  // up count buffer
    unsigned char data_dn[2*MAX_READINGS];  // down count buffer
    unsigned int scan_time;                 // time offset to apply

  private:
    int enabled;    // scan or not to scan?
    int shutdown;   // should we shutdown as soon as possible? 
    int servo_id;   // ID of servo to use
    int sensor_id;  // ID of analog channel to use

    int start;      // position to start at
    int ticks;      // how many ticks to step
    int steps;      // how many steps to take in a scan
    int index;      // how far we are through a scan

    unsigned long time;         // last time we moved
    unsigned char direction;    // are we on up scan (0) or down scan (1)?
    int position;               // current head position

    unsigned long data_dn_time; // when the dn_scan was started
    unsigned long data_up_time; // when the up_scan was started
};

#endif
