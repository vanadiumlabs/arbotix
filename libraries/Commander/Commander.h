/*
  Commander.h - Library for interfacing with arbotiX Commander
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

#ifndef Commander_h
#define Commander_h

/* bitmasks for buttons array */
#define WALK_1      0x01
#define WALK_2      0x02
#define WALK_3      0x04
#define LOOK_4      0x08
#define LOOK_5      0x10
#define LOOK_6      0x20
#define WALK_TOP    0x40
#define LOOK_TOP    0x80

/* the Commander will send out a frame at about 30hz, this class helps decipher the output. */
class Commander
{    
  public:
	Commander(); 
    int ReadMsgs();         // must be called regularly to clean out Serial buffer

    // joystick values are -125 to 125
    signed char walkV;      // vertical stick movement = forward speed
    signed char walkH;      // horizontal stick movement = sideways or angular speed
    signed char lookV;      // vertical stick movement = tilt    
    signed char lookH;      // horizontal stick movement = pan (when we run out of pan, turn body?)
    
    // buttons are 0 or 1 (PRESSED), and bitmapped
    unsigned char buttons;  // 
    unsigned char extra;    //

    // specialty helpers
    int walkZ;              // accumulated Z-offset, walk2=raise, walk3=lower
        
    // Hooks are used as callbacks for button presses -- NOT IMPLEMENT YET
        
  private:
    // internal variables used for reading messages
    unsigned char vals[7];  // temporary values, moved after we confirm checksum
    int index;              // -1 = waiting for new packet
    int checksum;
};

#endif
