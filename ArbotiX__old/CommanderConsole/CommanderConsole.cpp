

#include <Arduino.h>
#include "CommanderConsole.h"

/* Constructor */
CommanderConsole::CommanderConsole(){
    index = -1;
    status = 0;
}
void CommanderConsole::begin(unsigned long baud){
    Serial.begin(baud);
}



/* process messages coming from CommanderConsole 
 *  format = 0xFF X_AXIS Y_AXIS Z_AXIS W_ANGLE W_ROT GRIPPER BUTTONS EXT CHECKSUM */
int CommanderConsole::ReadMsgs(){
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
            if(index == 9){ // packet complete
                if(checksum%256 != 255){
                    // packet error!
                    index = -1;
                    return 0;
                }else{
                        x_axis = (signed char)( (int)vals[0]-128 );
                        y_axis = (signed char)( (int)vals[1]-128 );
                        z_axis = (signed char)( (int)vals[2]-128 );
                        w_angle = (signed char)( (int)vals[3]-128 );
						w_rot = (signed char)( (int)vals[4]-128 );
						gripper = (signed char)( (int)vals[5]-128 );
                    buttons = vals[6];
                    ext = vals[7];
                }
                index = -1;
                Serial.flush();
                return 1;
            }
        }
    }
    return 0;
}
