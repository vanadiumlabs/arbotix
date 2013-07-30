

#ifndef CommanderConsole_h
#define CommanderConsole_h

/* bitmasks for buttons array */
#define BUT_R1      0x01
#define BUT_R2      0x02
#define BUT_R3      0x04
#define BUT_L4      0x08
#define BUT_L5      0x10
#define BUT_L6      0x20
#define BUT_RT      0x40
#define BUT_LT      0x80

/* the CommanderConsole will send out a frame at about 30hz, this class helps decipher the output. */

class CommanderConsole
{
  public:
    CommanderConsole(); 
    void begin(unsigned long baud);
    int ReadMsgs();         // must be called regularly to clean out Serial buffer

    // joystick values are -125 to 125
    signed char x_axis;      // X Axis
    signed char y_axis;      // Y Axis
    signed char z_axis;      // Z Axis
    signed char w_angle;      // Wrist Angle
    signed char w_rot;	   // Wrist Rotate
    signed char gripper;	   // Gripper

    
    // buttons are 0 or 1 (PRESSED), and bitmapped
    unsigned char buttons;  // 
    unsigned char ext;      // Extended function set
        

        
  private:
    // internal variables used for reading messages
    unsigned char vals[9];  // temporary values, moved after we confirm checksum
    int index;              // -1 = waiting for new packet
    int checksum;
    unsigned char status; 
};

#endif
