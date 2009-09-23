#!/usr/bin/env python

""" 
PyMech Controller Ver A.0

  Copyright (c) 2009 Michael E. Ferguson.  All right reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

http://www.blunderingbotics.com/

pyMechCon is a python class for controlling an AVR-XBEE powered
mech warrior. It provides control over a mech's forward speed
and radius of rotation, as well as torso twist, and control of 
up to 4 guns, in rapid-all-fire mode or a rotating burst mode.

Configuration is put into pyMechConf.py, you can automatically 
create this file by running ConfigureJoystick.py
    
All values sent to controller are -100 to 100.
"""

import pygame, time, sys, serial, threading
from pygame.locals import *
from pyMechConf import *

# register definitions in AVR
REG_FORWARD = 16
REG_TURN = 17   
REG_PAN = 18
REG_TILT = 19
REG_FIRE = 20
REG_STATUS = 32

class pyMechCon:
    def __init__(self, ser=None, port="/dev/ttyUSB1", baud=38400):  
        # parameters we send
        self.forward = 0            # forward speed
        self.turn = 0               # turn speed
        self.pan = 0                # pan servo position
        self.tilt = 0               # tilt servo position
        self.fire = 0               # which guns are firing
        
        self.adjust = 0             # how much to turn
        self.TURRET_ACTIVE = False  # do we move torso or body?
        self.exitcond = False

        if ser != None:
            self.ser = ser
        else:
            try:
                self.ser = serial.Serial()
                self.ser.baudrate = baud
                self.ser.port = port
                self.ser.timeout = 3
                self.ser.open()    
                print "pyMechCon: Port Opened" 
            except:
                print "pyMechCon: Cannot open port"
                sys.exit(0)
        # we want a joystick (we should also have an on screen way to control the mech?) 
        self.stick = pygame.joystick.Joystick(0)
        self.stick.init()

    def sendPacket(self, reg, data):
        self.ser.write('\xFF')  
        self.ser.write(chr(reg))
        self.ser.write(chr(1))
        self.ser.write(chr(data + 128))
        self.ser.write(chr(255 - ((data + 129 + reg)%256))) 

    def sendFullPacket(self):
        self.ser.write('\xFF')  
        self.ser.write(chr(REG_FORWARD))
        self.ser.write(chr(5))
        self.ser.write(chr(self.forward + 128))
        self.ser.write(chr(self.turn + 128))
        self.ser.write(chr(self.pan + 128))
        self.ser.write(chr(self.tilt + 128))
        self.ser.write(chr(self.fire + 128))
        self.ser.write(chr(255 - ((self.forward + self.turn + self.pan + self. tilt + self.fire + 128 + 5 + REG_FORWARD)%256))) 
        return "pyMechCon: Status = " , str(int(self.forward)) , str(int(self.turn))# , str(int(self.servo))
            
    def thread(self):  
        """ This is the seperate thread, for handling joystick control """
        while not self.exitcond:
            for event in pygame.event.get():
                self.handle(event)
            # now send data!      
            self.sendFullPacket()
            time.sleep(0.033333) 

    def handle(self, event):
        """ Actually do something with the event. """
        if event.type == pygame.QUIT: 
            # send quit to serial thread?     
            self.exitcond = True               
            sys.exit()
        elif event.type == JOYAXISMOTION:
            if event.axis == FORWARD_AXIS:
                # user has pushed forward on joystick
                if self.TURRET_ACTIVE:
                    self.tilt = int(-50 * event.value) # was only 100s                 
                else:
                    self.forward = int(-50 * event.value) #50 for final # was 100            
            elif event.axis == TURN_AXIS:
                # joystick has moved to the side
                if self.TURRET_ACTIVE:
                    self.pan = int(100 * event.value)   
                else:   
                    if self.forward > 25:
                        self.turn = int(self.adjust * event.value * 0.25)
                    if self.forward >= -10:                            
                        self.turn = int(self.adjust * event.value)
                    else:
                        self.turn = -int(self.adjust * event.value * 0.25) 
                    #print self.turn
            elif event.axis == ADJUST_AXIS: 
                self.adjust = -15 * (event.value - 1)   
                print "Adjustment value: " + str(self.adjust)
        elif event.type == JOYBUTTONDOWN:
            if event.button == BUTTON_SERVO:
                self.TURRET_ACTIVE = True 
            elif event.button == BUTTON_SERVO_CENTER:
                self.pan = 0
                self.tilt = 0
            elif event.button == BUTTON_SERVO_LEFT:
                self.pan = self.pan + 3                          
            elif event.button == BUTTON_SERVO_RIGHT:
                self.pan = self.pan - 3
            elif event.button == BUTTON_STAND:
                print "stand"
                self.sendPacket(REG_STATUS, 1) #-127)
            elif event.button == BUTTON_RELAX:
                print "relax"
                self.sendPacket(REG_STATUS, -1) #-128)
            elif event.button == BUTTON_FIRE:   
                print "fire"
                self.fire = 1
        elif event.type == JOYBUTTONUP: 
            if event.button == BUTTON_SERVO:     
                self.TURRET_ACTIVE = False
                self.pan = 0 # do we want to do this?
                #self.tilt = 0
            elif event.button == BUTTON_FIRE:   
                print "end fire"
                self.fire = 0
        
if __name__ == "__main__":
    pygame.init()
    pyCon = pyMechCon()
    t = threading.Thread(target=pyCon.thread)
    t.start()
    try:    
        while True:
            while pyCon.ser.inWaiting > 0: 
                s = pyCon.ser.readline()
                print s,
            time.sleep(0.01)
    except KeyboardInterrupt:
        sys.exit(0)

