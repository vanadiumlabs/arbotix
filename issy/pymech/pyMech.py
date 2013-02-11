#!/usr/bin/env python

"""
PyMech Ver A.0

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

PyMech is a lightweight python based system for controlling 
an AVR-XBEE powered mech warrior. It provides control over a 
mech's forward speed and radius of rotation, as well as torso 
twist, and control of up to 4 guns, in rapid-all-fire mode 
or a rotating burst mode.

usage: pymech.py <serial_port> <ip_of_cam> [other modules]
If you pass the names of other modules, they will be loaded, ex:
    pymech.py /dev/ttyUSB0 192.168.1.7 mechdar
"""

import pygame, urllib
from pygame.locals import *
import serial, sys, time, threading
from trendnet import *
import pyMechCon
#import mechdar

class pyMech:
    """ Main class to define pyMech """

    def __init__(self, port="/dev/ttyUSB0", ip='192.168.1.7', other=None):
        pygame.init()
        pygame.display.set_caption("PyMech Version A.0")
        size = 1280,800
        screen = pygame.display.set_mode(size)
        screen.fill(pygame.Color("#CCCCCC"))
        #pygame.display.toggle_fullscreen()
        self.exitcond = False
        self.vupdate = False

        #if other != None and "mechdar" in other:
        #    mapSurface = screen.subsurface((584,384),(216,216))
        #    window = mechdar.mechdar(ser=self.ser) #,mapHook=self.myHook)  
        #    t = threading.Thread(target=window.thread)
        #    t.start()
        #else:
        #    window = None

        self.control = pyMechCon.pyMechCon(None, port)
        t1 = threading.Thread(target=self.eventThread)
        t1.start()
        t2 = threading.Thread(target=self.feedbackThread)
        t2.start()
        #self.temps = [0,0,70,65,40,40,0,0,50,62,40,32]
        #self.voltage = "12.22"
        #self.expired = True
        #self.vupdate = True

        # camera
        if ip == "offline":
            t = None
        else:
            t = trendnet(screen,ip)

        # main loop
        while not self.exitcond:
            # update camera image
            if t == None:
                background = pygame.image.load('tempfile').convert()
                background = pygame.transform.scale(background, (1024,768))#, DestSurface = tempSurface)
                screen.blit(background, (16,16))
            else:
                t.update()
            # update mechdar display
            #if window != None:
                #window.map.drawMap(mapSurface)
            # print out voltage stuff
            if self.vupdate == True:
                i = 0
                font = pygame.font.Font(None, 36)
                background = pygame.Surface((200,1000))
                background.fill(pygame.Color("#CCCCCC"))
                screen.blit(background, (1050,0))
                text = font.render("Voltage " + str(self.voltage), 1, (10, 10, 10))
                textpos = text.get_rect()
                textpos.centerx = 1160
                textpos.centery = 50
                screen.blit(text, textpos)
                for v in self.temps:
                    color = (10,10,10)  
                    if v > 60:
                        color = (192,0,0)
                    text = font.render("Servo " + str(i+1) + ": " + str(v) + "C", 1, color)
                    textpos = text.get_rect()
                    textpos.centerx = 1160
                    textpos.centery = 30*i + 100
                    screen.blit(text, textpos)
                    #print str(i) + ":" +  str(v)
                    i = i +1
                # battery expired?
                if self.expired == True:
                    text = font.render("BATTERY EXPIRED", 1, (192,0,0))
                    textpos = text.get_rect()
                    textpos.centerx = 1160
                    textpos.centery = 15
                    screen.blit(text,textpos)            
                self.vupdate = False
            # update rest of window
            pygame.display.update()
            time.sleep(0.01)

    def eventThread(self):
        """ This is the seperate thread, for handling events """
        while not self.exitcond:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: 
                    # send quit to serial thread?     
                    self.exitcond = True               
                    sys.exit()
                elif event.type is KEYDOWN and pygame.key.name(event.key) is "e":
                    self.exitcond = True
                    sys.exit(0)
                else:                 
                    self.control.handle(event)
            self.control.sendFullPacket()
            time.sleep(0.033333)

    def feedbackThread(self):   
        self.temps = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.voltage = "0"
        self.expired = False
        while not self.exitcond:
            while self.control.ser.inWaiting > 0 and not self.exitcond: 
                s = self.control.ser.readline()
                try:
                    if s[0:2] == "S0" and s[4:8] == "TEMP":
                        self.temps[int(s[1:4])-1] = int(s[9:])    
                    if s[0:4] == "S012":
                        self.vupdate = True 
                    if s[0:7] == "Voltage": 
                        self.voltage = s[8:-1]
                    if s[0:15] == "Battery Expired!":   
                        self.expired = True
                        self.vupdate = True
                except:
                    print s,            
                #print s,
            time.sleep(0.01)

if __name__ == "__main__":  
    if len(sys.argv) < 3:
        print __doc__        
    else:        
        if len(sys.argv) > 3:
            mech = pyMech(sys.argv[1],sys.argv[2], sys.argv[3:])
        else:
            mech = pyMech(sys.argv[1],sys.argv[2])
            

