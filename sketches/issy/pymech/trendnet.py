#!/usr/bin/env python

"""
trendnet.py: A python/pygame interface to the Trendnet TV-IP110W. 
Copyright 2009 Michael E Ferguson
http://www.blunderingbotics.com/

usage: trendnet.py <ip_of_cam>
"""

import sys, time, pygame, httplib
from pygame.locals import *
import base64

class trendnet:

    def __init__(self, surface, ip):
        """ Configures the class with the IP of the camera, and the surface to paint to. """
        username = "admin"
        password = "admin"
        # default log in information
        base64string = base64.encodestring('%s:%s' % (username, password))[:-1]
        
        self.surface = surface
        self.ip = ip
        self.size = (1024,768)
        self.offset = (16,16)
        
        h=httplib.HTTP(ip)
        h.putrequest('GET','/cgi/mjpg/mjpeg.cgi')
        h.putheader('Authorization', 'Basic %s' % base64string)
        h.endheaders()
        errcode,errmsg,headers=h.getreply()
        #print errcode, errmsg, headers
        self.file=h.getfile()
        self.filecount = 0
        #<if errcode is 200 go on the the next part>
  
    def update(self):
        """ This will paint an image to the surface already given in init. """
        data=self.file.readline()
        if data[0:15]=='Content-Length:':
            count=int(data[16:])
            s = self.file.read(count)
            # there is some funky stuff going on... lets make this a real jpeg    
            while s[0] != chr(0xff):
                s = s[1:]
            p=file("images/" + str(self.filecount) + ".jpg", 'wb') #'tempfile','wb')
            p.write(s)
            p.close()
            try:
                background = pygame.image.load("images/" + str(self.filecount) + ".jpg").convert()
                background = pygame.transform.scale(background, self.size)#, DestSurface = tempSurface)
                self.surface.blit(background, self.offset)
                self.filecount = self.filecount + 1
            except:
                print "file error"        
            
        
if __name__ == "__main__":
    # a simple little test of our system
    if len(sys.argv) < 2:
        print __doc__
    else:
        pygame.init()
        screen = pygame.display.set_mode((640,480),0,32)
        t = trendnet(screen,sys.argv[1])
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:  
                    sys.exit(0)
            t.update()
            pygame.display.update()
            time.sleep(.01)
