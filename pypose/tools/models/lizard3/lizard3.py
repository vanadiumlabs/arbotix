#!/usr/bin/env python

""" 
  PyPose: Bioloid pose system for arbotiX robocontroller
  Copyright (c) 2008-2010 Michael E. Ferguson.  All right reserved.

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
"""

import wx
from math import cos,sin,atan2,sqrt,acos
from NukeEditor import NukeDialog

# Some preliminaries
def sq(x):  
    return x*x
# Convert radians to servo position offset.
def radToServo(rads): 
    val = (rads*100)/51 * 100;
    return int(val) 

COXA = 0
FEMUR = 1
TIBIA = 2

class lizard3(dict):
    X_COXA = 29     # MM between front and back legs /2
    Y_COXA = 29     # MM between front/back legs /2

    L_COXA = 52     # MM distance from coxa servo to femur servo 
    L_FEMUR = 47    # MM distance from femur servo to tibia servo 
    L_TIBIA = 125   # MM distance from tibia servo to foot 

    bodyRotX = 0.0
    bodyRotY = 0.0
    bodyRotZ = 0.0

    bodyPosX = 0.0
    bodyPosY = 0.0

    def setNextPose(self, servo, pos):
        self.nextPose[servo] = pos

    def __init__(self, legs=4, debug=False, gaitGen = None):
        self.legs = legs
        self.debug = debug
        self.gaitGen = gaitGen
        self["RF_COXA"] = 1
        self["RF_FEMUR"] = 3
        self["RF_TIBIA"] = 5
        self["LF_COXA"] = 2
        self["LF_FEMUR"] = 4
        self["LF_TIBIA"] = 6
        self["RR_COXA"] = 7
        self["RR_FEMUR"] = 9
        self["RR_TIBIA"] = 11
        self["LR_COXA"] = 8
        self["LR_FEMUR"] = 10
        self["LR_TIBIA"] = 12
        self["RIGHT_FRONT"] = [60,90,100]
        self["RIGHT_REAR"] = [-60,90,100]
        self["LEFT_FRONT"] = [60,-90,100]
        self["LEFT_REAR"] = [-60,-90,100]

        if self.legs > 4:
            self["RM_COXA"] = 13
            self["RM_FEMUR"] = 15
            self["RM_TIBIA"] = 17
            self["LM_COXA"] = 14
            self["LM_FEMUR"] = 16
            self["LM_TIBIA"] = 18
            self["RIGHT_MIDDLE"] = [0,90,100]
            self["LEFT_MIDDLE"] = [0,-90,100]

        self["RF_GAIT"] = [0,0,0,0]
        self["LF_GAIT"] = [0,0,0,0]
        self["RR_GAIT"] = [0,0,0,0]
        self["LR_GAIT"] = [0,0,0,0]
        self["RM_GAIT"] = [0,0,0,0]
        self["LM_GAIT"] = [0,0,0,0]

        self.order = {"RF_GAIT":0,"LR_GAIT":2,"LF_GAIT":4,"RR_GAIT":6}

        self.mins = [512 for i in range(3*legs)]
        self.maxs = [512 for i in range(3*legs)]
        self.neutrals = [512 for i in range(3*legs)]
        self.nextPose = [0 for i in range(3*legs)]
        self.signs = [1 for i in range(3*legs)]
        self.step = 0

    def bodyIK(self, X, Y, Z, Xdisp, Ydisp, Zrot):
        """ Compute offsets based on Body positions. 
          BodyIK based on the work of Xan """    
        ans = [0,0,0]   # (X,Y,Z)
    
        cosB = cos(self.bodyRotX)
        sinB = sin(self.bodyRotX)
        cosG = cos(self.bodyRotY)
        sinG = sin(self.bodyRotY)
        cosA = cos(self.bodyRotZ+Zrot)
        sinA = sin(self.bodyRotZ+Zrot)
    
        totalX = int(X + Xdisp + self.bodyPosX); 
        totalY = int(Y + Ydisp + self.bodyPosY); 
    
        ans[0] = int(totalX - int(totalX*cosG*cosA + totalY*sinB*sinG*cosA + Z*cosB*sinG*cosA - totalY*cosB*sinA + Z*sinB*sinA)) + self.bodyPosX
        ans[1] = int(totalY - int(totalX*cosG*sinA + totalY*sinB*sinG*sinA + Z*cosB*sinG*sinA + totalY*cosB*cosA - Z*sinB*cosA)) + self.bodyPosY
        ans[2] = int(Z - int(-totalX*sinG + totalY*sinB*cosG + Z*cosB*cosG))
        
        if self.debug:        
            print "BodyIK:",ans
        return ans

    def legIK(self, X, Y, Z):
        """ Compute leg servo positions. """
        ans = [0,0,0,0]    # (coxa, femur, tibia)
       
        try:
            # first, make this a 2DOF problem... by solving coxa
            ans[0] = radToServo(atan2(X,Y))
            trueX = int(sqrt(sq(X)+sq(Y))) - self.L_COXA
            im = int(sqrt(sq(trueX)+sq(Z)))  # length of imaginary leg
            
            # get femur angle above horizon...
            q1 = -atan2(Z,trueX)
            d1 = sq(self.L_FEMUR)-sq(self.L_TIBIA)+sq(im)
            d2 = 2*self.L_FEMUR*im
            q2 = acos(d1/float(d2))
            ans[1] = radToServo(q1+q2)  
        
            # and tibia angle from femur...
            d1 = sq(self.L_FEMUR)-sq(im)+sq(self.L_TIBIA)
            d2 = 2*self.L_TIBIA*self.L_FEMUR;
            ans[2] = radToServo(acos(d1/float(d2))-1.57)
        except:
            if self.debug:
                "LegIK FAILED"
            return [1024,1024,1024,0]

        if self.debug:
            print "LegIK:",ans
        return ans

    def doIK(self):
        fail = 0
        req = [0,0,0,0]     # [x,y,z,r]
        gait = [0,0,0,0]    # [x,y,z,r]
        sol = [0,0,0]       # [coxa,femur,tibia]

        # right front leg
        if self.gaitGen != None:
            gait = self.gaitGen("RF_GAIT")    
        if self.debug:
            print "RIGHT_FRONT: ", [self["RIGHT_FRONT"][i] + gait[i] for i in range(3)]
        req = self.bodyIK(self["RIGHT_FRONT"][0]+gait[0], self["RIGHT_FRONT"][1]+gait[1], self["RIGHT_FRONT"][2]+gait[2], self.X_COXA, self.Y_COXA, gait[3])
        sol = self.legIK(self["RIGHT_FRONT"][0]+req[0]+gait[0],self["RIGHT_FRONT"][1]+req[1]+gait[1],self["RIGHT_FRONT"][2]+req[2]+gait[2])
        servo = self["RF_COXA"]        
        output = self.neutrals[servo]+self.signs[servo]*sol[COXA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                print "RF_COXA FAIL: ", output
            fail = fail + 1
        servo = self["RF_FEMUR"]
        output = self.neutrals[servo]+self.signs[servo]*sol[FEMUR]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                print "RF_FEMUR FAIL: ", output
            fail = fail + 1
        servo = self["RF_TIBIA"]
        output = self.neutrals[servo]+self.signs[servo]*sol[TIBIA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:            
            if self.debug:
                print "RF_TIBIA FAIL: ",output
            fail = fail + 1
        
        # right rear leg 
        if self.gaitGen != None:
            gait = self.gaitGen("RR_GAIT")    
        if self.debug:
            print "RIGHT_REAR: ", [self["RIGHT_REAR"][i] + gait[i] for i in range(3)]
        req = self.bodyIK(self["RIGHT_REAR"][0]+gait[0],self["RIGHT_REAR"][1]+gait[1],self["RIGHT_REAR"][2]+gait[2], -self.X_COXA, self.Y_COXA, gait[3])
        sol = self.legIK(-self["RIGHT_REAR"][0]-req[0]-gait[0],self["RIGHT_REAR"][1]+req[1]+gait[1],self["RIGHT_REAR"][2]+req[2]+gait[2]);
        servo = self["RR_COXA"]
        output = self.neutrals[servo]+self.signs[servo]*sol[COXA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                print "RR_COXA FAIL: ", output
            fail = fail + 1
        servo = self["RR_FEMUR"]
        output = self.neutrals[servo]+self.signs[servo]*sol[FEMUR]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                print "RR_FEMUR FAIL:", output
            fail = fail + 1
        servo = self["RR_TIBIA"]
        output = self.neutrals[servo]+self.signs[servo]*sol[TIBIA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                print "RR_TIBIA FAIL:", output
            fail = fail + 1
        
        # left front leg
        if self.gaitGen != None:
            gait = self.gaitGen("LF_GAIT")    
        if self.debug:
            print "LEFT_FRONT: ", [self["LEFT_FRONT"][i] + gait[i] for i in range(3)]
        req = self.bodyIK(self["LEFT_FRONT"][0]+gait[0],self["LEFT_FRONT"][1]+gait[1],self["LEFT_FRONT"][2]+gait[2], self.X_COXA, -self.Y_COXA, gait[3])
        sol = self.legIK(self["LEFT_FRONT"][0]+req[0]+gait[0],-self["LEFT_FRONT"][1]-req[1]-gait[1],self["LEFT_FRONT"][2]+req[2]+gait[2]);
        servo = self["LF_COXA"]
        output = self.neutrals[servo]+self.signs[servo]*sol[COXA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                print "LF_COXA FAIL:", output
            fail = fail + 1
        servo = self["LF_FEMUR"]
        output = self.neutrals[servo]+self.signs[servo]*sol[FEMUR]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                print"LF_FEMUR FAIL:", output
            fail = fail + 1
        servo = self["LF_TIBIA"]
        output = self.neutrals[servo]+self.signs[servo]*sol[TIBIA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                print "LF_TIBIA FAIL:", output
            fail = fail + 1

        # left rear leg
        if self.gaitGen != None:
            gait = self.gaitGen("LR_GAIT")   
        if self.debug:
            print "LEFT_REAR: ", [self["LEFT_REAR"][i] + gait[i] for i in range(3)] 
        req = self.bodyIK(self["LEFT_REAR"][0]+gait[0],self["LEFT_REAR"][1]+gait[1],self["LEFT_REAR"][2]+gait[2], -self.X_COXA, -self.Y_COXA, gait[3])
        sol = self.legIK(-self["LEFT_REAR"][0]-req[0]-gait[0],-self["LEFT_REAR"][1]-req[1]-gait[1],self["LEFT_REAR"][2]+req[2]+gait[2])
        servo = self["LR_COXA"]
        output = self.neutrals[servo]+self.signs[servo]*sol[COXA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                print "LR_COXA FAIL:", output
            fail = fail + 1
        servo = self["LR_FEMUR"]
        output = self.neutrals[servo]+self.signs[servo]*sol[FEMUR]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                print "LR_FEMUR FAIL:",output
            fail = fail + 1
        servo = self["LR_TIBIA"]        
        output = self.neutrals[servo]+self.signs[servo]*sol[TIBIA]
        if output < self.maxs[servo] and output > self.mins[servo]:
            self.setNextPose(servo, output)
        else:
            if self.debug:
                print "LR_TIBIA FAIL:", output
            fail = fail + 1
    
        if self.legs > 4:
            # right middle leg
            if self.gaitGen != None:
                gait = self.gaitGen("RM_GAIT")    
            req = self.bodyIK(self["RIGHT_MID"][0]+gait[0],self["RIGHT_MID"][1]+gait[1],self["RIGHT_MID"][2]+gait[2], 0, self.Y_MID, gait[3])
            sol = self.legIK(self["RIGHT_MID"][0]+req[0]+gait[0],-self["RIGHT_MID"][1]-req[1]-gait[1],self["RIGHT_MID"][2]+req[2]+gait[2])
            servo = self["RM_COXA"]
            output = self.neutrals[servo]+self.signs[servo]*sol[COXA]
            if output < self.maxs[servo] and output > self.mins[servo]:
                self.setNextPose(servo, output)
            else:
                if self.debug:
                    print "RM_COXA FAIL:", output
                fail = fail + 1
            servo = self["RM_FEMUR"]
            output = self.neutrals[servo]+self.signs[servo]*sol[FEMUR]
            if output < self.maxs[servo] and output > self.mins[servo]:
                self.setNextPose(servo, output)
            else:
                if self.debug:
                    print"RM_FEMUR FAIL:", output
                fail = fail + 1
            servo = self["RM_TIBIA"]
            output = self.neutrals[servo]+self.signs[servo]*sol[TIBIA]
            if output < self.maxs[servo] and output > self.mins[servo]:
                self.setNextPose(servo, output)
            else:
                if self.debug:
                    print "RM_TIBIA FAIL:", output
                fail = fail + 1

            # left middle leg
            if self.gaitGen != None:
                gait = self.gaitGen("LM_GAIT")    
            req = self.bodyIK(self["LEFT_MID"][0]+gait[0],self["LEFT_MID"][1]+gait[1],self["LEFT_MID"][2]+gait[2], 0, -self.Y_MID, gait[3])
            sol = self.legIK(-self["LEFT_MID"][0]-req[0]-gait[0],-self["LEFT_MID"][1]-req[1]-gait[1],self["LEFT_MID"][2]+req[2]+gait[2])
            servo = self["LM_COXA"]
            output = self.neutrals[servo]+self.signs[servo]*sol[COXA]
            if output < self.maxs[servo] and output > self.mins[servo]:
                self.setNextPose(servo, output)
            else:
                if self.debug:
                    print "LM_COXA FAIL:", output
                fail = fail + 1
            servo = self["LM_FEMUR"]
            output = self.neutrals[servo]+self.signs[servo]*sol[FEMUR]
            if output < self.maxs[servo] and output > self.mins[servo]:
                self.setNextPose(servo, output)
            else:
                if self.debug:
                    print "LM_FEMUR FAIL:",output
                fail = fail + 1
            servo = self["LM_TIBIA"]        
            output = self.neutrals[servo]+self.signs[servo]*sol[TIBIA]
            if output < self.maxs[servo] and output > self.mins[servo]:
                self.setNextPose(servo, output)
            else:
                if self.debug:
                    print "LM_TIBIA FAIL:", output
                fail = fail + 1
       
        self.step = self.step + 1
        if self.step > 7:
            self.step = 0   #gaitStep = (gaitStep+1)%stepsInGait
        return fail

    def defaultGait(self,leg):        
        # just walk forward for now
        travelX = 50
        travelY = 0
        travelRotZ = 0

        if abs(travelX)>5 or abs(travelY)>5 or abs(travelRotZ) > 0.05:   # are we moving?
            if(self.order[leg] == self.step):
                # up, middle position                    
                self[leg][0] = 0        # x
                self[leg][1] = 0        # y
                self[leg][2] = -20      # z
                self[leg][3] = 0        # r
            elif (self.order[leg]+1 == self.step) or (self.order[leg]-7 == self.step):   # gaits in step -1 
                # leg down!                    
                self[leg][0] = travelX/2
                self[leg][1] = travelY/2
                self[leg][2] = 0       
                self[leg][3] = travelRotZ/2      
            else:
                # move body forward 
                self[leg][0] = self[leg][0] - travelX/6
                self[leg][1] = self[leg][1] - travelY/6
                self[leg][2] = 0       
                self[leg][3] = self[leg][3] - travelRotZ/6    
        return self[leg]

    def strRep(self, t):
        if t > 0:
            return "+"
        else:
            return "-"

    def doSignTest(self,parent,step=0):
        if step == 0:
            print "Moving to neutral positions"
            self["RIGHT_FRONT"] = [0,self.L_FEMUR+self.L_COXA,self.L_TIBIA]
            self["RIGHT_REAR"] = [0,self.L_FEMUR+self.L_COXA,self.L_TIBIA]
            self["LEFT_FRONT"] = [0,-self.L_FEMUR-self.L_COXA,self.L_TIBIA]
            self["LEFT_REAR"] = [0,-self.L_FEMUR-self.L_COXA,self.L_TIBIA]
            self["RIGHT_MIDDLE"] = [0,self.L_FEMUR+self.L_COXA,self.L_TIBIA]
            self["LEFT_MIDDLE"] = [0,-self.L_FEMUR-self.L_COXA,self.L_TIBIA]
            self.doIK()
            parent.writePose(self.nextPose, 500)
            dlg = wx.MessageDialog(parent, "Click OK when ready!", 'Sign Test', wx.OK)
            if dlg.ShowModal() == wx.ID_OK:    
                return self.doSignTest(parent,1)
            else:
                return "".join([self.strRep(t) for t in self.signs])   
        else:
            msg = ""            # message to display to user
            servo = -1          # servo ID to reverse if we get a NCK    
            if step == 1:
                # SET COXAS FIRST
                self["RIGHT_FRONT"][0] = self.L_COXA/2
                msg = "Did my RF leg move forward?"
                servo = "RF_COXA"
            elif step == 2:
                self["LEFT_FRONT"][0] = self.L_COXA/2
                msg = 'Did my LF leg move forward?'
                servo = "LF_COXA"
            elif step == 3:
                self["RIGHT_REAR"][0] = -self.L_COXA/2
                msg = 'Did my RR leg move backward?'
                servo = "RR_COXA"
            elif step == 4:
                self["LEFT_REAR"][0] = -self.L_COXA/2
                msg = 'Did my LR leg move backward?'
                servo = "LR_COXA"
            elif step == 5:
                # Now FEMURs and TIBIAs
                self["RIGHT_FRONT"][2] = self["RIGHT_FRONT"][2] - 20
                msg = 'Did my RF leg move upward?'
                servo = "RF_FEMUR"
            elif step == 6: 
                msg = 'Is my RF tibia still straight up and down?'
                #msg = 'Did my RF leg move inward or outward too?'
                servo = "RF_TIBIA"
            elif step == 7:
                self["LEFT_FRONT"][2] = self["LEFT_FRONT"][2] - 20
                msg = 'Did my LF leg move upward?'
                servo = "LF_FEMUR"
            elif step == 8:
                msg = 'Is my LF tibia still straight up and down?'
                #msg = 'Did my LF leg move inward or outward too?'
                servo = "LF_TIBIA"
            elif step == 9:
                self["RIGHT_REAR"][2] = self["RIGHT_REAR"][2] - 20
                msg = 'Did my RR leg move upward?'
                servo = "RR_FEMUR"
            elif step == 10:
                msg = 'Is my RR tibia still straight up and down?'
                #msg = 'Did my RR leg move inward or outward too?'
                servo = "RR_TIBIA"
            elif step == 11:
                self["LEFT_REAR"][2] = self["LEFT_REAR"][2] - 20
                msg = 'Did my LR leg move upward?'
                servo = "LR_FEMUR"
            elif step == 12:
                msg = 'Is my LR tibia still straight up and down?'
                #msg = 'Did my LR leg move inward or outward too?'
                servo = "LR_TIBIA"   
            elif step == 13:
                # middle legs
                self["RIGHT_MIDDLE"][0] = self.L_COXA/2
                msg = "Did my RM leg move forward?"
                servo = "RM_COXA"
            elif step == 14:
                self["LEFT_MIDDLE"][0] = self.L_COXA/2
                msg = "Did my LM leg move forward?"
                servo = "LM_COXA"
            elif step == 15:
                self["RIGHT_MIDDLE"][2] = self["RIGHT_MIDDLE"][2] - 20
                msg = 'Did my RM leg move upward?'
                servo = "RM_FEMUR"
            elif step == 16:
                msg = 'Is my RM tibia still straight up and down?'
                servo = "RM_TIBIA"
            elif step == 17:
                self["LEFT_MIDDLE"][2] = self["LEFT_MIDDLE"][2] - 20
                msg = 'Did my LM leg move upward?'
                servo = "LM_FEMUR"
            elif step == 18:
                msg = 'Is my LM tibia still straight up and down?'
                servo = "LM_TIBIA"  

            # do IK and display dialog
            self.doIK()
            parent.writePose(self.nextPose, 500)
            #dlg = SignDialog(parent, 'Sign Test', msg)
            dlg = wx.MessageDialog(parent, msg, 'Sign Test', wx.YES | wx.NO)
            result = dlg.ShowModal()    
            if result == wx.ID_CANCEL:
                return "".join([self.strRep(t) for t in self.signs])            
            elif result == wx.ID_NO:
                print "Reversing", servo, "sign"       
                if self.signs[self[servo]] > 0:
                    self.signs[self[servo]] = -1
                else:
                    self.signs[self[servo]] = 1
                self.doIK()
                parent.writePose(self.nextPose, 500)
            if step < (3*self.legs):
                return self.doSignTest(parent,step+1)
            else:
                return "".join([self.strRep(t) for t in self.signs])

