#!/usr/bin/env python

""" 
  PyPose: Bioloid pose system for arbotiX robocontroller

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
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
"""

import sys, time
from Tkinter import *
from tkFileDialog import askopenfilename, asksaveasfilename

from ax12 import *
# drivers expose init(port, baud)
#                setReg(id, start_addr, vals)
#                getReg(id, start_addr, length)
#                syncWrite(regstart, ((id1, val1, val2..), (id2, val1, val2...), ..) )
import arbotix
import direct

from dialog import *
from robot import *

VERSION = "PyPose V0.91"

###############################################################################
# Main editor window
class editor(Tk):
    """ Implements the main window. """
    def __init__(self):
        """ Creates pose editor window. """
        Tk.__init__(self)     

        # data for our program
        self.seqPane = seqEditor(self)  
        self.posePane = poseEditor(self)
        self.robot = robot()
        self.port = None    #TODO???
        self.filename = ""
        
        # setup our main window
        self.resizable(0,0)        
        self.protocol("WM_DELETE_WINDOW",sys.exit)
        self.title(VERSION)     

        # build our menu bar  
        menubar = Menu(self)
        robotmenu = Menu(menubar, tearoff=0)
        robotmenu.add_command(label="new", command=self.newFile)  # dialog with name, # of servos
        robotmenu.add_command(label="open", command=self.openFile) # open file dialog
        robotmenu.add_command(label="save", command=self.saveFile) # if name unknown, ask, otherwise save
        robotmenu.add_command(label="save as",command=self.saveFileAs) # ask for name, save
        #robotmenu.add_separator()
        #robotmenu.add_command(label="set min/max",command=self.minMax) # dialog to walk through min/max
        robotmenu.add_separator()
        robotmenu.add_command(label="exit",command=sys.exit)
        menubar.add_cascade(label="robot",menu=robotmenu)
        
        posemenu = Menu(menubar, tearoff=0)
        posemenu.add_command(label="relax",command=self.relax) # relax our servos
        posemenu.add_command(label="capture",command=self.posePane.capturePose) # get values from port
        posemenu.add_command(label="set",command=self.posePane.setPose) # put values to port as required
        posemenu.add_command(label="export to AVR",command=self.export) # save as dialog
        menubar.add_cascade(label="pose",menu=posemenu)

        toolsmenu = Menu(menubar, tearoff=0)
        toolsmenu.add_command(label="Pose Editor",command=self.loadPosePane) # do pose creation
        toolsmenu.add_command(label="Sequence Editor",command=self.loadSeqPane) # do sequencing
        toolsmenu.add_command(label="NUKE")#,command=self.seq.deiconify) # do IK
        menubar.add_cascade(label="tools",menu=toolsmenu)

        configmenu = Menu(menubar, tearoff=0)
        configmenu.add_command(label="port",command=self.doPort) # dialog box: arbotix/thru, speed, port
        menubar.add_cascade(label="config",menu=configmenu)    
        self.config(menu=menubar)

        # editor area
        #self.editorPane = Frame(self)
        self.loadPosePane()

    ###########################################################################
    # pane handling      
    def loadPosePane(self):
        self.seqPane.grid_remove()
        self.posePane.grid()

    def loadSeqPane(self):
        self.posePane.grid_remove()
        self.seqPane.grid()
        
    ###########################################################################
    # file handling                
    def newFile(self):  
        """ Open a dialog that asks for robot name and servo count. """
        n = newFileDialog(self, "New Robot File Settings")  
        if n.result == None:    
            return
        self.robot.new(n.result[0], int(n.result[1])) 
        self.title(VERSION+": " + n.result[0])   
        # reset editors
        self.posePane.reset() 
        self.seqPane.reset()       

    def openFile(self):
        """ Loads a robot file into the GUI. """ 
        self.filename = askopenfilename()  
        self.robot.load(self.filename)    
        self.title(VERSION+": " + self.robot.name)
        # reset editors
        self.posePane.reset()
        self.seqPane.reset()

    def saveFile(self):
        """ Save a robot file from the GUI. """
        if self.filename == "": 
            self.filename = asksaveasfilename()
        self.robot.save(self.filename)

    def saveFileAs(self):
        self.filename = asksaveasfilename()
        self.saveFile()                

    ###########################################################################
    # Export functionality
    def export(self):        
        """ Export a pose file for use with Sanguino Library. """
        filename = asksaveasfilename()
        self.robot.export(filename)

    ###########################################################################
    # Port Manipulation
    def doPort(self):
        portDialog(self, "Port Settings")

    def relax(self):
        """ Relax servos so you can pose them. """
        if self.port != None:
            for servo in range(self.robot.count):
                self.port.setReg(servo+1,P_TORQUE_ENABLE, [0,])    


###############################################################################
# pose editor window
class poseEditor(Frame):
    def __init__(self, parent):
        Frame.__init__(self,parent)
        self.parent = parent  
        self.curpose = ""    

        # build servo editors
        self.servos = list() # the editors in the window
        for i in range(30):
            temp = Frame(self)
            
            if (i+1)<10:
                tempName = Label(temp, text="ID 0"+str(i+1))   
            else:
                tempName = Label(temp, text="ID "+str(i+1))
            tempName.pack(side=LEFT);
            
            temp.position = Scale(temp, orient=HORIZONTAL,to=1023,length=150)    
            temp.position.pack(side=LEFT);
            if i%2 == 0:
                temp.grid(row=i/2 + 1,column=0)
            else:
                temp.grid(row=i/2 + 1,column=1,sticky=E)
            self.servos.append(temp)

        # only show 18 servo boxes to start
        for i in self.servos:
            i.grid_remove()
        for i in range(18):
            self.servos[i].grid()
        
        self.toolbar = Frame(self)
        Button(self.toolbar, text="relax", command=self.parent.relax, width=4).pack(side=LEFT, padx=2,pady=2)
        Button(self.toolbar, text="capture", command=self.capturePose, width=6).pack(side=LEFT, padx=2,pady=2)
        Button(self.toolbar, text="set", command=self.setPose, width=3).pack(side=LEFT, padx=2,pady=2)
        self.toolbar.grid(row=30,column=0,columnspan=2)

        #Label(self, text="Poses:").grid(row=0,column=2,sticky=W+N,padx=5,pady=5)
        self.posebox = Listbox(self, width=25)
        self.posebox.grid(row=1,column=2,rowspan=15,columnspan=1,sticky=W+N+S,padx=5) #,pady=5)
        self.posebox.bind("<ButtonRelease-1>",self.doPose)        
        self.posebar = Frame(self)
        Button(self.posebar, text="add",command=self.addPose,width=4).pack(side=LEFT, padx=2,pady=2)
        Button(self.posebar, text="remove",command=self.remPose,width=4).pack(side=LEFT, padx=2,pady=2)
        self.posebar.grid(row=30,column=2)

    def reset(self):
        """ Reset the editor, load correct info. """
        # clear the window, bring up enough servo sliders
        for servo in self.servos:
            servo.position.set(512)
            servo.grid_remove()
        for i in range(self.parent.robot.count):
            self.servos[i].grid()
        # clear the posebox?
        self.posebox.delete(0,END)
        for p in self.parent.robot.poses.keys():
            self.posebox.insert(END, p)
        self.curpose = ""      

    ###########################################################################
    # Pose Manipulation
    def doPose(self, event):
        """ Save previous pose changes, load a pose into the slider boxes. """
        if self.curpose != "":
            for servo in range(self.parent.robot.count):
                self.parent.robot.poses[self.curpose][servo] = self.servos[servo].position.get()
        newpose = int(self.posebox.curselection()[0])
        self.curpose = self.posebox.get(newpose)
        for servo in range(self.parent.robot.count):
            self.servos[servo].position.set(self.parent.robot.poses[self.curpose][servo])
        
    def capturePose(self):
        """ Downloads the current pose from the robot to the GUI. """
        errors = "Could not read servos: "
        if self.parent.port != None:        
            for servo in range(self.parent.robot.count):
                pos = self.parent.port.getReg(servo+1,P_PRESENT_POSITION_L, 2)
                if pos != -1:
                    self.servos[servo].position.set(pos[0] + (pos[1]<<8))
                else: 
                    errors = errors + str(servo+1) + ", "
            if errors != "Could not read servos: ":
                # message box with errors
                Dialog(self.parent,"Capture Errors",errors[0:-2])

    def setPose(self):
        """ Write a pose out to the robot. """
        if self.parent.port != None:
            curPose = list()
            for servo in range(self.parent.robot.count):
                 pos = self.servos[servo].position.get()
                 self.port.setReg(servo+1, P_GOAL_POSITION_L, (pos%256, pos>>8))                
            #    pos = self.servos[servo].position.get()
            #    curPose.append( (servo+1, pos%256, pos>>8) )
            #self.pose.syncWrite(P_GOAL_POSITION_L, curPose)

    def addPose(self):
        """ Add a new pose. """
        if self.parent.robot.name != "":
            r = newPoseDialog(self.parent, "New Pose Settings")
            if r.result != None:
                self.parent.robot.poses[r.result] = pose("", self.parent.robot.count)
                self.posebox.insert(END, r.result)
        else:
            Dialog(self.parent,"Error","Please create a new robot first.")

    def remPose(self):
        """ Remove a pose. """
        if self.curpose != "":
            r = remPoseDialog(self,"Confirm Remove Pose")
            if r.result != None:
                rempose = int(self.posebox.curselection()[0])
                del self.parent.robot.poses[self.posebox.get(rempose)]
                self.posebox.delete(rempose)
                self.curpose = ""
        else:
            Dialog(self.parent,"Error","Please select a pose first.")    


###############################################################################
# Sequence editor window
class seqEditor(Frame):
    def __init__(self, parent):
        Frame.__init__(self,parent)
        self.parent = parent        

        # listbox for sequence
        Label(self, text="transitions:").grid(row=0,column=0,sticky=W+N,padx=5,pady=5)
        self.tranbox = Listbox(self, width=25)
        self.tranbox.grid(row=1,column=0,rowspan=15,sticky=E+N+S,padx=5,pady=5)

        # transition editor 
        Label(self, text="edit transition:").grid(row=1,column=1,columnspan=2,sticky=W+N,padx=5,pady=5)
        Label(self, text="pose:").grid(row=2,column=1,padx=5,pady=5)
        self.name = Entry(self)
        self.name.grid(row=2,column=2,sticky=E+N,padx=5,pady=5)
        self.time = 0
        Label(self, text="delta-T:").grid(row=3,column=1,padx=5,pady=5)
        self.timeE = Entry(self)
        self.timeE.insert(str(self.time),0) #.delete(0,END)
        self.timeE.grid(row=3,column=2,sticky=E+N,padx=5,pady=5)                
        Button(self, text="move up", command=self.moveUp).grid(row=4,column=1,padx=5,pady=5)
        Button(self, text="move down", command=self.moveDn).grid(row=4,column=2,padx=5,pady=5)
        
        self.toolbar = Frame(self)
        Button(self.toolbar, text="relax", command=self.parent.relax, width=4).pack(side=LEFT, padx=2,pady=2)
        Button(self.toolbar, text="run", command=self.runSeq, width=6).pack(side=LEFT, padx=2,pady=2)
        Button(self.toolbar, text="halt", command=self.haltSeq, width=3).pack(side=LEFT, padx=2,pady=2)
        self.toolbar.grid(row=30,column=0,columnspan=2)

        Label(self, text="sequences:").grid(row=0,column=3,sticky=W+N,padx=5,pady=5)
        self.seqbox = Listbox(self, width=25)
        self.seqbox.grid(row=1,column=3,rowspan=15,columnspan=1,sticky=W+N+S,padx=5,pady=5)
        #self.seqbox.bind("<ButtonRelease-1>",self.doSeq)        
        self.seqbar = Frame(self)
        Button(self.seqbar, text="add",command=self.addSeq,width=4).pack(side=LEFT, padx=2,pady=2)
        Button(self.seqbar, text="remove",command=self.remSeq,width=4).pack(side=LEFT, padx=2,pady=2)
        self.seqbar.grid(row=30,column=2)

    def reset(self):
        pass

    def addSeq(self):   
        pass
    def remSeq(self):
        pass

    def runSeq(self):
        pass
    def haltSeq(self):
        pass

    def doSeq(self):
        pass

    def moveUp(self):
        pass
    def moveDn(self):
        pass

if __name__ == "__main__":
    print "PyPose starting... "
    window = editor()
    window.mainloop()

