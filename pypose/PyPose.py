#!/usr/bin/env python

""" PyPose: Bioloid pose system for arbotiX robocontroller

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

Pose file: 
A pose file contains all of the poses for a particular robot. Each pose
should be on a new line. The first line of the file should be the robot 
name.

pose file ex.
robot_name:servo_count
pose_name: ID1_pos+var, ID2_pos-var, ..., ID1_speed, ID2_speed, ...
pose_name: ID1_pos, ...

Positions are saved in the following way: val +/- param. 
Stored in 16-bit unsigned int:
    h-bit is +(1) or -(0) the param
    next three bits are which param to add (1-7), 0 means no param
Variables are  denoted by a,b,c,d,e,f,g

http://www.vanadiumlabs.com/ """

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


###############################################################################
# a stock dialog that we can extend and use for all sorts of things
class Dialog(Toplevel):
    def __init__(self, parent, title = None, msg = None):
        Toplevel.__init__(self,parent)
        self.transient(parent)
        self.resizable(0,0)
        
        if title:
            self.title(title)
        self.parent = parent
        self.result = None
        
        body = Frame(self)
        if msg:
            self.initial_focus = self.body(body,msg)
        else:
            self.initial_focus = self.body(body)
        body.pack(padx=5, pady=5)
        
        self.buttonbox()
        self.grab_set()
        if not self.initial_focus:
            self.initial_focus = self

        self.protocol("WM_DELETE_WINDOW",self.cancel)
        self.geometry("+%d+%d" % (parent.winfo_rootx() + 50, parent.winfo_rooty() + 50))

        self.initial_focus.focus_set()
        self.wait_window(self)

    # create dialog body, override this for your uses 
    def body(self, master, msg=None):
        Label(master, text=msg).grid(row=0)
    def apply(self):
        pass
    
    # draw buttons, override this for your uses
    def buttonbox(self):
        box = Frame(self)
        Button(box, text="OK", width=10, command=self.ok, default=ACTIVE).pack(side=LEFT,padx=5, pady=5)
        Button(box, text="Cancel",width=10,command=self.cancel).pack(side=LEFT,padx=5, pady=5)
        self.bind("&lt;Return>",self.ok)
        self.bind("&lt;Escape>",self.cancel)
        box.pack()
    
    def ok(self, event=None):
        self.withdraw()
        self.update_idletasks()
        self.apply()
        self.cancel()

    def cancel(self, event=None):
        self.parent.focus_set()
        self.destroy()


###############################################################################
# A dialog for selecting new file settings
class newFileDialog(Dialog):
    def body(self, master):
        Label(master, text="Robot Name:").grid(row=0)
        Label(master, text="Servo Count:").grid(row=1)
        self.e1 = Entry(master)
        self.e2 = Entry(master)           
        self.e1.grid(row=0, column=1)
        self.e2.grid(row=1, column=1)
        return self.e1
    
    def apply(self):
        """ Creates a new robot file. """ 
        # load robot name and servo count
        self.parent.title("PyPose: " + self.e1.get())
        self.parent.robotName = self.e1.get()
        self.parent.servo_count = int(self.e2.get())    
        # clear the window, bring up enough servo sliders
        for servo in self.parent.servos:
            servo.position.set(512)
            servo.grid_remove()
        for i in range(self.parent.servo_count):
            self.parent.servos[i].grid()
        # load the poses into the GUI
        self.parent.poses = dict()
        # clear the posebox?
        self.parent.posebox.delete(0,END)
        self.parent.curpose = "" 


###############################################################################
# A dialog for selecting new pose settings
class newPoseDialog(Dialog):
    def body(self, master):
        Label(master, text="Pose Name:").grid(row=0)
        self.e1 = Entry(master)
        self.e1.grid(row=0, column=1)
        return self.e1
    
    def apply(self):
        """ Creates a new pose. """ 
        newpose = pose(self.e1.get() + ":",self.parent.servo_count)
        self.parent.poses[newpose[0]] = newpose
        self.parent.posebox.insert(END, newpose[0])


###############################################################################
# A dialog for removing the pose
class remPoseDialog(Dialog):
    def body(self, master):
        Label(master, text="Do you really want remove " + self.parent.curpose + "?").grid(row=0)
    
    def apply(self):
        """ removes the pose. """
        rempose = int(self.parent.posebox.curselection()[0])
        del self.parent.poses[self.parent.posebox.get(rempose)]
        self.parent.posebox.delete(rempose)
        self.parent.curpose = ""


###############################################################################
# A dialog for selecting port settings
class portDialog(Dialog):
    def body(self, master):
        Label(master, text="Port Name:").grid(row=0)
        Label(master, text="Baud Rate:").grid(row=1)
        self.e1 = Entry(master)
        self.e2 = Entry(master) 
        self.e2.insert(INSERT,"38400")           
        self.e1.grid(row=0, column=1)
        self.e2.grid(row=1, column=1)
        return self.e1
        
    def buttonbox(self):
        box = Frame(self)
        Button(box, text="Open", width=10, command=self.ok, default=ACTIVE).pack(side=LEFT,padx=5, pady=5)
        Button(box, text="Cancel",width=10,command=self.cancel).pack(side=LEFT,padx=5, pady=5)
        self.bind("&lt;Return>",self.ok)
        self.bind("&lt;Escape>",self.cancel)
        box.pack()

    def apply(self):
        """ Opens the port. """
        self.parent.port = arbotix.ax12(self.e1.get(),int(self.e2.get()))
        
###############################################################################
# Pose class is a list, first element is name, rest are servo positions. 
class pose(list):
    """ A class to hold a pose. """
    def __init__(self, line, length):
        line = line.rstrip()
        # now load the name, positions for this pose
        try:
            self.append(line[0:line.index(":")])
            line = line[line.index(":")+1:]
            for servo in range(length): 
                if line.find(",") > 0:
                    self.append(int(line[0:line.index(",")]))       
                else:
                    self.append(int(line[0:]))
                line = line[line.index(",")+1:] 
        # we may not have enough data, so dump it
        except:
            for i in range(length+1-len(self)):
                self.append(512)

    def __str__(self):
        data = self[0] + ": "
        for position in self[1:]:
            data = data + str(position) + ", "
        return data[0:-2]


###############################################################################
# Main editor window
class poseEditor(Tk):
    """ Implements the main window. """
    def __init__(self):
        """ Creates pose editor window. """
        Tk.__init__(self)

        # data for our bot
        self.robotName = ""
        self.curpose = ""
        self.filename = ""
        self.poses = dict() # collection of named poses
        self.servo_count = 18 # default pose size
        # data for our window
        self.servos = list() # the editors in the window
        self.port = None
        
        # setup our main window
        self.resizable(0,0)        
        self.protocol("WM_DELETE_WINDOW",sys.exit)
        self.title("PyPose")     

        # build our menu bar  
        menubar = Menu(self)
        robotmenu = Menu(menubar, tearoff=0)
        robotmenu.add_command(label="new", command=self.newFile)  # dialog with name, # of servos
        robotmenu.add_command(label="open", command=self.openFile) # open file dialog
        robotmenu.add_command(label="save", command=self.saveFile) # if name unknown, ask, otherwise save
        robotmenu.add_command(label="save as",command=self.saveFileAs) # ask for name, save
        robotmenu.add_separator()
        robotmenu.add_command(label="set min/max",command=self.minMax) # dialog to walk through min/max
        robotmenu.add_separator()
        robotmenu.add_command(label="exit",command=sys.exit)
        menubar.add_cascade(label="robot",menu=robotmenu)
        posemenu = Menu(menubar, tearoff=0)
        posemenu.add_command(label="relax",command=self.relax) # relax our servos
        posemenu.add_command(label="capture",command=self.capturePose) # get values from port
        posemenu.add_command(label="set",command=self.setPose) # put values to port as required
        posemenu.add_command(label="export to AVR",command=self.export) # save as dialog
        menubar.add_cascade(label="pose",menu=posemenu)
        configmenu = Menu(menubar, tearoff=0)
        configmenu.add_command(label="port",command=self.doPort) # dialog box: arbotix/thru, speed, port
        menubar.add_cascade(label="config",menu=configmenu)    
        self.config(menu=menubar)

        # build servo editors
        for i in range(30):
            temp = Frame(self)
            
            tempPos = IntVar()
            if (i+1)<10:
                tempName = Label(temp, text="ID 0"+str(i+1))   
            else:
                tempName = Label(temp, text="ID "+str(i+1))
            tempName.pack(side=LEFT);
            
            temp.position = Scale(temp, orient=HORIZONTAL,to=1024)
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
        #Button(self.toolbar, text="Load", width=4).pack(side=LEFT, padx=2,pady=2)
        #Button(self.toolbar, text="New", width=3).pack(side=LEFT, padx=2,pady=2)
        #Button(self.toolbar, text="Save", width=4).pack(side=LEFT, padx=2,pady=2)
        #Button(self.toolbar, text="Save As", width=6).pack(side=LEFT, padx=2,pady=2)
        Button(self.toolbar, text="relax", command=self.relax, width=4).pack(side=LEFT, padx=2,pady=2)
        Button(self.toolbar, text="capture", command=self.capturePose, width=6).pack(side=LEFT, padx=2,pady=2)
        Button(self.toolbar, text="set", command=self.setPose, width=3).pack(side=LEFT, padx=2,pady=2)
        #Button(self.toolbar, text="Port Setup", width=7).pack(side=LEFT, padx=2,pady=2)
        self.toolbar.grid(row=30,column=0,columnspan=2)

        self.posebox = Listbox(self, width=25)
        self.posebox.grid(row=1,column=2,rowspan=15,columnspan=1,sticky=W+N+S,padx=5,pady=5)
        self.posebox.bind("<ButtonRelease-1>",self.doPose)        
        self.posebar = Frame(self)
        Button(self.posebar, text="add",command=self.addPose,width=4).pack(side=LEFT, padx=2,pady=2)
        Button(self.posebar, text="remove",command=self.remPose,width=4).pack(side=LEFT, padx=2,pady=2)
        self.posebar.grid(row=30,column=2)

    ###########################################################################
    # file handling                
    def newFile(self):  
        """ Open a dialog that asks for robot name and servo count. """
        a = newFileDialog(self, "New Robot File Settings")

    def openFile(self):
        """ Loads a robot file into the GUI. """ 
        self.filename = askopenfilename()        
        robotFile = open(self.filename, "r").readlines()    
        # load robot name and servo count
        self.title("PyPose: " + robotFile[0].split(":")[0])
        self.robotName = robotFile[0].split(":")[0]
        self.servo_count = int(robotFile[0].split(":")[1])    
        # clear the window, bring up enough servo sliders
        for servo in self.servos:
            servo.position.set(512)
            servo.grid_remove()
        for i in range(self.servo_count):
            self.servos[i].grid()
        # load the poses into the GUI
        self.poses = dict()
        # clear the posebox?
        self.posebox.delete(0,END)
        for line in robotFile[1:]:   
            nextpose = pose(line.rstrip(),self.servo_count)
            self.poses[nextpose[0]] = nextpose
            self.posebox.insert(END, nextpose[0]) 
            print nextpose   
        self.curpose = ""        
    
    def saveFile(self):
        """ Save a robot file from the GUI. """
        if self.filename == "": 
            return self.saveFileAs()
        robotFile = open(self.filename, "w")
        print>>robotFile, self.robotName + ":" + str(self.servo_count)
        for pose in self.poses.values():            
            print>>robotFile, str(pose)

    def saveFileAs(self):
        self.filename = asksaveasfilename()
        self.saveFile()                

    ###########################################################################
    # Pose Manipulation
    def doPose(self, event):
        """ Save previous pose changes, load a pose into the slider boxes. """
        if self.curpose != "":
            for servo in range(self.servo_count):
                self.poses[self.curpose][servo+1] = self.servos[servo].position.get()
        newpose = int(self.posebox.curselection()[0])
        newname = self.posebox.get(newpose)
        self.curpose = newname
        for servo in range(self.servo_count):
            self.servos[servo].position.set(self.poses[newname][servo+1])

    def relax(self):
        """ Relax servos so you can pose them. """
        if self.port != None:
            for servo in range(self.servo_count):
                self.port.setReg(servo+1,P_TORQUE_ENABLE, [0,])    

    def capturePose(self):
        """ Downloads the current pose from the robot to the GUI. """
        errors = "Could not read servos: "
        if self.port != None:        
            for servo in range(self.servo_count):
                pos = self.port.getReg(servo+1,P_PRESENT_POSITION_L, 2)
                if pos != -1:
                    self.servos[servo].position.set(pos[0] + (pos[1]<<8))
                else: 
                    errors = errors + str(servo+1) + ", "
            if errors != "Could not read servos: ":
                # message box with errors
                Dialog(self,"Capture Errors",errors[0:-2])

    def setPose(self):
        """ Write a pose out to the robot. """
        if self.port != None:
            curPose = list()
            for servo in range(self.servo_count):
                 pos = self.servos[servo].position.get()
                 self.port.setReg(servo+1, P_GOAL_POSITION_L, (pos%256, pos>>8))                
            #    pos = self.servos[servo].position.get()
            #    curPose.append( (servo+1, pos%256, pos>>8) )
            #self.pose.syncWrite(P_GOAL_POSITION_L, curPose)

    def addPose(self):
        """ Add a new pose. """
        if self.robotName != "":
            newPoseDialog(self, "New Pose Settings")
        else:
            Dialog(self,"Error","Please create a new robot first.")

    def remPose(self):
        """ Remove a pose. """
        if self.curpose != "":
            remPoseDialog(self,"Confirm Remove Pose")
        else:
            Dialog(self,"Error","Please select a pose first.")

    # TODO: handle cancel?
    def minMax(self):
        """ Get the min/max of each servo. """
        self.relax()
        self.mins = list()
        self.maxs = list()
        for i in range(self.servo_count):
            Dialog(self,"Capture Min/Max","Please move servo ID " + str(i+1) + " to an extreme of it's range.")
            x = 0 #self.port.getReg(i+1,P_PRESENT_POSITION_L,2)
            Dialog(self,"Capture Min/Max","Please move servo ID " + str(i+1) + " to the other extreme.")
            y = 0 #self.port.getReg(i+1,P_PRESENT_POSITION_L,2) 
            if x != -1 and y != -1:
                #x = x[0] + x[1]<<8
                #y = y[0] + y[1]<<8
                if x < y:
                    self.mins.append(x)
                    self.maxs.append(y)
                else:       
                    self.mins.append(y)
                    self.maxs.append(x)
            else:
                Dialog(self,"Capture Error","Could not read servo: " + str(i+1))
                self.mins.append(-1)
                self.maxs.append(1024)                                        

    ###########################################################################
    # Export functionality
    def export(self):        
        """ Export a pose file for use with Sanguino Library. """
        filename = asksaveasfilename()
        posefile = open(filename, "w")
        print>>posefile, "#ifndef " + self.robotName.upper() + "_POSES"
        print>>posefile, "#define " + self.robotName.upper() + "_POSES"
        print>>posefile, ""
        print>>posefile, "#include <avr/pgmspace.h>"
        print>>posefile, ""
        for pose in self.poses.values():
            print>>posefile, "PROGMEM prog_uint16_t " + pose[0] + "[] = {0x0C,",
            for x in pose[1:-1]:
                print>>posefile, str(x) + ",",
            print>>posefile, str(pose[-1]) + "};"
            print>>posefile, ""
        print>>posefile, "#endif"
        posefile.close()

    ###########################################################################
    # Port Manipulation
    def doPort(self):
        portDialog(self, "Port Settings")

if __name__ == "__main__":
    window = poseEditor()
    window.mainloop()

