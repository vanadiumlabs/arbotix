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

from Tkinter import *

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
        self.result = (self.e1.get(), self.e2.get())


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
        print "Opening port: " + self.e1.get() + " @ " + self.e2.get()
        self.parent.port = arbotix.ax12(self.e1.get(),int(self.e2.get()))


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
        self.result = self.e1.get()

###############################################################################
# A dialog for removing the pose
class remPoseDialog(Dialog):
    def body(self, master):
        Label(master, text="Do you really want remove " + self.parent.curpose + "?").grid(row=0)
    
    def apply(self):
        """ removes the pose. """
        self.result = True

