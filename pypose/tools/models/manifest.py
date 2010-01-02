#!/usr/bin/env python

""" Manifest of IK Models """

class IkModel:
    def __init__(self, folder, legoptions = [4,6]):
        self.folder = folder
        self.legoptions = legoptions

iKmodels = dict()
iKmodels["Lizard 3DOF"] = IkModel("lizard3")
#iKmodels["Mammal 3DOF"] = IkModel("mammal3", [4,])
#iKmodels["Linear-Lift + 2DOF"] = IkModel("linear2")

