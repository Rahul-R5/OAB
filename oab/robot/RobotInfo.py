#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 22 16:48:33 2021

@author: rahulr
"""

class RobotInfo():
    
    robotInstances = []
    
    @staticmethod
    def addRobot(robot):
        RobotInfo.robotInstances.append(robot)
        
    @staticmethod
    def getRobot(index = None):
        if RobotInfo.robotInstances:
            if index is not None and index < len(RobotInfo.robotInstances):
                return RobotInfo.robotInstances[index]
            else:
                return RobotInfo.robotInstances[0]
        else:
            raise IndexError
    
    @staticmethod
    def removeRobot(index = None):
        if RobotInfo.robotInstances:
            if index is not None and index < len(RobotInfo.robotInstances):
                del RobotInfo.robotInstances[index]
            else:
                RobotInfo.robotInstances.pop()
        else:
            raise IndexError
        
        