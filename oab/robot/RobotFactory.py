# -*- coding: utf-8 -*-
"""
Created on Sat Jan  2 12:44:20 2021

@author: Rahul R
"""

import random
from oab.robot.Robot import Robot
from oab.MapInfo import MapInfo

class RobotFactory():
    
    seed = 8
    mapInstance = None
    
    @staticmethod
    def getRobot():
        if RobotFactory.mapInstance is None:
            RobotFactory.mapInstance = MapInfo.getMap()
            RobotFactory.seed = RobotFactory.mapInstance.seed
        
        origin = RobotFactory.getRandomPos()
        start_ang = RobotFactory.getRandomAngle()
                
        robot = Robot(origin, start_ang)        
        robot.add_sensor()
        return robot
    
    @staticmethod
    def getRobot(options = []):
        if RobotFactory.mapInstance is None:
            RobotFactory.mapInstance = MapInfo.getMap()
            RobotFactory.seed = RobotFactory.mapInstance.seed
        
        if len(options) == 0:
            origin = RobotFactory.getRandomPos()
            start_ang = RobotFactory.getRandomAngle()
        elif len(options) == 2:
            origin = options[0]
            start_ang = options[1]
                
        robot = Robot(origin, start_ang)        
        robot.add_sensor()
        return robot
        
    
    @staticmethod
    def getRandomPos():
        random.seed(RobotFactory.seed-1)
        posX = random.randint(0, RobotFactory.mapInstance.rows)
        posY = random.randint(0, RobotFactory.mapInstance.columns)   
        
        return [posX, posY]
    
    @staticmethod
    def getRandomAngle():
        random.seed(RobotFactory.seed-1)
        angle = random.randint(0, 359)
        angle = (angle/180) * 3.14  # Convert to radians
        
        return angle
    
    @staticmethod
    def getRadians(degrees):
        radians = (degrees/180) * 3.14  # Convert to radians
        
        return radians