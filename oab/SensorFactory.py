#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 19 10:20:00 2020

@author: rahulr
"""
import random
from oab.Sensor import Sensor
from oab.MapInfo import MapInfo

class SensorFactory():
    
    seed = 8
    mapInstance = None
    
    @staticmethod
    def getSensor():
        if SensorFactory.mapInstance is None:
            SensorFactory.mapInstance = MapInfo.mapInstance
            SensorFactory.seed = SensorFactory.mapInstance.seed
        
        origin = SensorFactory.getRandomPos()
        start_ang = SensorFactory.getRandomAngle()
        ang_range = SensorFactory.getRadians(90)
        
        sensor = Sensor(origin, start_ang, ang_range, 30, 10)
        return sensor
        
    
    @staticmethod
    def getRandomPos():
        posX = random.randint(0, SensorFactory.mapInstance.rows)
        posY = random.randint(0, SensorFactory.mapInstance.columns)   
        
        #return [posX, posY]
        return [20,20]
    
    @staticmethod
    def getRandomAngle():
        angle = random.randint(0, 359)
        angle = (angle/180) * 3.14  # Convert to radians
        
        return angle
    
    @staticmethod
    def getRadians(degrees):
        radians = (degrees/180) * 3.14  # Convert to radians
        
        return radians