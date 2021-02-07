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
    
    # Try using **kwargs to get the sensor options
    @staticmethod
    def getSensor():
        if SensorFactory.mapInstance is None:
            SensorFactory.mapInstance = MapInfo.getMap()
            SensorFactory.seed = SensorFactory.mapInstance.seed
        
        origin = SensorFactory.getRandomPos()
        start_ang = SensorFactory.getRandomAngle()
        ang_range = SensorFactory.getRadians(90)
        sensor_range = 20
        num_rays = 10
        sensor = Sensor(origin, start_ang, ang_range, sensor_range, num_rays)
        return sensor
        
    """
    Returns a randomized initial coordinate (x,y) for a newly created sensor
    Seed set by the map instance's seed for reproducibility
    @return [posX,posY]: list of float
    """
    @staticmethod
    def getRandomPos():
        random.seed(SensorFactory.seed)
        posX = random.randint(0, SensorFactory.mapInstance.rows)
        posY = random.randint(0, SensorFactory.mapInstance.columns)   
        
        return [posX, posY]
    
    """
    Returns a randomized initial orientation (theta) between 0 and 360 for a newly created sensor
    Seed set by the map instance's seed for reproducibility
    @return angle: float(radian)
    """
    @staticmethod
    def getRandomAngle():
        random.seed(SensorFactory.seed)
        angle = random.randint(0, 359)
        angle = (angle/180) * 3.14  # Convert to radians
        
        return angle
    
    @staticmethod
    def getRadians(degrees):
        radians = (degrees/180) * 3.14  # Convert to radians
        
        return radians