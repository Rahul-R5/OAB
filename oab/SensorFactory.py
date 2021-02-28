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
    def getSensor(**kwargs):
        if SensorFactory.mapInstance is None:
            SensorFactory.mapInstance = MapInfo.getMap()
            SensorFactory.seed = SensorFactory.mapInstance.seed
        
        keys = kwargs.keys()
        
        if "origin" in keys:
            origin = kwargs["origin"]
        else:
            origin = SensorFactory.getRandomPos()
            
        if "start_ang" in keys:
            start_ang = kwargs["start_ang"]
        else:
            start_ang = SensorFactory.getRandomAngle()
        
        if "ang_range" in keys:
            ang_range = SensorFactory.getRadians(kwargs["ang_range"])
        else:
            ang_range = SensorFactory.getRadians(90)
        
        if "sensor_range" in keys:
            sensor_range = kwargs["sensor_range"]
        else:
            sensor_range = 20
        
        if "num_rays" in keys:
            num_rays = kwargs["num_rays"]
        else:
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