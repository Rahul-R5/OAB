#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 19 10:10:15 2020

@author: rahulr
"""
import time
from oab.Map import Map

class SensorObstacleIntersectionTest():
    
    def run():
        num = 5
        mapInstance = Map(0.5, (50,50), 9)
        mapInstance.addObstacles(num)
        mapInstance.addRobot()
        itr = 10
        reso = (2 * 3.14/itr) 
        for i in range(itr):            
            mapInstance.sensor.rotate(reso)
            mapInstance.drawState()
            time.sleep(0.5)