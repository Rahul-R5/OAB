#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 19 10:10:15 2020

@author: rahulr
"""
import time
import matplotlib.pyplot as plt
from oab.Map import Map
from oab.MapInfo import MapInfo

class SensorObstacleIntersectionTest():
    
    def run(self):
        num = 5
        mapInstance = Map(0.5, (50,50), 9)
        MapInfo.setMap(mapInstance)
        mapInstance.addObstacles(num)
        mapInstance.addRobot()
        itr = 10
        reso = (2 * 3.14/itr) + 0.25
        sensor = mapInstance.robot[0]

        fig, ax = plt.subplots()
        for i in range(itr):             
            sensor.rotate(reso) 
            
            # Plotting the state of the map
            ax = mapInstance.drawState(ax)                   
            plt.title("SensorIntersectionsTest")
            plt.show()
            time.sleep(0.1)