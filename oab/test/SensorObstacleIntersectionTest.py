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
        itr = 3
        reso = (2 * 3.14/itr) 
        sensor = mapInstance.robot[0]
        obstacles = mapInstance.obstacles
        xPoints = []
        yPoints = []
        fig, ax = plt.subplots()
        for i in range(itr):             
            sensor.rotate(reso)
            intersections = mapInstance.getIntersections(obstacles, sensor)
            ax = mapInstance.drawState(ax)
            if len(intersections) > 0:
                for i in range(len(intersections)):
                    xPoints.append(intersections[i][0])
                    yPoints.append(intersections[i][1])
                ax.plot(xPoints, yPoints, 'r*')
            plt.title("SensorIntersectionsTest")
            plt.show()
            time.sleep(0.5)