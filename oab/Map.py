#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 10:49:55 2020

@author: rahulr
"""
import math

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

from oab.shape.ShapeFactory import ShapeFactory
from oab.shape.Shape import Shape
from oab.Sensor import Sensor
from oab.SensorFactory import SensorFactory

class Map():
    
    def __init__(self, resolution, grid, seed):
        self.resolution = resolution
        self.rows = grid[0]
        self.columns = grid[1]
        self.seed = seed
        self.obstacles = []
        self.robot = []
        
    
    def addObstacles(self, num):
        if self.obstacles is not None:
            self.obstacles += ShapeFactory.getShapes(num)
        else:
            self.obstacles = ShapeFactory.getShapes(num)
            
    def addRobot(self):
        if self.robot is not None:
            self.robot.append(SensorFactory.getSensor())
        else:
            self.robot = SensorFactory.getSensor()
            
    def getIntersections(self, param_1, param_2):
        intersections = []
        if (not isinstance(param_1, list))  and isinstance(param_1, type(Shape)) and isinstance(param_2, type(Sensor)):
            intersections += self.getSingleSensorIntersections(param_1, param_2)
            
        elif isinstance(param_1, list) and isinstance(param_1[0], Shape) and isinstance(param_2, Sensor):
            for obstacle in param_1:
                intersections += self.getSingleSensorIntersections(obstacle, param_2)
                
        elif (not isinstance(param_1, list)) and isinstance(param_1[0], type(Shape)) and (not isinstance(param_2, list)) and isinstance(param_2[0], type(Shape)):
            intersections += self.getSingleShapeIntersections(param_1, param_2)
             
        elif isinstance(param_1, list) and isinstance(param_1[0], type(Shape)) and isinstance(param_2, list) and isinstance(param_2[0], type(Shape)):
            for obstacle_1 in param_1:
                for obstacle_2 in param_2:
                    intersections += self.getSingleShapeIntersections(obstacle_1, obstacle_2)
        
        return intersections
    
    def getSingleSensorIntersections(self, obj_1, obj_2):        
        intersections = []
        obstacle_pts = obj_1.get_coords()
        obstacle_origin  = obj_1.origin
        sensor_pts = obj_2.get_pts()
        sensor_orgin = obj_2.origin
        
        # Loop through all edges of an obstacle and check if particular line intersects it,
        # figure out which intersections are true
        
        for j in range(len(sensor_pts)):
            temp_pts = []
            for i in range(len(obstacle_pts) - 1):               
                [flag, point] = self._getLineIntersection(obstacle_pts[i], obstacle_pts[i+1], 
                                                          sensor_pts[j], sensor_orgin)
                if flag:
                    temp_pts.append(point)
            
            # Eliminating intersections through obstacles through L2 norm
            min_d = 10000000000
            pos = -1
            for i in range(len(temp_pts)):
                point = temp_pts[i]
                d = self._getL2Distance(point, sensor_orgin)
                
                if d <= min_d:
                    min_d = d
                    pos = i
            
            if pos >= 0:
                intersections.append(temp_pts[pos])
                
        return intersections 
    
    def getSingleShapeIntersections(self, obj_1, obj_2):
        raise NotImplementedError
            
                    
    def _getLineIntersection(self, A, B, C, D):   
        [M1, C1] = self._getLineParam(A, B)         
        [M2, C2] = self._getLineParam(C, D)         
        
        if M1 == M2:
            return [False, None]
        else:
            X = (C2 - C1)/(M1 - M2)
            Y = M1 * X + C1
            min_1 = np.min(A[0])
            max_1 = np.max(B[0])
            min_2 = np.min(C[0])
            max_2 = np.max(D[0])
        if min_1 <= X <= max_1 and min_2 <= X <= max_2:
            return [True, [X, Y]]
        else:            
            return [False, None]
            
    def _getLineParam(self, A, B):
        M = (B[1] - A[1])/ (B[0] - A[0])
        C = A[1] - (M * A[0])
        return [M, C]
    
    def _getL2Distance(self, point1, point2):
        return math.sqrt((point2[0] - point1[0]) * (point2[0] - point1[0]) + (point2[1] - point1[1]) * (point2[1] - point1[1]))
    
    def drawState(self, ax):
        ax = self.drawManyObstacles(ax)
        ax = self._drawRobot(ax)
        return ax
        #plt.show()
        
    def drawManyObstacles(self, ax):
        obstacles = self.obstacles
        for obstacle in obstacles:
            vertices = obstacle.get_coords()
            ax = self._drawObstacle(ax, vertices)
        return ax
            
    def _drawObstacle(self, ax, points):        
        patches = []

        polygon = Polygon(points, closed = True)
        patches.append(polygon)
    
        xmax = self.rows
        ymax = self.columns
        ax = plt.gca()
        ax.add_patch(polygon)
        ax.set_xlim(-10,xmax)
        ax.set_ylim(-10,ymax)
        
        return ax
    
    def _drawRobot(self, ax):
        sensor = self.robot[0]
        origin = sensor.origin
        points = sensor.get_pts()
        for point in points:
            ax.plot([origin[0], point[0]], [origin[1], point[1]]) 
        
        return ax
    def moveRobot(self):
        raise NotImplementedError