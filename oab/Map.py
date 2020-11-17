#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 10:49:55 2020

@author: rahulr
"""
import math
import Shape
import ShapeFactory
import Sensor

class Map():
    
    def __init__(self, resolution, grid, seed):
        self.resolution = resolution
        self.rows = grid[0]
        self.columns = grid[1]
        self.seed = seed
        
    
    def addObstacles(self, num):
        if self.obstacles is not None:
            self.obstacles += ShapeFactory.getShapes(num)
        else:
            self.obstacles = ShapeFactory.getShapes(num)
            
    def addRobot(self):
        if self.sensor is not None:
            self.sensor += Sensor()
        else:
            self.sensor = Sensor()
        
    def getShapeIntersections(self, obj_1, obj_2):
        intersections = []
        if isinstance(obj_1, type(Shape)) and isinstance(obj_2, type(Shape)):
            pass
            
        elif isinstance(obj_1, type(Shape)) and isinstance(obj_2, type(Sensor)):
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
                for i in range(temp_pts):
                    point = temp_pts[i]
                    d = self._getL2Distance(point, sensor_orgin)
                    
                    if d <= min_d:
                        min_d = d
                        pos = i
                
                if pos >= 0:
                    intersections.append(temp_pts[pos])
                
        return intersections
                    
    def _getLineIntersection(self, A, B, C, D):   
        [M1, C1] = self._getLineParam(A, B)         
        [M2, C2] = self._getLineParam(C, D)         
        
        if M1 == M2:
            return [False, None]
        else:
            X = (C2 - C1)/(M1 - M2)
            Y = M1 * X + C1
            
        if math.min(A[0]) <= X <= math.max(B[0]) and math.min(C[0]) <= X <= math.max(D[0]):
            return [True, (X, Y)]
        else:            
            return [False, None]
            
    def _getLineParam(A, B):
        M = (B[1] - A[1])/ (B[0] - A[0])
        C = A[1] - (M * A[0])
        return [M, C]
    
    def _getL2Distance(point1, point2):
        return (point2[0] - point1[0])^2 + (point2[1] - point1[1])^2
    
    def drawState(self):
        raise NotImplementedError
    
    def moveRobot(self):
        raise NotImplementedError