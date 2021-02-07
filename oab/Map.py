#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 10:49:55 2020

@author: rahulr
"""
import math

#import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

from oab.shape.ShapeFactory import ShapeFactory
from oab.shape.Shape import Shape
from oab.robot.Robot import Robot
from oab.robot.RobotFactory import RobotFactory

# NOTE: Can use **kwargs for keyword arguments passed. Better reaability
class Map():
    
    
    def __init__(self, resolution, grid, seed):
        self.resolution = resolution
        self.rows = grid[0]
        self.columns = grid[1]
        self.seed = seed
        self.obstacles = []
        self.robot = None
        self.intersections = []
    
    def addObstacles(self, num):
        if self.obstacles is not None:
            self.obstacles += ShapeFactory.getShapes(num)
        else:
            self.obstacles = ShapeFactory.getShapes(num)
            
    def addRobot(self, options = []):
        if self.robot is None:            
            if len(options) == 2:
                self.robot = RobotFactory.getRobot([options[0], options[1]])
            else:
                self.robot = RobotFactory.getRobot()
            
            
    def getIntersections(self, param_1, param_2):
        intersections = []
        
        if len(param_1) == 0 or param_2 is None:
            return []
        
        elif (not isinstance(param_1, list))  and isinstance(param_1, type(Shape)) and isinstance(param_2, type(Robot)):
            param_2 = param_2.sensors[0]
            intersections += self.getSingleObstacleSensorIntersections(param_1, param_2)
            
        elif isinstance(param_1, list) and isinstance(param_1[0], Shape) and isinstance(param_2, Robot):
            #for obstacle in param_1:
            param_2 = param_2.sensors[0]
            intersections += self.getManyObstacleSensorIntersections(param_1, param_2)
                
        elif (not isinstance(param_1, list)) and isinstance(param_1[0], type(Shape)) and (not isinstance(param_2, list)) and isinstance(param_2[0], type(Shape)):
            intersections += self.getSingleShapeIntersections(param_1, param_2)
             
        elif isinstance(param_1, list) and isinstance(param_1[0], type(Shape)) and isinstance(param_2, list) and isinstance(param_2[0], type(Shape)):
            for obstacle_1 in param_1:
                for obstacle_2 in param_2:
                    intersections += self.getSingleShapeIntersections(obstacle_1, obstacle_2)
        
        return intersections
    
    """
    Called when only one sensor and many obstacles exist in environment
    """
    def getManyObstacleSensorIntersections(self, obj_1, obj_2):
        intersections = []
        #obstacle_pts = obj_1.get_coords()
        sensor_pts = obj_2.get_pts()
        sensor_origin = obj_2.origin
        
        for j in range(len(sensor_pts)):
            temp_pts = []
            for obstacle in obj_1:
                obstacle_pts = obstacle.get_coords()
                for i in range(len(obstacle_pts)):    
                    vertA = i
                    
                    if i == (len(obstacle_pts) - 1):
                        vertB = 0              
                    else:
                        vertB = vertA + 1
                        
                    [flag, point] = self._getLineIntersection(obstacle_pts[vertA], obstacle_pts[vertB], 
                                                                  sensor_pts[j], sensor_origin)
                        
                    if flag:
                        temp_pts.append(point)
            
            # Eliminating intersections through obstacles through L2 norm
            val, pos = self._getClosestIntersection(temp_pts, sensor_origin)
            
            if pos >= 0:
                intersections.append(temp_pts[pos])
                
        return intersections
    """
    Called when only one sensor and one obstacle exists in environment
    """
    def getSingleObstacleSensorIntersections(self, obj_1, obj_2):        
        intersections = []
        obstacle_pts = obj_1.get_coords()
        sensor_pts = obj_2.get_pts()
        sensor_origin = obj_2.origin
        
        # Loop through all edges of an obstacle and check if particular line intersects it,
        # figure out which intersections are true
        
        for j in range(len(sensor_pts)):
            temp_pts = []
            for i in range(len(obstacle_pts)):    
                vertA = i
                
                if i == (len(obstacle_pts) - 1):
                    vertB = 0              
                else:
                    vertB = vertA + 1
                    
                [flag, point] = self._getLineIntersection(obstacle_pts[vertA], obstacle_pts[vertB], 
                                                              sensor_pts[j], sensor_origin)
                    
                if flag:
                    temp_pts.append(point)
            
            # Eliminating intersections through obstacles through L2 norm
            val, pos = self.getClosestIntersection(temp_pts, sensor_origin)
            
            if pos >= 0:
                intersections.append(temp_pts[pos])
                
        return intersections
    """
    Returns the intersection having the least L2 norm with origin and its index
    @param intersections - list of intersection points
    @param origin - reference point to calculate distance to intersection
    """
    def _getClosestIntersection(self, intersections, origin):
        min_d = 10000000000
        pos = -1
        
        for i in range(len(intersections)):
            point = intersections[i]
            d = self._getL2Distance(point, origin)
            
            if d <= min_d:
                min_d = d
                pos = i
        # return the mid
        return [min_d, pos]
        
        
    def getSingleShapeIntersections(self, obj_1, obj_2):
        raise NotImplementedError
            
                    
    def _getLineIntersection(self, A, B, C, D):   
        [M1, C1] = self._getLineParam(A, B)         
        [M2, C2] = self._getLineParam(C, D)         
        
        if abs(M1) == abs(M2):
            return [False, None]
        else:
            X = (C2 - C1)/(M1 - M2)
            Y = M1 * X + C1
            min_1 = min([A[0],B[0]])
            max_1 = max([A[0],B[0]])
            min_2 = min([C[0],D[0]])
            max_2 = max([C[0],D[0]])
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
        ax = self.drawRobot(ax)
        ax = self.drawIntersections(ax)
        return ax
        
    def drawManyObstacles(self, ax):
        obstacles = self.obstacles
        for obstacle in obstacles:
            vertices = obstacle.get_coords()
            ax = self._drawObstacle(ax, vertices)
        return ax
            
    def _drawObstacle(self, ax, points):        
        patches = []
        
        # polygon has fill and edge colour options to differentiate from robot
        polygon = Polygon(points, closed = True)
        patches.append(polygon)
    
        xmax = self.rows
        ymax = self.columns
        ax = plt.gca()
        ax.add_patch(polygon)
        ax.set_xlim([-xmax, xmax])
        ax.set_ylim([-ymax, ymax])
        
        return ax
    ## TODO: Change the camera focus from starting at the robot to a static frame
    def drawRobot(self, ax):
        # Drawing robot
        robot = self.robot
        robot_pts = robot.get_pts()
        patches = []

        polygon = Polygon(robot_pts, closed = True)
        patches.append(polygon)
    
        ax.add_patch(polygon)
        
        # Drawing sensor
        sensor = robot.sensors[0]
        origin = sensor.origin
        sensor_pts = sensor.get_pts()
        for point in sensor_pts:
            ax.plot([origin[0], point[0]], [origin[1], point[1]]) 
        
        return ax
    
    def drawIntersections(self, ax):
        self.intersections += self.getIntersections(self.obstacles, self.robot)
        xPoints = []
        yPoints = []
        
        for i in range(len(self.intersections)):
            xPoints.append(self.intersections[i][0])
            yPoints.append(self.intersections[i][1])
        ax.plot(xPoints, yPoints, 'r*')
        
        return ax
    
    def moveRobot(self):
        raise NotImplementedError