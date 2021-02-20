#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 10:49:55 2020

@author: rahulr
"""
from math import sqrt
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

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
            
    #--------------------Intersection Detection Agorithmns---------------------        
    def getIntersections(self, param_1, param_2):
        intersections = []
        
        if len(param_1) == 0 or param_2 is None:
            return []
        
        elif (not isinstance(param_1, list))  and isinstance(param_1, type(Shape)) and isinstance(param_2, type(Robot)):
            param_2 = param_2.sensors[0]
            intersections += self.getSingleObstacleSensorIntersections(param_1, param_2)
            
        elif isinstance(param_1, list) and isinstance(param_1[0], Shape) and isinstance(param_2, Robot):
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
   
    def getSingleObstacleSensorIntersections(self, obj_1, obj_2):      
        """
        Called when only one sensor and one obstacle exists in environment
        """
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
    
    def _getClosestIntersection(self, intersections, origin):
        """
        Returns the intersection having the least L2 norm with origin and its index
        @param intersections - list of intersection points
        @param origin - reference point to calculate distance to intersection
        """
        min_d = 10000000000
        pos = -1
        
        for i in range(len(intersections)):
            point = intersections[i]
            d = self._getL2Distance(point, origin)
            
            if d <= min_d:
                min_d = d
                pos = i
                
        # return the min
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
        if (B[0] - A[0]) != 0:
            M = (B[1] - A[1])/ (B[0] - A[0])
        else:
            M = 10e3
        C = A[1] - (M * A[0])
        return [M, C]
    
    def _getL2Distance(self, point1, point2):
        return sqrt((point2[0] - point1[0]) * (point2[0] - point1[0]) 
                         + (point2[1] - point1[1]) * (point2[1] - point1[1]))
    
   #--------------------------Animation Methods-------------------------------
    def getPatches(self, states):
        patches = []
        # Patches of obstacles(Done once since they don't move)
        patches.append(self._getObstaclePatches())
        
        for state in states:
            frame = []
            # Patches of robot(and sensor)
            frame.append(self._getRobotPatches(state))
            # Points of intersection
            frame.append(self._getIntersections())
            
            patches.append(frame)
            
        # Return accumulated list of patches and points
        return patches
    
    def _getObstaclePatches(self):        
        patches = []        
        obstacles = self.obstacles
        for obstacle in obstacles:
            polygon = Polygon(obstacle.get_coords(), closed = True)
            patches.append(polygon)
    
        return patches
    
    def _getRobotPatches(self, state):        
        patches = []        
        
        # Drawing robot
        robot = self.robot
        robot.set_state(state)
        robot_pts = robot.get_pts()
        
        # Make polygon patch
        polygon = Polygon(robot_pts, closed = True)
        patches.append(polygon)
        
        # Drawing sensor
        sensor = robot.sensors[0]
        origin = sensor.origin
        sensor_pts = sensor.get_pts()
        for point in sensor_pts:
            line = plt.Line2D([origin[0], point[0]], [origin[1], point[1]]) 
            patches.append(line)
            
        return patches
    
    def _getIntersections(self, state = None):        
        """
        Gets the intersection points between sensor rays and obstacles
        for the given state of the robot. No plotting is done
        @returns: a list of x and y points
        """
        xPoints = []
        yPoints = []
        
        # Need not set robot state as it is already done before calling this
        # function
        if state is not None:
            self.robot.set_state(state)
            
        intersections = self.getIntersections(self.obstacles, self.robot)
        self.intersections += intersections
        
        for i in range(len(intersections)):
            xPoints.append(intersections[i][0])
            yPoints.append(intersections[i][1])
        
        return [xPoints, yPoints]
    
    #--------------------Map Rendering Methods--------------------------------
    def drawState(self, ax):
        """
        Draws the current state of the map onto the the given axes object
        """
        self.drawManyObstacles(ax)
        self.drawRobot(ax)
        self.drawIntersections(ax)
            
    def drawManyObstacles(self, ax):
        obstacles = self.obstacles
        for obstacle in obstacles:
            vertices = obstacle.get_coords()
            self._drawObstacle(ax, vertices)
            
    def _drawObstacle(self, ax, points):    
        """
        Method for drawing a single obstacle from points onto given axes object

        Parameters
        ----------
        ax : matplotlib.axes._subplots.AxesSubplot
            axes object onto which the obstacle is drawn
        points : list
            obstacle coordinates used to generate a polygon
        """
        patches = []
        
        # polygon has fill and edge colour options to differentiate from robot
        polygon = Polygon(points, closed = True, facecolor = "darkblue")
        patches.append(polygon)
        ax.add_patch(polygon)
    
    def drawRobot(self, ax):
        # Drawing robot
        robot = self.robot
        robot_pts = robot.get_pts()
        patches = []

        polygon = Polygon(robot_pts, closed = True, facecolor = "darkgrey")
        patches.append(polygon)
    
        ax.add_patch(polygon)
        
        # Drawing sensor
        sensor = robot.sensors[0]
        origin = sensor.origin
        sensor_pts = sensor.get_pts()
        for point in sensor_pts:
            line = plt.Line2D([origin[0], point[0]], [origin[1], point[1]], color = "yellow") 
            ax.add_line(line)
    
    def drawIntersections(self, ax):
        intersections = self.getIntersections(self.obstacles, self.robot)
        self.intersections += intersections
        xPoints = []
        yPoints = []
        
        for i in range(len(intersections)):
            xPoints.append(intersections[i][0])
            yPoints.append(intersections[i][1])
        ax.plot(xPoints, yPoints, 'r*')
    
    #---------------------------Outdated--------------------------------------
    def moveRobot(self):
        raise NotImplementedError