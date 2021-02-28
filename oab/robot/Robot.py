# -*- coding: utf-8 -*-
"""
Created on Fri Jan  1 13:59:59 2021

@author: Rahul R
"""
import numpy as np
from scipy.integrate import solve_ivp

from numpy.linalg import inv
from numpy import dot
from math import pi, cos, sin, tanh, sqrt

from oab.shape import Shape
from oab.SensorFactory import SensorFactory
from oab.MapInfo import MapInfo
from oab.robot.RobotInfo import RobotInfo


# NOTE: Can use **kwargs for keyword arguments passed. Better readability
class Robot():
    
    def __init__(self, origin, angle, size = 3):
        self.origin = origin
        self.angle = angle
        self.size = size
        
        self.sensors = []
        self.coords = []         
        self.timestamps = np.array([])
        self.states = np.array([])
        
        self.mapInstance = MapInfo.getMap()
        self.history = []
        self.populate_coords()
        
    def populate_coords(self):
        size = self.size
        reso = (2 * pi/5)
        ptX = self.origin[0]
        ptY = self.origin[1]
        start_ang = self.angle
        
        coords = []
        for i in range(5):
            if i == 0:
                X = ptX + size * cos(start_ang + i * reso) * 1.5
                Y = ptY + size * sin(start_ang + i * reso) * 1.5
            else:
                X = ptX + size * cos(start_ang + i * reso) 
                Y = ptY + size * sin(start_ang + i * reso) 
            coords.append([X, Y])
        
        # Update the associated sensor to the new robot position, if any    
        if len(self.sensors) >= 1:
            for sensor in self.sensors:
                sensor.relocate(self.origin)
                sensor.set_pts()  
                
        self.coords = coords
     
    def set_state(self,state):
        self.origin = state[0:2]
        self.angle = state[2]
        self.populate_coords()
        self.sensors[0].set_state(state)
        
    #--------------------------Sensor Management------------------------------        
    def add_sensor(self, **kwargs):
        sensor = SensorFactory.getSensor(**kwargs)
        self.sensors.append(sensor)
    
    def remove_sensor(self, index = None):
        if index is None:
            self.sensors.pop()
        else:
            del self.sensors[index]
            
    def get_tangent_nodes(self):
        intersections = []
        #det_index = []
        sensor = self.sensors[0]
        sensor_pts = sensor.get_pts()
        sensor_origin = sensor.origin
        
        obstacles = self.mapInstance.obstacles
        for j in range(len(sensor_pts)):
            temp_pts = []
            obs_index = []
            for index, obstacle in enumerate(obstacles):
                obstacle_pts = obstacle.get_coords()
                for i in range(len(obstacle_pts)):    
                    vertA = i
                    
                    if i == (len(obstacle_pts) - 1):
                        vertB = 0              
                    else:
                        vertB = vertA + 1
                        
                    [flag, point] = self.mapInstance._getLineIntersection(obstacle_pts[vertA], obstacle_pts[vertB], 
                                                                  sensor_pts[j], sensor_origin)
                        
                    if flag:
                        temp_pts.append(point)
                        obs_index.append(index)
            
            # Eliminating intersections through obstacles through L2 norm
            val, pos = self.mapInstance._getClosestIntersection(temp_pts, sensor_origin)
            
            if pos >= 0:
                intersections.append([temp_pts[pos], obs_index[pos]])
                #det_index.append(obs_index[pos])
                
        # Categorizing the detections by obstacle
        intersections.sort(key = lambda x: x[1], reverse = False)
                
        return intersections
        
    #-------------------------Movement actuators------------------------------  
    def rotate(self, angle):
        self.angle += angle
        self.populate_coords()
        if len(self.sensors) > 0:
            for sensor in self.sensors:
                sensor.rotate(angle)   
                
    def relocate(self, dest):
        self.origin = dest
        self.populate_coords()
        if len(self.sensors) > 0:
            for sensor in self.sensors:
                sensor.relocate(dest)   
                
            
    #--------------------------Path algorithmns-------------------------------
    def moveto(self, dest):
        
        # Given dest, set this dest in the controller of choice
        map = self.mapInstance
        solver = PathSolver(map)
        
        # Set the initial condition variables
        x0 = self.origin[0]
        y0 = self.origin[1]
        theta0 = self.angle
        start = np.array([x0, y0, theta0])
        
        # Solve the path with the initial conditions
        [t, x] = solver.solve(start,dest)

        # Add the path to the history
        if len(self.timestamps) == 0:
            self.timestamps = t
            self.states = x
        else:
            self.timestamps = np.append(self.timestamps,t)
            self.states = np.append(self.states,x,axis = 0)
            
        # The path has been solved separately but the robot's 
        # origin/orientation has not been updated for next move
        self.origin = self.states[-1,0:2]
        self.angle = self.states[-1,2]
    
    def checkPath(self, t, path):
        # Get robot size 
        size = self.size
        
        # Loop through states
        for index in range(len(path)):
            state = path[index]
            # If robot intersects, trim states and timestamps
            for obstacle in self.mapInstance.obstacles:
                dist = self._getL2Distance(obstacle.origin, state[0:2])
                if dist <= (obstacle.size + size):
                    t = t[:index]
                    path = path[:index]
                    return [t, path]
            
        return [t, path]
    
    def checkCollision(self, state):
        # If robot intersects, trim states and timestamps
        for obstacle in self.mapInstance.obstacles:
            dist = self._getL2Distance(obstacle.origin, state[0:2])
            if dist <= (obstacle.size + self.size):
                return True
            
        return False

    # Not used           
    def perform(self, index):
        # Take index and get action string from translate list
        action = getattr(str(index), lambda: "Invalid action")
        
        # Take action string and execute function        
        action()
    
    #---------------------------Helper functions------------------------------    
    def _getLineParam(self, A, B):
        M = (B[1] - A[1])/ (B[0] - A[0])
        C = A[1] - (M * A[0])
        return [M, C]
    
    def getHistory(self):
        compressed = []
        currentMove = self.history[0]
        count = 0
        for move in self.history:
            if currentMove is None or move is not currentMove:
                compressed.append(currentMove + str(count))
                currentMove = move
                count = 1
            elif currentMove == move:
                count += 1
        
        # compressed[] updated when the move changes, 
        # this takes care of end of moves condition
        compressed.append(currentMove + str(count))
        return compressed
    
    def _getL2Distance(self, point1, point2):
        return sqrt((point2[0] - point1[0]) * (point2[0] - point1[0]) 
                    + (point2[1] - point1[1]) * (point2[1] - point1[1]))
    
    def get_pts(self):
        return self.coords
    
#------------------------Path Solving Algorithmns------------------------------            
xd = 5
yd = 5
Dxd = 0
Dyd = 0
D2xd = -1
D2yd = -1

class PathSolver():
    def __init__(self, mapInstance):
        self.mapInstance = mapInstance
        self.robotInstance = RobotInfo.getRobot()
        
    def solve(self, start, dest):
        # Initial condition
        init_state = start
        _setDerivatives(xd = dest[0], yd = dest[1])
        
        # Simulation time
        time = 100
        
        # Model limits        
        # TODO:These limits need to be used
        # v_limit = 2
        # omega_limit = pi
        
        solver = "BDF"
        
        # Model solver
        sol = solve_ivp(tangentbug_model, [0, time], init_state, 
                        method=solver, dense_output=True)
        t = np.array(sol.t)
        x = np.transpose(sol.y)
        return [t,x]
    
def _setDerivatives(**kwargs):
        global xd
        global yd
        global Dxd
        global Dyd
        global D2xd
        global D2yd
        
        keys = kwargs.keys()
        
        if "xd" in keys:            
            xd = kwargs["xd"]
            
        if "yd" in keys:  
            yd = kwargs["yd"]
            
        if "Dxd" in keys:            
            Dxd = kwargs["Dxd"]
            
        if "Dyd" in keys:  
            Dyd = kwargs["Dyd"]
            
#decorator with argument to attach arguments
def attach_static(**kwargs):
    def decorate(func):
        for args in kwargs:
            setattr(func, args, kwargs[args])
        return func
    return decorate     

#@attach_static(inBoundaryMode = False, finalTarget = [], robot = RobotInfo.getRobot(), map = MapInfo.getMap())
def tangentbug_model(t,x):
    # States extracted
    theta = x[2]    
    robot = RobotInfo.getRobot()
    # Remember: heuristics monotonically decreases throughout the path
    # The break away point is when it starts increasing, it shifts there to boundary following
    
    # Common on repeat
    # Get sensor data
    nodes = robot.get_tangent_nodes()
    # If no detection(i.e no obstacles in path), continue as normal
    if len(nodes) > 0:
    # If detection, calculate heuristic for all tangent nodes(i.e detections)
        pass
    # If the heuristic through node is higher than straight through, continue as same
    
    # Else change target to this temporary min heuristic target and move towards it?
    # Now it is transitioning to the point on the obstacle where boundary following happens
    # Switch mode or some flag for boundary mode
    
    # let it run
    
    # If in boundary mode and target is not destination
    
    # Set target to a point on parallel line to the sensed endpoints in direction of heuristics
    
    # Get control inputs
    [v, omega] = CL_controller(t, x)

    # Model evaluated
    dState = [  v*cos(theta),
                v*sin(theta),
                omega ]
    
    # Estimating next state of robot for collision detection
    next_x = x[0] + dState[0] * 0.01
    next_y = x[1] + dState[1] * 0.01
    next_theta = x[2] + dState[2] * 0.01
    next_state = [next_x, next_y, next_theta]
    
    # Check collision for next state 
    if robot.checkCollision(next_state) is True:
        # Can be toggled for disabling collision
        return [0, 0, 0]
    else:
        return dState
    
def mobile_model(t,x):
    # States extracted
    theta = x[2]
    
    # Get control inputs
    [v, omega] = CL_controller(t, x)

    # Model evaluated
    dState = [  v*cos(theta),
                v*sin(theta),
                omega ]
    
    # Estimating next state of robot for collision detection
    next_x = x[0] + dState[0] * 0.01
    next_y = x[1] + dState[1] * 0.01
    next_theta = x[2] + dState[2] * 0.01
    next_state = [next_x, next_y, next_theta]
    
    # Check collision for next state 
    robotInstance = RobotInfo.getRobot()
    if robotInstance.checkCollision(next_state) is True:
        # Can be toggled for disabling collision
        return [0, 0, 0]
    else:
        return dState
        
def CL_controller(t, x):
    # Variables
    pt_offset = 0.2;
    K1 = 5;
    K2 = K1;
    
    # Not Used
    # a = 2;
    # c = 0.05;
    # b = 3 * c;
    
    x_pos = x[0]
    y_pos = x[1]
    theta = x[2]
     
    # Trajectory calculated 
    # Taken from global variables
    
    P = [x_pos + pt_offset * cos(theta), y_pos + pt_offset * sin(theta)]
    #The look ahead point is tracking the trifolium trajectory
    Pd = [xd, yd]          
    DPd = [Dxd, Dyd] 
    
    A = [[cos(theta), (-1*pt_offset * sin(theta))], 
         [sin(theta), (pt_offset * cos(theta))]]
    
    # e_t = P - Pd 
    e_0 = P[0] - Pd[0]   
    e_1 = P[1] - Pd[1]
    e_t = []
    e_t.append(tanh(e_0))
    e_t.append(tanh(e_1))
    
    #K1 and K2 can be different since they are different inputs
    u1 = -1 * K1 * e_t[0] 
    u2 = -1 * K2 * e_t[1]
    
    # Control inputs calculated 
    control = dot(inv(A), [DPd[0] + u1, DPd[1] + u2]) 
    
    v = control[0]
    omega = control[1]
    
    return [v, omega]