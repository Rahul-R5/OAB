# -*- coding: utf-8 -*-
"""
Created on Fri Jan  1 13:59:59 2021

@author: Rahul R
"""
import numpy as np
from scipy.integrate import solve_ivp

from numpy.linalg import inv
from numpy import dot
from math import pi, cos, sin, tanh, atan, sqrt

#from oab.shape import Shape
from oab.SensorFactory import SensorFactory
from oab.MapInfo import MapInfo
from oab.robot.RobotInfo import RobotInfo


# NOTE: Can use **kwargs for keyword arguments passed. Better readability
class Robot():

    def __init__(self, origin, angle, size=3):
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

    def set_state(self, state):
        self.origin = state[0:2]
        self.angle = state[2]
        self.populate_coords()
        self.sensors[0].set_state(state)

    # --------------------------Sensor Management------------------------------
    def add_sensor(self, **kwargs):
        sensor = SensorFactory.getSensor(**kwargs)
        self.sensors.append(sensor)

    def remove_sensor(self, index=None):
        if index is None:
            self.sensors.pop()
        else:
            del self.sensors[index]

    # -------------------------Movement actuators------------------------------
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

    # --------------------------Path algorithmns-------------------------------

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
        [t, x] = solver.solve(start, dest)

        # Add the path to the history
        if len(self.timestamps) == 0:
            self.timestamps = t
            self.states = x
        else:
            self.timestamps = np.append(self.timestamps, t)
            self.states = np.append(self.states, x, axis=0)

        # The path has been solved separately but the robot's
        # origin/orientation has not been updated for next move
        self.origin = self.states[-1, 0:2]
        self.angle = self.states[-1, 2]

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

    # def get_tangent_lines(self):
    #     mapObj = self.mapInstance
    #     intersections = mapObj.getIntersections(mapObj.obstacles, mapObj.robot)
    #     #intersections.sort(key=lambda x: self._getL2Distance(self.origin, x), reverse=False)
        
    #     grp_thresh = 0.6
    #     lines = []
    #     line = TangentLine(intersections[0])
        
    #     for i in range(1,len(intersections)):
            
    #         if self._getL2Distance(intersections[i], intersections[i-1]) <= grp_thresh:
    #             # Add it to object
    #             line.add_point(intersections[i])
    #         else:
    #             # Add current tangent line to list
    #             lines.append(line)
    #             # Create new object
    #             line = TangentLine(intersections[i])
        
    #     return lines
            
    def get_best_node(self, start, G1, target):
        best = []
        heur_dist = []
        for line in G1:
            h = []
            for node in line.endPoints:
                h.append(-1)
            heur_dist.append(h)
        
        for l_ind, line in enumerate(G1):
            for p_ind, endPoint in enumerate(line.endPoints):
                if heur_dist[l_ind][p_ind] == -1:
                    self.calc_heur_dist(start, [l_ind, p_ind], G1, target, heur_dist)
              
    #TODO: Recursive implementation
    def calc_line_heur_dist(self, start, line_index, G1, target, heur_dist):
        l_ind = line_index
        
        if heur_dist[l_ind][0] != -1:
            heur_dist[l_ind][0] = self.calc_node_heur_dist(start, [l_ind, 0], G1, target, heur_dist)
            
        if heur_dist[l_ind][1] != -1:
            heur_dist[l_ind][1] = self.calc_node_heur_dist(start, [l_ind, 1], G1, target, heur_dist)
        
        [node_index, best_heur] = min(heur_dist[l_ind][:])
        
        return [node_index, best_heur, heur_dist]
    
    def calc_node_heur_dist(self, start, node_index, G1, target, heur_dist):
        [l_ind, p_ind] = node_index
        mapObj = self.mapInstance
        
        # Check collision with self
        line = G1[l_ind]
        pts = line.points
        blocked = False
        for i in range(len(pts)):
                       
            # if line from potential node doesn't intersect with self at 
            # line (pts[i], pts[i+1]), then move to next line
            if not (mapObj._getLineIntersection(line.endPoints[p_ind], target, pts[i], pts[i+1])[0]):
                [M,C] = self.getLineParam(start, target)
                
                # Find line connecting to target angle relative to robot orientation
                relative = atan(M) % (2*pi) > self.angle % (2*pi)                 
                
                # if positive, next line is to left of robot
                # else if negative, next line is to right of robot
                # NOTE: +1, -1 will change if the way the sensor reads points changes
                next_l =  l_ind + 1 if relative else l_ind - 1                    
                
                # if -pi/2 < M < pi/2:
                #     next_l = l_ind - 1                    
                # else:
                #     next_l = l_ind + 1
                    
                heur_dist[l_ind][0] += self.calc_heur_dist(start, [next_l, i], G1, target, heur_dist)
                blocked = True
                break
            else:
                # get the distance to next valid node on same line
                heur_dist[l_ind][p_ind] += self._getL2Distance(pts[i], pts[i+1])
                
        if not blocked:
            heur_dist[l_ind][0] = self._getL2Distance(pts[0], target)
    def calc_node_heur_dist(self, start, node_index, G1, target, heur_dist):
        [l_ind, p_ind] = node_index
        mapObj = self.mapInstance
        if start == target:
            return 0
        elif heur_dist[l_ind][p_ind] != -1:
            return heur_dist[l_ind][p_ind]
        else:
            # Check collision with self
            line = G1[l_ind]
            pts = line.points
            blocked = False
            for i in range(len(pts)):
                           
                # if line from potential node doesn't intersect with self at 
                # line (pts[i], pts[i+1]), then move to next line
                if not (mapObj._getLineIntersection(line.endPoints[p_ind], target, pts[i], pts[i+1])[0]):
                    [M,C] = self.getLineParam(start, target)
                    
                    # Find line connecting to target angle relative to robot orientation
                    relative = atan(M) % (2*pi) > self.angle % (2*pi)                 
                    
                    # if positive, next line is to left of robot
                    # else if negative, next line is to right of robot
                    # NOTE: +1, -1 will change if the way the sensor reads points changes
                    next_l =  l_ind + 1 if relative else l_ind - 1                    
                    
                    # if -pi/2 < M < pi/2:
                    #     next_l = l_ind - 1                    
                    # else:
                    #     next_l = l_ind + 1
                        
                    heur_dist[l_ind][0] += self.calc_heur_dist(start, [next_l, i], G1, target, heur_dist)
                    blocked = True
                    break
                else:
                    # get the distance to next valid node on same line
                    heur_dist[l_ind][p_ind] += self._getL2Distance(pts[i], pts[i+1])
                    
            if not blocked:
                heur_dist[l_ind][0] = self._getL2Distance(pts[0], target)
    """
    Method to determine the next tangent node(if available) or target destination
    using heuristic distance
    """
    def get_heuristic_destination(self, start, lines, target):
        min_h = self._getL2Distance(start, target)
        heuristics = []
        blocked = False
        best_node = target
        tangent_node_present = False
        
        ## Safe path area definition
        # TODO: check if path to target is blocked and choose new node appropriately
        M,C = self.getLineParam(start, target)
        
        # Parameters
        dS = 1      # The length after the sensor
        dR = 0.5    # The tolerance to obstacle gap for robot to pass through
        
        theta = atan(M)
        endPoint = start + (self.sensors[0].length + dS) * [cos(theta), sin(theta)]
        
        left_line = TangentLine(start)
        right_line = TangentLine(start)
        left_line.add_point(endPoint)
        right_line.add_point(endPoint)
        left_line.shift_along_normal(self.size + dR, 1)
        right_line.shift_along_normal(self.size + dR, -1)
            
        for line in lines:
            nodes = [line.start, line.end]            
            
            if self.mapInstance._getLineIntersection(line.start, line.end, \
                                                     left_line.start, left_line.end) \
                or self.mapInstance._getLineIntersection(line.start, line.end, \
                                                      right_line.start, right_line.end):
                blocked = True
            for node in nodes:
                if self._getL2Distance(start, node) + self._getL2Distance(node, target) < min_h:
                    tangent_node_present = True
                    min_h = self._getL2Distance(start, node) + self._getL2Distance(node, target)                    
                    best_node = node

        return [tangent_node_present, blocked, min_h, best_node]

    # Not used
    def perform(self, index):
        # Take index and get action string from translate list
        action = getattr(str(index), lambda: "Invalid action")

        # Take action string and execute function
        action()

    # ---------------------------Helper functions------------------------------
    def _getLineParam(self, A, B):
        M = (B[1] - A[1]) / (B[0] - A[0])
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


# ------------------------Path Solving Algorithmns------------------------------
xd = 5
yd = 5
Dxd = 0
Dyd = 0
D2xd = -1
D2yd = -1
target = []


class PathSolver():
    def __init__(self, mapInstance):
        self.mapInstance = mapInstance
        self.robotInstance = RobotInfo.getRobot()

    def solve(self, start, dest):
        # Initial condition
        init_state = start
        _setDerivatives(xd=dest[0], yd=dest[1], target=[dest[0], dest[1]])

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
        return [t, x]

def _setDerivatives(**kwargs):
    global xd
    global yd
    global Dxd
    global Dyd
    global D2xd
    global D2yd
    global target

    keys = kwargs.keys()

    if "xd" in keys:
        xd = kwargs["xd"]

    if "yd" in keys:
        yd = kwargs["yd"]

    if "Dxd" in keys:
        Dxd = kwargs["Dxd"]

    if "Dyd" in keys:
        Dyd = kwargs["Dyd"]
        
    if "target" in keys:
        target = kwargs["target"]
        
# decorator with argument to attach arguments
def attach_static(**kwargs):
    def decorate(func):
        for args in kwargs:
            setattr(func, args, kwargs[args])
        return func
    return decorate

class TangentLine():
    ## This needs to become a graph class
    def __init__(self, start_point):
        self.points = [start_point]
        self.endPoints = [start_point, None]
        self.robotInstance = RobotInfo.getRobot()
    
    def add_point(self, point):
        self.points.append(point)
        self.endPoints[1] = point
        
    def shift_along_normal(self, offset, direction = 1):
        points = self.points
        for i in range(len(points)-1):
            M,C = self.robotInstance.getLineParam(points[i], points[i+1])
            theta = atan(M) + pi/2
            
            if direction == -1:
                theta = atan(M) + pi/2 + pi
                
            self.points[i] = self.points[i] + offset * [cos(theta), sin(theta)]
            
            if i+1 == len(points) - 1:
                self.points[i+1] = self.points[i+1] + offset * [cos(theta), sin(theta)]
                break
"""
LTG defintion
"""
class LTG():
    def __init__(self):
        self.lines = []
        self.G1 = []
        self.G2 = []
        self.mapInstance = MapInfo.getMap()
        self.robotInstance = RobotInfo.getRobot()
        
    def calc_tangent_graph(self):
        mapObj = self.mapInstance
        intersections = mapObj.getIntersections(mapObj.obstacles, mapObj.robot)
        
        grp_thresh = 0.6
        lines = []
        line = TangentLine(intersections[0])
        robot = self.robotInstance
        for i in range(1,len(intersections)):
            
            if robot._getL2Distance(intersections[i], intersections[i-1]) <= grp_thresh:
                # Add it to object
                line.add_point(intersections[i])
            else:
                # Add current tangent line to list
                line.shift_along_normal(0.3, -1)
                lines.append(line)
                
                # Create new object
                line = TangentLine(intersections[i])    # Can create line with one node
                if i == len(intersections) - 1:
                    lines.append(line)
        
        self.lines = lines
    
    def calc_admissible_sub_graph(self):
        G1 = []
        robot = self.robotInstance
        for line in self.lines:
            filtered_line = []
            for node in line.endPoints:
                if robot._getL2Distance(node, target) < robot._getL2Distance(robot.origin[0], target):
                    filtered_line.append(node)
            G1.append(filtered_line)
        
        self.G1 = G1
        
    def calc_leave_sub_graph(self):
        G2 = []
        robot = self.robotInstance
        for line in self.G1:
            filtered_line = []
            for node in line.endPoints:
                if robot._getL2Distance(node, target) < dMin:
                    filtered_line.append(node)
            G2.append(filtered_line)
        
        self.G2 = G2
        
    def get_tangent_graph(self):
        if len(self.lines) == 0:
            self.calc_tangent_graph()
            
        return self.lines            
        
    def get_admissible_sub_graph(self):
        if len(self.G1) == 0:
            self.calc_admissible_sub_graph()
            
        return self.G1   
        
    def get_leave_sub_graph(self):
        if len(self.G2) == 0:
            self.calc_leave_sub_graph()
            
        return self.G2   
        
    def update_all(self):
        self.calc_tangent_graph()
        self.calc_admissible_sub_graph()
        self.calc_leave_sub_graph()
        
"""
Function which models a movement algorithm similar to the tangent bug algorithm

param:
    t : time step (float)
    x : state of the robot([posX, posY, orientation])
"""
inMoveToTargetMode = True
inBoundaryFollowMode = False
inTransition = False

minNode = None
dMin = None
leaveNode = None
dLeave = None

# Algorithmn
# 1. Move to target till
#   a. target reached, stop (No need as movement is default till target, it stops auto matically)
#   b. reached local minimum, go to step 2
# 2. Choose boundary following direction and move through LTG edges while 
#    recording dmin(T) till
#   a. target reached, stop 
#   b. found Vleave point ST distance to target is less than dmin(T), go to step 3
#   c. robot made a loop around obstacle, stop
# 3. Move directly to Vleave till find Z ST d(Z,T) < dmin(T)

# Robot moves along LTG "edges" both visible and augmented edges(between LTG node and target)
# From the visible detections, filter it to LTG nodes (i.e ends of line segments)
# From the LTG nodes, filter the ones which do not give an admissible edge(i.e travelling along it doesn't decrease distance to target)
# Hint: This can be done by checking if the node is closer to target than current
#       position. Might look trivial but we can think of it as moving to that node
#       is decreasing our distance to target(i.e That admissible node lies inside circle of centre target and radius of (|robot - target|))

# After above processing is done, you have sub graph G1 that can be used for analysis
# You can now choose LTG node which has best heuristic (dist(start, node) + dist(node, target))
# The dist from node to target is not always a straight line, it can pass through the boundary itself
# In that case, loop through the boundary points till one has direct line of sight to target
# Total distance will be (dist(start, node) + dist(node, LOSnode) + dist(LOSnode, target))
# The motion to target terminates when all nodes in sub graph don't decrease distance(i.e not admissible, not in that rectangle)

# Save local min dist to dmin
# This local minimum point lies on the boundary of obstacle, it should be an LTG node
# For you, it is on the normally shifted tangent line
# Go straight to this local min point and Switch to boundary following

ltg = LTG()

# @attach_static(inBoundaryFollowMode = False, finalTarget = [], robot = RobotInfo.getRobot(), map = MapInfo.getMap())
def tangentbug_model(t, x):
    # States extracted
    theta = x[2]
    robot = RobotInfo.getRobot()
    robot.set_state(x)
 
    ##------------------------Controller Related-------------------------------
    # Get control inputs
    [v, omega] = CL_controller(t, x)

    # Model evaluated
    dState = [v*cos(theta),
              v*sin(theta),
              omega]

    # Estimating next state of robot for collision detection
    next_x = x[0] + dState[0] * 0.01
    next_y = x[1] + dState[1] * 0.01
    next_theta = x[2] + dState[2] * 0.01
    next_state = [next_x, next_y, next_theta]
    
    ##-----------------------Tangent Bug Algorithm-----------------------------
    global inMoveToTargetMode
    global inBoundaryFollowMode
    global inTransition
    
    global minNode
    global dMin
    global leaveNode
    global dLeave
    global ltg
    
    # Update graph for new timestep
    ltg.update()
    
    # Get admissible tangent sub graph G1(common)
    G1 = ltg.get_admissible_sub_graph()
    
    # Determine which mode you are in
    
    # If in moveToTarget mode
    if inMoveToTargetMode and not inTransition:
        # If no tangent nodes present just travel to target
        
        # if there exists LTG edges and an admissible edge exists in sub graph G1
        if len(ltg.lines) > 0 and len(G1) > 0:
            # continue in target mode
            
            # find best tangent node with heuristics and move towards it            
            # Save this node as local min node and its local min value                 
            
            [minNode, dMin] = get_best_node(G1)            
            _setDerivatives(xd = minNode[0], yd = minNode[1])
            
        # if there exists LTG edges and sub graph G1 is empty
        elif len(ltg.lines) > 0 and len(G1) == 0:
            # Save dmin
            
            # Choose the LTG node that is local min and switch to 
            # boundary follow and transition to it
            _setDerivatives(xd = minNode[0], yd = minNode[1])
            inTransition = True
            
    # If in moveToTarget mode and transition
    elif inMoveToTargetMode and inTransition:    
        # if robot position is equal to temp dest
        if next_state[0:2] == [xd,yd]:
            # switch to boundary follow
            inBoundaryFollowMode = True
            
            # remove moveToTarget and transition
            inMoveToTargetMode = False
            inTransition = False
            
    # If in boundaryFollow mode
    elif inBoundaryFollowMode and not inTransition:
        # Get sub graph from G1 as G2, where d((V,T)) < dmin (i.e there is a node inside the graph)
        G2 = ltg.get_leave_sub_graph()
        
        # If G2 empty
        if len(G2) == 0:
           [minNode, dMin] = get_best_node(G1)            
           _setDerivatives(xd = minNode[0], yd = minNode[1])
            
        # If G2 not empty
        if len(G2) > 0:
            # move to that point and transition to moveToTarget
            [leaveNode, dLeave] = get_leave_node(G2)
            _setDerivatives(xd = leaveNode[0], yd = leaveNode[1])
    
    # If in boundaryFollow mode and transition
    elif inBoundaryFollowMode and inTransition:        
        # if robot position in a closer position than dmin
        if robot._getL2Distance(x[0:2], target) < dMin:
            # switch to moveToTarget
            inMoveToTargetMode = True            
            
            # remove boundaryMode and transition
            inBoundaryFollowMode = False
            inTransition = False
            
            # set destination to target
            _setDerivatives(xd = target[0], yd = target[1])
            
    # Change dest to target from temp target if near temp target as the integration will stop
    if next_state[0:2] == [xd,yd] and next_state[0:2] != target:
        _setDerivatives(xd = target[0], yd = target[1])
        
    # Check collision for next state
    if robot.checkCollision(next_state) is True:
        # Can be toggled for disabling collision
        return [0, 0, 0]
    else:
        return dState


def mobile_model(t, x):
    # States extracted
    theta = x[2]

    # Get control inputs
    [v, omega] = CL_controller(t, x)

    # Model evaluated
    dState = [v*cos(theta),
              v*sin(theta),
              omega]

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
    pt_offset = 0.2
    K1 = 5
    K2 = K1

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
    # The look ahead point is tracking the trifolium trajectory
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

    # K1 and K2 can be different since they are different inputs
    u1 = -1 * K1 * e_t[0]
    u2 = -1 * K2 * e_t[1]

    # Control inputs calculated
    control = dot(inv(A), [DPd[0] + u1, DPd[1] + u2])

    v = control[0]
    omega = control[1]

    return [v, omega]
