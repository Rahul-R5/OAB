# -*- coding: utf-8 -*-
"""
Created on Fri Jan  1 13:59:59 2021

@author: Rahul R
"""
from math import pi, cos, sin, atan2, sqrt
from oab.SensorFactory import SensorFactory

# NOTE: Can use **kwargs for keyword arguments passed. Better reaability
class Robot():
    
    def __init__(self, origin, angle):
        self.origin = origin
        self.angle = angle
        self.sensors = []
        self.coords = []         
        #self.actions =  self.populate_actions()
        self.history = []
        self.populate_coords()
        
    def populate_coords(self):
        size = 3
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
         
        self.coords = coords
        
            
    def add_sensor(self):
        sensor = SensorFactory.getSensor()
        sensor.relocate(self.origin)
        offset = self.angle - (sensor.start_ang + sensor.ang_range/2)
        sensor.rotate(offset)
        self.sensors.append(sensor)
    
    def remove_sensor(self, index = None):
        if index is None:
            self.sensors.pop()
        else:
            del self.sensors[index]
        
    ## Movement actuators
    
    def forward(self):
        x,y = self.origin
        self.origin = [x + cos(self.angle), y + sin(self.angle)]
        self.history.append("F")
        
    def rightTurn(self):
        self.angle -= pi/180
        self.history.append("R")
    
    def backward(self):
        x,y = self.origin
        self.origin = [x - cos(self.angle), y - sin(self.angle)]
        self.history.append("B")
        
    def leftTurn(self):
        self.angle += pi/180
        self.history.append("L")
        
    def rotate(self, angle):
        self.angle += angle
        self.coords = self.populate_coords()
        if len(self.sensors) > 0:
            for sensor in self.sensors:
                sensor.rotate(angle)   
                
    ## Path algorithmns
    
    def moveto(self, dest):
        origin = self.origin
        dy = dest[1] - origin[1]
        dx = dest[0] - origin[0]
        angle = atan2(dy, dx)
        ang_offset = angle - self.angle
        self.rotate(ang_offset)
        
        
        while(True):
            if self._getL2Distance(self.origin, dest) < 1:
                self.origin = dest
                self.populate_coords()
                
                # The sensor coords are not updated
                self.sensors[0].relocate(self.origin)
                self.sensors[0].set_pts()                
                break
            else:
                self.forward()
                self.populate_coords()
                self.sensors[0].relocate(self.origin)
                self.sensors[0].set_pts()
    
        
    def perform(self, index):
        # Take index and get action string from translate list
        action = getattr(str(index), lambda: "Invalid action")
        
        action()
        # Take action string and execute function
        
    
    ## Helper functions
    
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
        return sqrt((point2[0] - point1[0]) * (point2[0] - point1[0]) + (point2[1] - point1[1]) * (point2[1] - point1[1]))
    
    def get_pts(self):
        return self.coords
            
        
        