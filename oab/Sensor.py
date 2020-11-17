#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 09:48:47 2020

@author: rahulr
"""
from math import cos, sin

class Sensor():
    
    def __init__(self, origin, start_ang, ang_range, length, num_ray):
        self.origin = origin
        self.start_ang = start_ang
        self.ang_range = ang_range
        self.length = length
        self.num_ray = num_ray
        self.ray_pts = self.getEndPoints(self.origin, self.start_ang, 
                                         self.ang_range, self.length,
                                         self.num_ray)
    """
    Calculates the end of the finite length sensor rays with given parameters
    and returns the list of tuples
    @return pts: list of float tuples
    """
    def getEndPoints(origin, start_ang, ang_range, length, num_ray):
        pts = []
        reso = ang_range/num_ray
        for i in range(num_ray):
            
            ptX = origin(0)
            ptY = origin(1)
            ptX = ptX + length * cos(start_ang + i * reso)
            ptY = ptY + length * sin(start_ang + i * reso)
            pt = (ptX, ptY)
            
            pts.append(pt)
            
        return pts
    
    """
    Moves the sensor to a different coordinate
    """
    def relocate(self, origin):
        self.origin = origin
        self.ray_pts = self.getEndPoints(self.origin, self.start_ang, 
                                         self.ang_range, self.length,
                                         self.num_ray)
    """
    Rotates the sensor by given angle
    +ve angle: counter-clockwise rotation
    -ve angle: clockwise rotation
    """
    def rotate(self, angle):
        self.start_ang += angle
        self.ray_pts = self.getEndPoints(self.origin, self.start_ang, 
                                         self.ang_range, self.length,
                                         self.num_ray)
    """
    Returns the ray end points
    """    
    def get_pts(self):
        return self.ray_pts