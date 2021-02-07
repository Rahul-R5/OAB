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
        self.ray_pts = []
        self.set_pts()
        
    """
    Calculates the end of the finite length sensor rays with given parameters
    and returns the 2D array
    @return pts: list of float list(2D array)
    """
    # NOTE: Can use **kwargs for keyword arguments passed. Better reaability
    def set_pts(self):
        pts = []        
        length = self.length
        start_ang = self.start_ang
        num_ray = self.num_ray
        reso = self.ang_range/num_ray
        for i in range(num_ray):
            
            ptX = self.origin[0]
            ptY = self.origin[1]
            ptX = ptX + length * cos(start_ang + i * reso)
            ptY = ptY + length * sin(start_ang + i * reso)
            pt = [ptX, ptY]
            
            pts.append(pt)
        
        self.ray_pts = pts
        
    
    """
    Moves the sensor to a different coordinate
    """
    def relocate(self, origin):
        self.origin = origin
        self.set_pts()

    """
    Rotates the sensor by given angle
    +ve angle: counter-clockwise rotation
    -ve angle: clockwise rotation
    """
    def rotate(self, angle):
        self.start_ang += angle
        self.set_pts()
        
    """
    Returns the ray end points
    """    
    def get_pts(self):
        return self.ray_pts