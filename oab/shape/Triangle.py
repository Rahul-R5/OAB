#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 17 13:52:47 2020

@author: rahulr
"""
from oab.shape.Shape import Shape
from math import sin,cos

class Triangle(Shape):
    
    def populate_coords(self):
        size = self.size
        reso = (2 * 3.14/3)
        ptX = self.origin[0]
        ptY = self.origin[1]
        start_ang = self.angle
        
        coords = []
        for i in range(3):
            X = ptX + size * cos(start_ang + i * reso)
            Y = ptY + size * sin(start_ang + i * reso)
            coords.append([X, Y])
        
        return coords