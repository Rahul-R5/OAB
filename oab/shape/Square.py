#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 17 14:01:16 2020

@author: rahulr
"""

from oab.shape.Shape import Shape
from math import sin,cos

class Square(Shape):
    
    def populate_coords(self):
        sides = self.sides
        reso = (2 * 3.14/4)
        ptX = self.origin[0]
        ptY = self.origin[1]
        start_ang = self.angle
        
        coords = []
        for i in range(4):
            X = ptX + sides * cos(start_ang + i * reso)
            Y = ptY + sides * sin(start_ang + i * reso)
            coords.append([X, Y])
        
        return coords