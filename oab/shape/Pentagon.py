#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 17 14:02:18 2020

@author: rahulr
"""

from oab.shape import Shape
from math import sin,cos

class Pentagon(Shape):
    
    def populate_coords(self):
        sides = self.sides
        reso = (3.14/sides)
        ptX = self.origin[0]
        ptY = self.origin[1]
        start_ang = self.angle
        
        coords = []
        for i in range(sides):
            ptX = ptX + sides * cos(start_ang + i * reso)
            ptY = ptY + sides * sin(start_ang + i * reso)
            coords.append((ptX, ptY))
        
        return coords