#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 15 22:43:44 2020

@author: rahulr
"""

from abc import ABC, abstractmethod 

class Shape(ABC):
    '''
    @param origin: tuple of origin coordinates (int, int)
    @param sides: no of sides of the shape
    @param coords: list of int tuples representing the shape's vertices
    @param angle: orientation angle in radians
    '''
    def __init__(self, origin, sides, angle, size = 3):
        self.origin = origin
        self.sides = sides
        self.size = size
        self.angle = angle
        self.coords = self.populate_coords()
        
    def get_degrees(self):
        return (180/3.14) * self.angle
    
    def get_coords(self):    
        return self.coords
    
    @abstractmethod    
    def populate_coords(self):
        raise NotImplementedError
        