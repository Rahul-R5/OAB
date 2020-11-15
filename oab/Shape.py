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
    @param side: length of shape side int
    @param coords: list of int tuples representing the shape's vertices
    @param angle: orientation angle in radians
    '''
    def __init__(self, origin = (0,0), side = 1, angle = 0):
        self.origin = origin
        self.side = side
        self.angle = angle
        self.coords = self.populate_coords()
        
    def get_degrees(self):
        return (180/3.14) * self.angle
        
    @abstractmethod    
    def populate_coords():
        raise NotImplementedError
        