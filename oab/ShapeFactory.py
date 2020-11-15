#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 00:25:09 2020

@author: rahulr
"""

class ShapeFactory():
    """
    @param seed: seed for random function, int
    @param numObjects: number of objects initialised through factory, int    
    """
    numObjects = 0
    mapInstance = None  # Have a map class variable (i.e static variable)
    
    @staticmethod
    def getShapes(num):
        # use static variable of map to check for collision when making new random
        # objects
        # generate 'num' amount of objects and return them
        # increment the static object counter
        raise NotImplementedError
        
    @staticmethod
    def getRandomPos():
        raise NotImplementedError
        
    @staticmethod
    def getRandomSide():
        raise NotImplementedError
        
    @staticmethod
    def getRandomeAngle(num):
        raise NotImplementedError