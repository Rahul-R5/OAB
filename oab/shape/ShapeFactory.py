#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 00:25:09 2020

@author: rahulr
"""
from oab.Map import Map
from oab import MapInfo
from oab.shape.Square import Square
from oab.shape.Pentagon import Pentagon
from oab.shape.Triangle import Triangle
import random



class ShapeFactory():
    """
    @param seed: seed for random function, int
    @param numObjects: number of objects initialised through factory, int    
    """
    seed = 8
    numObjects = 0
    mapInstance = None  # Have a map class variable (i.e static variable)
    
    """
    Returns 'num' amount of shapes(obstacles) with random properties
    """
    @staticmethod
    def getShapes(num):

        # TODO 
        # In future, avoid clipping obstacles with a mapInstance
        if ShapeFactory.mapInstance is None:
            ShapeFactory.mapInstance = MapInfo.getMap()
        
        # generate 'num' amount of objects and return them
        # increment the static object counter
        shapes = []
        for i in range(num):
            origin = ShapeFactory.getRandomPos()
            angle = ShapeFactory.getRandomAngle()
            shape = Square(origin, 4, angle)
            shapes.append(shape)
        
        ShapeFactory.numObjects += num
        return shapes    
        
            
    @staticmethod
    def getRandomPos():
        posX = random.randint(0, ShapeFactory.mapInstance.rows)
        posY = random.randint(0, ShapeFactory.mapInstance.columns)   
        
        return (posX, posY)
     
    """  
    Randomly selects the side of the shape needed from available shape
    """
    @staticmethod
    def getRandomSide():
        raise NotImplementedError
        
    @staticmethod
    def getRandomAngle():
        angle = random.randint(0, 359)
        angle = (angle/180) * 3.14  # Convert to radians
        
        return angle