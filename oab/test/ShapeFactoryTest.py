#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 17 19:26:10 2020

@author: rahulr
"""

from oab.shape.ShapeFactory import ShapeFactory
import matplotlib.pyplot as plt

def run():
    num = 3
    shapes = ShapeFactory.getShapes(num) 
    for shape in shapes:
        coords = shape.get_coords()
        x = []
        y = []
        for i in range(len(coords)):
            x.append(coords[i][0])
            y.append(coords[i][1])
        plt.plot(x, y)
   
