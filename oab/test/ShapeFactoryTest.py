#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 17 19:26:10 2020

@author: rahulr
"""

from oab.shape.ShapeFactory import ShapeFactory
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import numpy as np
from oab.Map import Map
from oab.MapInfo import MapInfo
class ShapeFactoryTest():
    def run(self):
        num = 5
        mapInstance = Map(0.5, (50,50), 9)
        MapInfo.setMap(mapInstance)
        fig, ax = plt.subplots()
        shapes = ShapeFactory.getShapes(num) 
        
        for shape in shapes:
            coords = shape.get_coords()
            points = np.zeros((len(coords),2))
            for i in range(len(coords)):
                points[i][0] = coords[i][0]
                points[i][1] = coords[i][1]
    
            self.draw(fig, ax, num, points)
        plt.title("ShapeFactoryTest")
        plt.show()
    
    def draw(self, fig, ax, num, points):
        
        patches = []
        num_polygons = 1
        mapInstance = ShapeFactory.mapInstance
        
        polygon = Polygon(points, closed = True)
        patches.append(polygon)
    
        xmax = mapInstance.rows
        ymax = mapInstance.columns
        ax = plt.gca()
        ax.add_patch(polygon)
        ax.set_xlim(-10,xmax)
        ax.set_ylim(-10,ymax)
