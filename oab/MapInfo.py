#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 19 10:56:28 2020

@author: rahulr
"""

class MapInfo():
    
    mapInstance = None
    
    @staticmethod
    def setMap(mapInstance):
        MapInfo.mapInstance = mapInstance
        
    @staticmethod
    def getMap():
        if MapInfo.mapInstance  is not None:
            return MapInfo.mapInstance 
        else:
            return -1   # TODO: Error handling