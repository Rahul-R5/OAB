#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 17 03:28:25 2021

@author: rahulr
"""
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np


    
fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 6.5)


ax = plt.axes(xlim=(0, 10), ylim=(0, 10))
patch = plt.Circle((5, -5), 0.75, fc='y')
                

def init():
    patch.center = (5, 5)
    ax.add_patch(patch)
    return patch,

def animate(i):
    x, y = patch.center
    x = 5 + 3 * np.sin(np.radians(i))
    y = 5 + 3 * np.cos(np.radians(i))
    patch.center = (x, y)
    return patch,


if __name__ =='__main__':
    anim = animation.FuncAnimation(fig, animate, 
                                   init_func=init, 
                                   frames=360, 
                                   interval=20,
                                   blit=True)
    
    plt.show()