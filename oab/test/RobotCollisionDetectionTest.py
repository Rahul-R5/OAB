#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb 21 04:08:45 2021

@author: rahulr
"""

# -*- coding: utf-8 -*-
"""
Created on Sun Jan  3 12:11:05 2021

@author: Rahul R
"""

from math import pi
import matplotlib.pyplot as plt
from celluloid import Camera

from oab.Map import Map
from oab.MapInfo import MapInfo
from oab.robot.RobotInfo import RobotInfo

class RobotCollisionDetectionTest():
    
    def run(self):
        # Get map instance  
        # seed = 10,obstacle = sedd and forth,robot = seed-1
        map = Map(0.5, (100,100), 10)
        robot_pos = [40, 40]
        robot_angle = (90/180) * pi
        robot_size = 3
        MapInfo.setMap(map)
        map.addObstacles(25)
        
        # Create robot
        options = [robot_pos, robot_angle, robot_size]
        map.addRobot(options=options)
        robot = map.robot
        robot.add_sensor(num_rays = 10, ang_range = 359)
        RobotInfo.addRobot(robot)
        
        # Make custom route(No obstacles and ignores obstacles)
        path = [[50, 70], [90, 50], [85, 10], [10, 10]]
        
        # Simulate the robot movement
        for dest in path:
            robot.moveto(dest)
       
        # Get the timestamps and states of the simulation
        global t,x
        t = robot.timestamps
        x = robot.states
        
        #-------------------------Animation-----------------------------------
        # Figure settings
        offset = 5
        size = (7,7)
        
        # Setup figure
        fig = plt.figure(figsize=size)
        ax = fig.add_subplot(111, autoscale_on=False)      
        
        
        # Get camera object
        camera = Camera(fig)
        
        # Loop thorugh states and snapshots
        for state in x:
            robot.set_state(state)
            map.drawState(ax)
            
            # Capture frame
            camera.snap()
        
        # Structure figure
        ax.set_xlim([min(x[:, 0]) - offset, max(x[:, 0]) + offset])
        ax.set_ylim([min(x[:, 1]) - offset, max(x[:, 1]) + offset])
        ax.axis('square')
        ax.set_title("RobotCollisionDetectionTest")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        
        # Animate camera snapshots
        anim = camera.animate(interval = 30, repeat = True, repeat_delay = 500)
        anim.save("RobotCollisionDetectionTest.mp4", 
                  metadata={'artist':'Rahul R'}, 
                  dpi=300, 
                  savefig_kwargs={
                   'pad_inches': 'tight'
                   })
        