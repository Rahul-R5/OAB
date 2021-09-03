# -*- coding: utf-8 -*-
"""
Created on Thu Jan  7 11:19:33 2021

@author: Rahul R
"""
from math import pi
import matplotlib.pyplot as plt
from oab.Map import Map
from oab.MapInfo import MapInfo
from matplotlib import animation

t = None
x = None
patches = None

#---------------------------Simulation----------------------------------------
class RobotMovementTest():
    
    def run(self):
        
        mapInstance = Map(0.5, (100,100), 8)
        robot_pos = [40, 40]
        robot_angle = (90/180) * pi
        MapInfo.setMap(mapInstance)
        #mapInstance.addObstacles(3)
        
        # Create robot
        options = [robot_pos, robot_angle]
        mapInstance.addRobot(options=options)
        robot = mapInstance.robot
        
        # Make custom route(No obstacles and ignores obstacles)
        path = [[40, 50], [30, 50], [30, 40], [20, 40]]
        
        # Simulate the robot movement
        for dest in path:
            robot.moveto(dest)
        
        # Get the timestamps and states of the simulation
        global t,x
        t = robot.timestamps
        x = robot.states
        
        # Get the patches for animation
        global patches
        patches = mapInstance.getPatches(x)
        
        
# ----------------------------Animation---------------------------------------

# Figure setup
# Variables
offset = 0.2

# Get test object and run
test = RobotMovementTest()
test.run()

# Create figure
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, 
                      xlim=([min(x[:, 0]) - offset, max(x[:, 0]) + offset]), 
                      ylim=([min(x[:, 1]) - offset, max(x[:, 1]) + offset]))
ax.axis('square')
ax.set_title("RobotMovementTest")
ax.set_xlabel("x")
ax.set_ylabel("y")
        
def init():
    # initialize with obstacles if any
    frame = []
    # if len(patches[0]) > 0:
    #     for patch in patches[0]:
    #         frame.extend(ax.add_patch(patch))
  
    patch = patches[1][0][0]
    ax.add_patch(patch)
    

    return patch,

def animate(i):
    # draw circles, select to color for the circles based on the input argument i. 
    #someColors = ['r', 'b', 'g', 'm', 'y']
    frame = []
    # if len(patches[0]) > 0:
    #   for itr in range(len(patches[0])):
    #       obstacle_patch = patches[0][itr] 
    #       ax.add_patch(obstacle_patch)
    # #for patch in patches[i+1][0]:        
    # for j in range(0,len(patches[i+2][0])):
    #     patch = patches[i+2][0][j]
    #     if j == 0:
    #         frame.extend(ax.add_patch(patch))
    #     else:
    #         #frame.extend(ax.add_line(patch))
    #         continue
    patch = patches[i+1][0][0]
    ax.add_patch(patch)
    
    return patch,

# frames = []
# for i in range(len(t)):
#     patch = animate(i)
#     frames.append(patch) 
#anim = animation.ArtistAnimation(fig, frames, repeat=False)    
anim = animation.FuncAnimation(fig, animate, init_func=init,
                            frames=len(t), interval=30, blit=True)
plt.show()
    

#--------------------------------Outdated-------------------------------------    
    ## Original line plot code
    
    # plt.figure(figsize = (7,7))
    # plt.plot(x[:, 0], x[:, 1])
    # plt.axis("equal")
    # plt.xlabel('x')
    # plt.ylabel('y')
    # plt.title("RobotMovementTest")
    # plt.xlim([min(x[:, 0]) - offset, max(x[:, 0])+offset])
    # plt.ylim([min(x[:, 1])-  offset, max(x[:, 1])+offset])
    # plt.grid(True)
    # plt.show()