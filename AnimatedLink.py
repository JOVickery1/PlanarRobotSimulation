#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 23 10:22:02 2023

@author: James Vickery

Correctly animates a 2 link planar manipulator moving between two points on a given (straight line) path
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

#from Link import *
from Kinematics import*

#%% Initilizing the figure

fig, ax = plt.subplots()

fig.set_size_inches(8, 8)


#%% Defining paramaters of the robot (link lenghts)

L1=1
L2=1

#%% Defining path parameters (modify as desired)

# Starting and final positions
initial_point = [1.5,1] #Define the initial position of the end effector
final_point = [0.25,0.25] #Define the final position of the end effector

# Desired number of interpolated points
N = 100 

#%% Calculating derived quantities based on inputs

m = (final_point[1]-initial_point[1])/(final_point[0]-initial_point[0])

b = -m*initial_point[0]+initial_point[1]

#%% Calculating interpolated points

# Divides up the x-values of desired path into N equal slices
xcoords = np.linspace(initial_point[0],final_point[0],N) 
# Equation of the line between initial and final point in slope intecept form
ycoords = m*xcoords + b 

d = {'xcoords':xcoords, 'ycoords':ycoords}

coords = pd.DataFrame(data=d)

#%% Calculating the inverse kinematics using the IK function 
  # defined in Kinematics.py

theta1= np.linspace(0,0,N)
theta2= np.linspace(0,0,N)

for i in range(0,N):
   theta1[i] = IK(coords.loc[i],L1,L2)[0]
   theta2[i] = IK(coords.loc[i],L1,L2)[1]

d1 = {'theta1':theta1,'theta2':theta2}
thetas =pd.DataFrame(data=d1)

#Initializing empty arrays to optimize memory usage

x0= np.linspace(0,0,N)
y0= np.linspace(0,0,N)

x1=np.linspace(0,0,N)
y1=np.linspace(0,0,N)

x2=np.linspace(0,0,N)
y2=np.linspace(0,0,N)


for i in range(0,N):
  x1[i] = L1*np.cos(theta1[i])
  x2[i] = FK(thetas.loc[i],L1,L2)[0]
  y1[i] = L1*np.sin(theta1[i])
  y2[i] = FK(thetas.loc[i],L1,L2)[1]


#%% Animation

# link 1
line,= ax.plot([x0[0],x1[1]],[y0[0],y1[1]]) 
# link 2
line2,= ax.plot([x1[0],x2[1]],[y1[0],y2[1]]) 

# static straight line path between the two desired points
line3, = ax.plot([xcoords[0],xcoords[-1]],[ycoords[0],ycoords[-1]]) 

# Workspace boundaries
center = [0,0]
circle = plt.Circle((0,0),(L1+L2),facecolor='none',edgecolor='blue')


# animate(i) updates the plot

def animate(i):
    if i<N:
        line.set_xdata([x0[i],x1[i]])  # update the data for link 1 in x
        line.set_ydata([y0[i],y1[i]])  # update the data for link 1 in y
        line2.set_xdata([x1[i],x2[i]])  # update the data for link 2 in x
        line2.set_ydata([y1[i],y2[i]])  # update the data for link 2 in y
    return line, line2, line3, 


# Init only required for blitting to give a clean slate.
def init():
    # Defining the plot size based on the length of the robot links
    ax.set_xlim(-(L1+L2),(L1+L2))
    ax.set_ylim(-(L1+L2),(L1+L2))
    # Adding in a circle showing workspace boundaries
    circle.center=([0,0])
    ax.add_patch(circle)
    return line, line2, line3,

ani = animation.FuncAnimation(fig, animate, np.arange(1, 200,), init_func=init,
                              interval=50, blit=True)
plt.show()