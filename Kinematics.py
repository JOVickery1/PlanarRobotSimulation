#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 22 16:48:37 2023

@author: jamesvickery
"""
import numpy as np

def IK(
     point,
     L1=1,
     L2=1,
):
  """Calculates the inverse kinematics of 2 link planar manipulator

    Args:
        point: An array of the X and Y coords of the End Effector.
        L1: The length of link 1
        L2: The length of link 2

    Returns:
        (array): An array of the joint angles theta1 and theta2 [theta1,theta2]
    """
  XE=point[0]
  YE=point[1]

  sigma1 = np.sqrt(-L1**4 + 2*L1**2*L2**2 + 2*L1**2*XE**2 + 2*L1**2*YE**2 - L2**4 + 2*L2**2*XE**2 + 2*L2**2*YE**2 - XE**4 - 2*XE**2*YE**2 - YE**4) #This is the sigma from MatLAB
  theta1 = 2*np.arctan2(2*L1*YE+sigma1, L1**2+2*L1*XE-L2**2+XE**2+YE**2) #This is the inverse kinematic equation for theta 1

  #theta1 = np.arctan((2*L1*YE+sigma1)/(L1**2+2*L1*XE-L2**2+XE**2+YE**2)) #Inverse kinematics using just regular arctan

  sigma2 = np.sqrt((-L1**2 + 2*L1*L2 - L2**2 + XE**2 + YE**2)*(L1**2 + 2*L1*L2 + L2**2 - XE**2 - YE**2)) #This is ONLY the square root from sigma1 from theta2 in MatLAB code
  theta2 = -2*np.arctan2(sigma2, -L1**2 + 2*L1*L2 - L2**2 + XE**2 + YE**2) #This is the inverse kinematics for theta 2

  #theta2 = -np.arctan(sigma2/(-L1**2 + 2*L1*L2 - L2**2 + XE**2 + YE**2)) #Inverse kinematics using just regular arctan

  #theta1_alternate = theta1+np.pi #uncomment both and add to return if using regular arctan
  #theta2_alternate = theta2+np.pi

  return theta1, theta2

def FK(
    thetas,
    L1 = 1, 
    L2 = 1,
    n = 5, 
):
  """Calculates the forward kinematics of 2 link planar manipulator

    Args:
        thetas: An array of the two joint variables (theta1 and theta2).
        L1: The length of link 1
        L2: The length of link 2
        n: The number of decimal points to be precise to

    Returns:
        (array): The X and Y position of the end effector as an array [XE,YE]
    """
  XE = L1*np.cos(thetas[0])+L2*np.cos(thetas[0]+thetas[1])
  YE = L1*np.sin(thetas[0])+L2*np.sin(thetas[0]+thetas[1])

  XE = round(XE,n)
  YE = round(YE,n)

  return XE, YE

# def IK_fproj(
#         ):
    
    
    