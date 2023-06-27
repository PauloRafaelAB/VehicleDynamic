#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 23 11:26:40 2023

@author: roadview
"""

from scipy.interpolate import interp1d
from numpy.linalg import inv
import numpy as np
import matplotlib.pyplot as plt

wheel_load_z = 1000
slip_x = np.linspace(-1.,1.,100)
slip_y = 0
b= 10.
c= 1.9
d= 1.2
e=0.97
fx =  d * np.sin(  c * np.arctan(  b*    slip_x -   e * (  b*    slip_x - np.arctan(  b *    slip_x))))

plt.plot(slip_x,fx)


fy = wheel_load_z *   d * np.sin(  c * np.arctan(  b*    slip_y -   e * (  b*    slip_y - np.arctan(  b *    slip_y))))

          

# print('fx',fx)
# print('fy',fy)