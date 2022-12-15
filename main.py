# -*- coding: utf-8 -*-
"""
Created on Thu Oct 13 09:45:43 2022

@author: Paulo R.A. Bloemer
"""

#Import all the lyb; 

import math
import pandas as pd
import numpy as np
import sys, select, os
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from BMW_M8 import Parameters as P
from VehicleDynamics import VehicleDynamics

##TODO: Import class;  


## Initialize rpm = minrpm, gear = 1, initial position x and y -> hardcoded in the beginning, timestep, (Object Old) 

gear = 0
#position = [0, 0, 0]        # meters
t_step = 0.01               # seconds


#vehicle_dynamics = VehicleDynamics.
vehicle_dynamics = VehicleDynamics(min_rpm = 1500., initial_vel = 0., position_0 = [0, 0, 0], throttle=0.0, steer=0.0, brake=0.0, gear = 0)



def func_KS(x, t, u, p):
    f = vehicle_dynamics_ks(x, u, p)
    return f

tStart = 0  # start time
tFinal = 5  # start time

# load vehicle parameters
p = Parameters_bmw_m8()

# initial state for simulation
delta0 = 0
vel0 = 15
Psi0 = 0
sy0 = 0

initialState = [0, sy0, delta0, vel0, Psi0]
t0_state = init_ks(initialState)

t = numpy.arange(0, tFinal, 0.01)
u = [0, 5]
x = odeint(func_KS, t0_state, t, args=(u, p))



#Read real vehicle data.

manouver_input = [0,0,0,0,0,0,0,0,0,1,1,1,1,1,1]

#TODO: vecto of desired torques>>> a matrix with all inputs can be used

lenght_vector = 10000

des_torque = np.zeros(lenght_vector)

for i in range(len(des_torque)):
    if i > (lenght_vector/2):
        des_torque[i] = 1.
        
des_brake = np.zeros(lenght_vector)

plt.plot(time_stemp, x)
plt.show()

plt.plot(time_stemp, x_dot)

plt.plot(time_stemp, x_dot_dot) 
         

#ODO:des ...



# TODO: make vector_x (0,0); 
# TODO: vector_y;
# TODO: vector_vel;


# TODO: Improve to  

for vector in range(len(vector)):
    

# Read input (des engine torque, des brake torque; orientation)
    
    
  #   this is a method to avoid the the parameters needs to be re-passed
    
    
    
  #   vehicle_dynamics.longitudinal(d_engine_torque, d_brake_toque)
    
  #   vehicle_dynamics.lateral(orientation)
    
  #   vector_x[i] = vehicle_dynamics.x

  #   same for the othe vectors
    
  # plot