# -*- coding: utf-8 -*-
"""
Created on Thu Oct 13 09:45:43 2022

@author: Paulo R.A. Bloemer, Maikol Funk Drechsler
"""
import math
import pandas as pd
import numpy as np
import sys, select, os
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

from VehicleDynamics import VehicleDynamics

points = 100
frequency= 100
maxi = 10000
time_step = 1/frequency
time = np.linspace(0, maxi, points)*time_step


### initialize manuever -> gas pedal, brake pedal and steering wheel angle 
u = np.zeros((4,points)) #
steering = np.zeros(points)
throttle = np.linspace(0., 1, points)
brake = np.zeros(points)
u[1] = throttle


## TODO: Create a structure 
states = np.zeros((15,points))

vehicle_dynamics = VehicleDynamics(initial_speed = 0., state_0 = [0., 0., 0., 0., 0., 0.], initial_gear = 0, freq=frequency, param_path = "bmw_m8.yaml")

for i in range(len(u[0])):
       

    states[:,i] = vehicle_dynamics.tick(throttle[i], brake[i], steering[i],time[i])
    # states[:,i] = vehicle_dynamics.debug_powertrain(throttle[i], brake[i],time[i])
    # states is a vector of output 

# plt.plot(time, throttle, 'b',label='X vel', time, states[6], 'g-',label='X vel',time, states[9],'r', label='Acc x')
plt.plot(time, throttle, 'b',label='throttle')
plt.plot(time, states[0], '4k',label='position')
plt.plot(time, states[3], '1r',label='pitch')
plt.plot(time, states[6], '2y',label='vx')
plt.plot(time, states[9], '--',label='ang vel')
plt.plot(time, states[12], '3m',label='x acc')
plt.xlabel('time (s)')
plt.title('Logitudinal dynamic')
plt.legend()
# plt.plot(time, throttle, 'k', time, states[0], '-',time, states[3],'^')
# plt.plot(time, throttle, 'd', time, states[9], '--',time, states[13],':')
#plt.plot(time, throttle, 'g^',time, states[13],'r')
  
# #plot x, y, z
# fig, axs = plt.subplots(3, 1, figsize=(8, 15))
# axs[0].plot(time, states[:,0], label='x')
# axs[0].plot(time, states[:,1], label='y')
# axs[0].plot(time, states[:,2], label='z')
# axs[0].set_xlabel('time (s)')
# axs[0].set_ylabel('position (m)')
# axs[0].legend()

# # plot x_vel, y_vel, z_vel
# axs[1].plot(time, states[4], label='x_vel')
# axs[1].plot(time, states[5], label='y_vel')
# axs[1].plot(time, states[6], label='z_vel')
# axs[1].set_xlabel('time (s)')
# axs[1].set_ylabel('velocity (m/s)')
# axs[1].legend()


plt.show()