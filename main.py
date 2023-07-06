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

frequency= 1000
maxi = 5.0
time = np.linspace(0, maxi, int(maxi*frequency))


### initialize manuever -> gas pedal, brake pedal and steering wheel angle 
points = int(maxi*frequency)
u = np.zeros((4,points)) #
steering = np.zeros(points)
throttle = np.linspace(0., 1, points)
brake = np.zeros(points)
u[1] = throttle


## TODO: Create a structure 
states = np.zeros((22,points))

vehicle_dynamics = VehicleDynamics(initial_speed = 0., state_0 = [0., 0., 0., 0., 0., 0.], initial_gear = 1, freq=frequency, param_path = "bmw_m8.yaml")

for i in range(len(u[0])):
       

    states[:,i] = vehicle_dynamics.tick(throttle[i], brake[i], steering[i],time[i])
    # states[:,i] = vehicle_dynamics.debug_powertrain(throttle[i], brake[i],time[i])
    # states is a vector of output 

# plt.plot(time, throttle, 'b',label='X vel', time, states[6], 'g-',label='X vel',time, states[9],'r', label='Acc x')
# plt.plot(time, throttle, 'b',label='throttle')
# plt.plot(time, states[0], '4k',label='x position')
# plt.plot(time, states[3], '4b',label='roll')
plt.plot(time, states[4], '4c',label='pitch')
# plt.plot(time, states[5], '1g',label='yaw')

#plt.plot(time, states[6], '2y',label='vx')
# plt.plot(time, states[10], '--',label='pitch rate')
#plt.plot(time, states[12], '3m',label='acc x')
plt.plot(time, states[15], 'k',label='gear')
plt.plot(time, states[16], 'b',label='Slip0')
plt.plot(time, states[17], 'r',label='Slip1')
#plt.plot(time, states[18], 'k',label='Slip2')
#plt.plot(time, states[19], 'k',label='Slip3')
#plt.plot(time, states[20], 'g',label='Wheel Speed 0')
#plt.plot(time, states[21], 'm',label='Wheel Speed 1')
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