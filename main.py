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


frequency= 100
time_step = 1/frequency
time = np.linspace(0, 100, 100)*time_step


### initialize manuever -> gas pedal, brake pedal and steering wheel angle 
u = np.zeros((4,100)) #
steering = np.zeros(100)
throttle = np.linspace(0, 1, 100)
brake = np.zeros(100)
u[1] = throttle


## TODO: Create a structure 
states = np.zeros((13,100))

vehicle_dynamics = VehicleDynamics(initial_speed = 0., state_0 = [0., 0., 0., 0., 0., 0.], initial_gear = 0, freq=frequency, param_path = "bmw_m8.yaml")

for i in range(len(u[0])):
       
    #states[:,i] = vehicle_dynamics.tick(throttle[i], brake[i], steering[i])
    states[:,i] = vehicle_dynamics.debug_powertrain(throttle[i], brake[i], steering[i])
    # states is a vector of output 

plt.plot(time, throttle, 'g^', time, states[7], 'g-')
  
 # plot x, y, z
   # fig, axs = plt.subplots(3, 1, figsize=(8, 12))
   # axs[0].plot(t, sol[:,0], label='x')
   # axs[0].plot(t, sol[:,1], label='y')
   # axs[0].plot(t, sol[:,2], label='z')
   # axs[0].set_xlabel('time (s)')
   # axs[0].set_ylabel('position (m)')
   # axs[0].legend()

   # # plot x_vel, y_vel, z_vel
   # axs[1].plot(t, sol[:,3], label='x_vel')
   # axs[1].plot(t, sol[:,4], label='y_vel')
   # axs[1].plot(t, sol[:,5], label='z_vel')
   # axs[1].set_xlabel('time (s)')
   # axs[1].set_ylabel('velocity (m/s)')
   # axs[1].legend()

   # # plot psi_dot, theta_dot, phi_dot
   # axs[2].plot(t, sol[:,6], label='psi')
   # axs[2].plot(t, sol[:,7], label='theta')
   # axs[2].plot(t, sol[:,8], label='phi')
   
   # axs[2].plot(t, sol[:,9], label='psi_dot')
   # axs[2].plot(t, sol[:,10], label='theta_dot')
   # axs[2].plot(t, sol[:,11], label='phi_dot')
   # axs[2].set_xlabel('time (s)')
   # axs[2].set_ylabel('angular velocity (rad/s)')
   # axs[2].legend()

   # plt.show()