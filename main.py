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
u = np.zeros((4,100)) ##3
steering = np.zeros(100)
throttle = np.linspace(0, 1, 100)
brake = np.zeros(100)
u[1] = throttle


## TODO: Create a structure 
states = np.zeros((13,100))

vehicle_dynamics = VehicleDynamics(initial_speed = 0., state_0 = [0., 0., 0., 0., 0., 0.], initial_gear = 0, freq=frequency, param_path = "bmw_m8.yaml")

for i in range(len(u[0])):
       
    states[:,i] = vehicle_dynamics.tick(throttle[i], brake[i], steering[i])
    

plt.plot(time, throttle, 'g^', time, states[7], 'g-')