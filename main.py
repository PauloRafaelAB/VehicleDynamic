# -*- coding: utf-8 -*-
"""
Created on Thu Oct 13 09:45:43 2022

@author: Paulo R.A. Bloemer, Maikol Funk Drechsler, Yuri Poledna
"""
from matplotlib.pyplot import axes
from vehicle_dynamics.structures.OutputStates import OutputStates
from vehicle_dynamics.utils.plot_function import plot_function
from vehicle_dynamics.structures.Manoeuvre import Manoeuvre

from VehicleDynamics import VehicleDynamics

import numpy as np
import tqdm

frequency = 1000
end_time = 20
points = int(end_time * frequency)

time = np.linspace(0, end_time, points)

# initialize manuever -> gas pedal, brake pedal and steering wheel angle 
#a = np.ones(points)* 0.3
steering =  np.ones(20000).tolist()
throttle = np.ones(20000).tolist()#linspace(0.0, 1, int(5000)).tolist()
#throttle.extend(np.ones(15000).tolist())
brake = np.zeros(points)

manoeuvre = Manoeuvre(steering, throttle, brake, time)

output_states = OutputStates()

vehicle_dynamics = VehicleDynamics(state_0 = np.zeros(15), initial_gear = 1, freq=frequency, param_path = "Audi_R8.yaml")

THROTTLE_GRACE_PERIOD = 10
current_throttle_grace = 0
for i in tqdm.tqdm(range(points)):
    output_states.set_states(vehicle_dynamics.tick(*manoeuvre[i]))
    vx = output_states[-1].x_a.vx

    try:
        if current_throttle_grace>0:
            current_throttle_grace-=1
            manoeuvre.throttle[i+1] = last_changed_velocity
            continue

        if vx >=11:
            
            manoeuvre.throttle[i+1] =manoeuvre.throttle[i]*0.5
            last_changed_velocity =manoeuvre.throttle[i+1]
            current_throttle_grace = THROTTLE_GRACE_PERIOD
        elif vx <= 9:
            manoeuvre.throttle[i+1] =manoeuvre.throttle[i]*1.1
            current_throttle_grace = THROTTLE_GRACE_PERIOD
            last_changed_velocity = manoeuvre.throttle[i+1]
        if manoeuvre.throttle[i+1]>1:
            manoeuvre.throttle[i+1]=1
            last_changed_velocity=1
    except IndexError:
        pass
        
      


plot_function(output_states, manoeuvre)

