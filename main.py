# -*- coding: utf-8 -*-
"""
Created on Thu Oct 13 09:45:43 2022

@author: Paulo R.A. Bloemer, Maikol Funk Drechsler, Yuri Poledna
"""
from vehicle_dynamics.structures.OutputStates import OutputStates
from vehicle_dynamics.utils.plot_function import plot_function
from vehicle_dynamics.structures.Manoeuvre import Manoeuvre

from VehicleDynamics import VehicleDynamics

import numpy as np
import tqdm

frequency = 1000
end_time = 10.0
points = int(end_time * frequency)

time = np.linspace(0, end_time, points)

# initialize manuever -> gas pedal, brake pedal and steering wheel angle 
steering = np.zeros(points)
throttle = np.linspace(0, 1, int(5000)).tolist()
throttle.extend(np.ones(5000).tolist())
brake = np.zeros(points)

manoeuvre = Manoeuvre(steering, throttle, brake, time)

output_states = OutputStates()

vehicle_dynamics = VehicleDynamics(state_0 = np.zeros(15), initial_gear = 1, freq=frequency, param_path = "Audi_R8.yaml")

for i in tqdm.tqdm(range(points)):
    output_states.set_states(vehicle_dynamics.tick(*manoeuvre[i]))

plot_function(output_states, manoeuvre)
