# -*- coding: utf-8 -*-
"""
Created on Thu Oct 13 09:45:43 2022

@author: Paulo R.A. Bloemer, Maikol Funk Drechsler, Yuri Poledna
"""
import pickle
import matplotlib.pyplot as plt
from vehicle_dynamics.structures.OutputStates import OutputStates
from vehicle_dynamics.utils.plot_function import plot_function
from vehicle_dynamics.structures.Manoeuvre import Manoeuvre

from VehicleDynamics import VehicleDynamics


import numpy as np
import tqdm
import time

frequency = 1000
end_time = 10.0
points = int(end_time * frequency)

timeseries = np.linspace(0, end_time, points)

# initialize manuever -> gas pedal, brake pedal and steering wheel angle 
steering = np.zeros(points)
throttle = np.linspace(0, 1, int(5000)).tolist()
throttle.extend(np.ones(5000).tolist())
brake = np.ones(points)

manoeuvre = Manoeuvre(steering, throttle, brake, timeseries)

output_states = OutputStates()

vehicle_dynamics = VehicleDynamics(state_0 = np.zeros(15), initial_gear = 1, freq=frequency, param_path = "Audi_R8.yaml")

looping_time = np.ones(points)

# for i in tqdm.tqdm(range(points)):
for i in range(points):
    start_time = time.monotonic_ns()
    output_states.set_states(vehicle_dynamics.tick(*manoeuvre[i]))
    looping_time[i] = time.monotonic_ns() - start_time

#plot_function(output_states, manoeuvre)

with open("timing.pickle", "wb+")as handle:
    pickle.dump(looping_time, handle)

plt.plot(looping_time * 1e-6)
plt.xlabel("Iteration Number")
plt.ylabel("Time taken per iteration (ms)")
plt.title(f"Tick's Time per Iteration for a {end_time}s simulation")
plt.show()
