"""
Created on Thu Oct 13 09:45:43 2022

@author: Maikol Funk Drechsler, Yuri Poledna, Paulo R.A. Bloemer
"""
from matplotlib.pyplot import axes
from vehicle_dynamics.structures.OutputStates import OutputStates
from vehicle_dynamics.utils.plot_function import plot_function
from vehicle_dynamics.structures.Manoeuvre import Manoeuvre
from vehicle_dynamics.utils.import_data_CM import import_data_CM


from VehicleDynamics import VehicleDynamics

import numpy as np
import tqdm

frequency = 1000
end_time = 20
points = int(end_time * frequency)

time = np.linspace(0, end_time, points)

# initialize manuever -> gas pedal, brake pedal and steering wheel angle 
steering = 0.0 * np.ones(points)  
throttle = 1. * (np.ones(points))  
brake = np.zeros(points)

manoeuvre = Manoeuvre(steering, throttle, brake, time)

path_to_simulation_data = "exampledata/Lanechange_new/SimulationData.pickle"
sim_data = import_data_CM(path_to_simulation_data)

points = len(sim_data)

time = np.linspace(0, points / frequency, points)

steering = [sim_data[i].Steering_angle / (4 * np.pi) for i in sim_data]
throttle = [sim_data[i].gas_pedal for i in sim_data]
brake = [sim_data[i].brake_pedal for i in sim_data]

manoeuvre_carMaker = Manoeuvre(steering, throttle, brake, time)

output_states = OutputStates()

vehicle_dynamics = VehicleDynamics(state_0 = np.zeros(15), initial_gear = 1, freq=frequency, param_path = "Audi_R8.yaml")

for i in tqdm.tqdm(range(points)):
    output_states.set_states(vehicle_dynamics.tick(*manoeuvre_carMaker[i]))

plot_function(output_states, manoeuvre_carMaker, sim_data)
