import matplotlib.pyplot as plt
import numpy as np
from vehicle_dynamics.utils.import_data_CM import import_data_CM

"""
    Forces is a module that import the calculated forces in 
    Car Marker to use a benchmark.

    Required Arguments:
        1. wheel_load_x_FL
        2. wheel_load_x_FR
        3. wheel_load_x_RL
        4. wheel_load_x_RR
        5. wheel_load_y_FL
        6. wheel_load_y_FR
        7. wheel_load_y_RL
        1. wheel_load_y_RR
        1. wheel_load_z_FL
        1. wheel_load_z_FR
        1. wheel_load_z_RL
        1. wheel_load_z_RR
    """
path_to_simulation_data = "../../exampledata/acc_brake/SimulationData.pickle"
sim_data = import_data_CM(path_to_simulation_data)

sum_wheel_forces = []
y_forces = []
for i in range(40003):
    wheel_forces = np.array([[sim_data[i].wheel_load_x_FL, sim_data[i].wheel_load_x_RL, sim_data[i].wheel_load_x_FR, sim_data[i].wheel_load_x_RR],
                             [sim_data[i].wheel_load_y_FL, sim_data[i].wheel_load_y_RL, sim_data[i].wheel_load_y_FR, sim_data[i].wheel_load_y_RR],
                             [sim_data[i].wheel_load_z_FL, sim_data[i].wheel_load_z_RL, sim_data[i].wheel_load_z_FR, sim_data[i].wheel_load_z_RR]])
    y_forces .append([sim_data[i].wheel_load_y_FL, sim_data[i].wheel_load_y_RL, sim_data[i].wheel_load_y_FR, sim_data[i].wheel_load_y_RR])
    sum_wheel_forces.append(np.sum(wheel_forces, axis=1).tolist())


plt.title("forces")
sum_wheel_forces = np.array(sum_wheel_forces)
y_forces = np.array(y_forces)
plt.plot(sum_wheel_forces[:, 0], label="x")
plt.plot(sum_wheel_forces[:, 1], label="y")
plt.plot(sum_wheel_forces[:, 2], label="z")

plt.legend()

plt.figure()
plt.plot(y_forces[:, 0], label = "FL")
plt.plot(y_forces[:, 1], label = "RL")
plt.plot(y_forces[:, 2], label = "FR")
plt.plot(y_forces[:, 3], label = "RR")

plt.legend()

plt.show()
