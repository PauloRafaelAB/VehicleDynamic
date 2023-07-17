import matplotlib.pyplot as plt
import numpy as np
from vehicle_dynamics.utils.import_data_CM import import_data_CM
path_to_simulation_data = "../../exampledata/lanechange/SimulationData.pickle"
sim_data = import_data_CM(path_to_simulation_data)

sum_wheel_forces = []
for i in range(15003):
    wheel_forces = np.array([[sim_data[i].wheel_load_x_FL, sim_data[i].wheel_load_x_RL, sim_data[i].wheel_load_x_FR, sim_data[i].wheel_load_x_RR],
                             [sim_data[i].wheel_load_y_FL, sim_data[i].wheel_load_y_RL, sim_data[i].wheel_load_y_FR, sim_data[i].wheel_load_y_RR],
                             [sim_data[i].wheel_load_z_FL, sim_data[i].wheel_load_z_RL, sim_data[i].wheel_load_z_FR, sim_data[i].wheel_load_z_RR]])

    sum_wheel_forces.append(np.sum(wheel_forces, axis=1).tolist())


plt.title("forces")
sum_wheel_forces = np.array(sum_wheel_forces)
plt.plot(sum_wheel_forces[:, 0], label="x")
plt.plot(sum_wheel_forces[:, 1], label="y")
plt.plot(sum_wheel_forces[:, 2], label="z")

plt.legend()
plt.show()
