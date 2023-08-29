from vehicle_dynamics.modules.powertrain import Powertrain

from vehicle_dynamics.utils.Initialization import Initialization
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.import_data_CM import import_data_CM

import numpy as np
from geneticalgorithm import geneticalgorithm as ga


class Powertrain_Optimization(object):
    """docstring for Powertrain_Optimization"""

    def __init__(self, param_path, path_to_simulation_data):
        super(Powertrain_Optimization, self).__init__()
        self.logger = LocalLogger("MainLogger").logger
        self.logger.setLevel("INFO")
        self.parameters = Initialization(param_path, logger=self.logger)
        self.powertrain = Powertrain(self.parameters)
        self.sim_data = import_data_CM(path_to_simulation_data)
        self.simulation_range = range(len(self.sim_data))
        self.powertrain_net_torque_CM = np.array([getattr(self.sim_data[i], "wheel_torque_FR") + 
                                                  getattr(self.sim_data[i], "wheel_torque_FL") + 
                                                  getattr(self.sim_data[i], "wheel_torque_RR") + 
                                                  getattr(self.sim_data[i], "wheel_torque_RL") for j, i in enumerate(self.sim_data)])

    def run(self, current_car_parameters):
        self.parameters.car_parameters.gearbox_inertia = current_car_parameters[0]
        self.parameters.car_parameters.engine_inertia = current_car_parameters[1]
        self.parameters.car_parameters.wheel_inertia = [current_car_parameters[2], current_car_parameters[2], current_car_parameters[2], current_car_parameters[2]]
        self.parameters.car_parameters.i_d_shaft = current_car_parameters[3]
        powertrain_torque_calculated = []
        for i in self.simulation_range:
            self.parameters.x_a.vx = self.sim_data[i].Vhcl_PoI_Vel_1_x
            powertrain_torque_calculated.append(self.powertrain.powertrain(self.parameters, self.logger,
                                                                           throttle=self.sim_data[i].gas_pedal,
                                                                           brake=self.sim_data[i].brake_pedal)[0].get_data()["powertrain_net_torque"])

        powertrain_net_torque_calculated = np.array(np.sum(powertrain_torque_calculated, axis=1))

        return np.sqrt(np.mean(np.square(powertrain_net_torque_calculated - self.powertrain_net_torque_CM)))


def main():
    PARAM_PATH = "../../../Audi_r8.yaml"
    PATH_TO_SIMULATION_DATA = "../../../exampledata/lanechange_new/SimulationData.pickle"
    powertrain_optimizator = Powertrain_Optimization(PARAM_PATH, PATH_TO_SIMULATION_DATA)

    varbound = np.array([[0, 2], [1, 5], [15, 40], [0, 1]])

    algorithm_param = {'max_num_iteration': 1000,
                       'population_size': 100,
                       'mutation_probability': 0.1,
                       'elit_ratio': 0.01,
                       'crossover_probability': 0.5,
                       'parents_portion': 0.3,
                       'crossover_type': 'uniform',
                       'max_iteration_without_improv': 50}

    model = ga(function = powertrain_optimizator.run,
               dimension = 4,
               variable_type='real',
               variable_boundaries=varbound,
               algorithm_parameters=algorithm_param)
    model.run()

    convergence = model.report
    solution = model.ouput_dict

    print(convergence, solution)

    with open("solution.powertrain.pickle", "wb+")as handle:
        pickle.dump(model, handle)


if __name__ == '__main__':
    main()
