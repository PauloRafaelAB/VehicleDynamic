from vehicle_dynamics.modules.powertrain import Powertrain

from vehicle_dynamics.utils.Initialization import Initialization
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.import_data_CM import import_data_CM

import numpy as np
from geneticalgorithm import geneticalgorithm as ga


class Roll_Optimization(object):
    """docstring for Roll_Optimization"""

    def __init__(self, param_path, path_to_simulation_data):
        super(Roll_Optimization, self).__init__()
        self.logger = LocalLogger("MainLogger").logger
        self.logger.setLevel("INFO")

        self.parameters = Initialization(param_path, logger=self.logger)
        
        
        
        
        self.chassis_rotation = Chassis_rotation(self.parameters)
        self.sim_data = import_data_CM(path_to_simulation_data)
        self.simulation_range = range(len(self.sim_data))
        
        
        
        
        #parameter to be optmized
        self.x_a.roll = np.array([getattr(self.sim_data[i], "roll") for i in enumerate(self.sim_data)])

    def run(self, current_car_parameters):
        self.parameters.car_parameters.k_roll = current_car_parameters[0] # rewrite right side of the equation
        self.parameters.car_parameters.k_pitch = current_car_parameters[1]
        self.parameters.car_parameters.i_x_s = current_car_parameters[2]
        self.parameters.car_parameters.i_y_s = current_car_parameters[3]

        powertrain_torque_calculated = []

        for i in self.simulation_range:
            self.parameters.x_a.vx = self.sim_data[i].Vhcl_PoI_Vel_1_x
            powertrain_torque_calculated.append(self.powertrain.powertrain(self.parameters, self.logger,
                                                                           throttle=self.sim_data[i].gas_pedal,
                                                                           brake=self.sim_data[i].brake_pedal)[0].get_data()["powertrain_net_torque"])

        powertrain_net_torque_calculated = np.array(np.sum(powertrain_torque_calculated, axis=1))


        return np.sqrt(np.mean(np.square(roll - self.powertrain_net_torque_CM))) # comparison of variable of interest


def main():
    PARAM_PATH = "../../../Audi_r8.yaml"
    PATH_TO_SIMULATION_DATA = "../../../exampledata/lanechange_new/SimulationData.pickle"
    roll_optimizator = Roll_Optimization(PARAM_PATH, PATH_TO_SIMULATION_DATA)

    varbound = np.array([[7000, 13000], [5000, 12000], [100, 1000], [1000, 2000]])

    algorithm_param = {'max_num_iteration': 500,
                       'population_size': 50,
                       'mutation_probability': 0.1,
                       'elit_ratio': 0.01,
                       'crossover_probability': 0.5,
                       'parents_portion': 0.3,
                       'crossover_type': 'uniform',
                       'max_iteration_without_improv': 50}

    model = ga(function = roll_optimizator.run,
               dimension = 4,
               variable_type='real',
               variable_boundaries=varbound,
               algorithm_parameters=algorithm_param)
    model.run()

    convergence = model.report
    solution = model.ouput_dict

    print(convergence, solution)

    with open("solution.chassis_rotation.pickle", "wb+")as handle:
        pickle.dump(model, handle)


if __name__ == '__main__':
    main()