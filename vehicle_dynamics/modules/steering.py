from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.utils.Initialization import Initialization
from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger

import matplotlib.pyplot as plt
import numpy as np
import logging
import yaml 


def steering(parameters: Initialization, logger: logging.Logger, steering_input: float):
    """
    steering is a function that calculates the current angle
    of the frontal wheel based on the vehicle coordinate system,
    where the output delta is steering angle of the frontal wheels.

    Required Parameters from car_parameters:
        1. steering_lock
        2. steering_ratio
    Required Arguments:
        1.steering_input
    Returns:
        1. delta

    """
    # Convert Steering input [-1,1] to whell steering (delta)
    steering_angle = steering_input * parameters.car_parameters.steering_lock 
    parameters.delta = steering_angle / parameters.car_parameters.steering_ratio

    return parameters, logger 


def main():
    test_function = steering
    function_name = test_function.__name__

    logger = LocalLogger(function_name).logger

    parameters = Initialization("../../Audi_r8.yaml", logger=logger)
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/steering data/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []
    steering_angle_v = []
    for i in range(len(sim_data)):
        parameters.x_a.roll = sim_data[i].Vhcl_Roll
        parameters.x_a.pitch = sim_data[i].Vhcl_Pitch
        parameters.x_a.yaw = sim_data[i].Vhcl_Yaw
        steering_angle_calc = sim_data[i].Steering_angle/(4 * np.pi)
        data.append(test_function(parameters, logger, steering_angle_calc)[0].get_data())  
        steering_angle_v.append(steering_angle_calc)
    plt.figure()
    plt.title(function_name)
    plt.plot([i["last_delta"] for i in data], "--", label="delta")
    plt.plot(steering_angle_v, label ="steering_angle_calc")

    var_name = "Steering_angle"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label = var_name)
    plt.legend()

    plt.show()


if __name__ == '__main__':
    main()