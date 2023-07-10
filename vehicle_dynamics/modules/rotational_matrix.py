from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.utils.Initialization import Initialization

import numpy as np
import logging
import yaml


def rotational_matrix(parameters: Initialization, logger: logging.Logger):
    """
    rotational_matrix calculates the Eutler angle conversion from vehicle system to inertial system. 

    Required Arguments:
        1. x_a
            1.01 roll
            1.02 pitch
        2. angular_vel_2inercial_sys_in_vehicle_coord
        3. rotationalmatrix
        4. angular_rates

    Returns:
        1. rotationalmatrix
        2. angular_vel_2inercial_sys_in_vehicle_coord

    """

    # For the angular velocity of the chassis we have:   Bardini Pag. 261 Eq. 11.5
    parameters.rotationalmatrix = [[-np.sin(parameters.x_a.pitch), 0, 1],
                                   [np.cos(parameters.x_a.pitch) * np.sin(parameters.x_a.roll), np.cos(parameters.x_a.roll), 0],
                                   [np.cos(parameters.x_a.pitch) * np.cos(parameters.x_a.roll), - np.sin(parameters.x_a.roll), 0]]

    # Pitch_dot, roll_dot, yaw_dot -- Bardini Pag. 261 Eq. 11.4
    parameters.angular_rates = np.array([parameters.x_a.wx,
                                         parameters.x_a.wy,
                                         parameters.x_a.wz])  # It is already calculated in the parameters.rotationalmatrix CALCULATE Only once please

    # Coordinate representation of the absolute velocity of point v with respect to coordinate system “E”, described in coordinates of coordinate system “v” bardni. Pag 260
    parameters.angular_vel_2inercial_sys_in_vehicle_coord = parameters.rotationalmatrix @ parameters.angular_rates  # Bardini Pag. 261 Eq. 11.4    

    return parameters, logger 


def main():
    SIM_ITER = 1000
    test_function = rotational_matrix
    function_name = function.__name__

    logger = LocalLogger(function_name).logger

    parameters = Initialization("../../bmw_m8.yaml")
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/2_acc_brake/SimulationData.pickle"

    data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")

    data = [test_function(parameters, logger)[0] for i in range(SIM_ITER)]

    plt.title(function_name)
    plt.plot(data)
    plt.show()


if __name__ == '__main__':
    main()
