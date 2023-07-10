from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.StateVector import StateVector

import numpy as np
import logging
import yaml


def wheel_slip(parameters: Initialization, logger: logging.Logger):
    """ This method calculate the wheel slip on each wheel. This is done using the relative 
    velocity between the wheel angular speed and the vehicle speed.

        Required Parameters from Param:
            1. r_dyn
        Required Arguments:
            1. x_a
                1.01 vx
                1.02 vy
            4. wheel_w_vel

        Returns:
            1. slip_x
            2. slip_y


    """
    slip_x, slip_y = np.zeros([1, 4]), np.zeros([1, 4])

    vx_4lines = np.ones([1, 4]) * parameters.x_a.vx
    vy_4lines = np.ones([1, 4]) * parameters.x_a.vy

    if (abs(parameters.car_parameters.r_dyn * parameters.wheel_w_vel) == 0) and (abs(parameters.x_a.vx) == 0):
        parameters.slip_x = 0
        parameters.slip_y = 0
    else:
        # equation 11.30 Bardini
        parameters.slip_x = ((((parameters.car_parameters.r_dyn * parameters.wheel_w_vel) - parameters.x_a.vx) / max([abs(parameters.car_parameters.r_dyn * parameters.wheel_w_vel), abs(parameters.x_a.vx)])))         
        # equation 11.31 Bardini
        parameters.slip_y = - np.arctan(parameters.x_a.vy / max([abs(parameters.car_parameters.r_dyn * parameters.wheel_w_vel), abs(parameters.x_a.vx)]))  

    logger.debug("SLIP X ", parameters.slip_x)
    return parameters, logger 


def main():
    SIM_ITER = 1000
    test_function = wheel_slip
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
