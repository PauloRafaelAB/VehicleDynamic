from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.utils.Initialization import Initialization


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

        Returns: (parameters)
            1. slip_x
            2. slip_y


    """
    
    long_f = parameters.car_parameters.lv      # x-distance from Vehicle CoG to the front axle [m] 
    long_r = - parameters.car_parameters.lh    # x-distance from Vehicle Cog to the rear axle [m] 
    lat_l  = - parameters.car_parameters.sl    # y-distance from Vehicle CoG to the left chassis point [m]
    lat_r  = parameters.car_parameters.sr      # y-distance from Vehicle CoG to the right chassis point  [m]

    if (abs(parameters.car_parameters.r_dyn * parameters.wheel_w_vel).all() == 0) and (abs(parameters.x_a.vx) == 0):
        parameters.slip_x = np.zeros(4)
    else:
        # equation 11.30 Bardini
        parameters.slip_x = ((((parameters.car_parameters.r_dyn * parameters.wheel_w_vel) - parameters.x_a.vx) / np.maximum(np.absolute(parameters.car_parameters.r_dyn * parameters.wheel_w_vel), np.absolute(parameters.x_a.vx))))         
        # equation 11.31 Bardini
        #parameters.slip_y = - np.arctan(parameters.x_a.vy / np.maximum(abs(parameters.car_parameters.r_dyn * parameters.wheel_w_vel), abs(parameters.x_a.vx)))  
        
    # Refenrence from VTI Y Slip angle
    if(parameters.x_a.vy == 0 and parameters.x_a.wz == 0):
        parameters.slip_y = np.zeros(4)
    else: 
        parameters.slip_y[0] = parameters.last_delta - np.arctan((parameters.x_a.vx + long_f*parameters.x_a.wz)/(parameters.x_a.vy - lat_l*parameters.x_a.wz))  #Front Left
        parameters.slip_y[1] = - np.arctan((parameters.x_a.vx + long_r*parameters.x_a.wz)/(parameters.x_a.vy - lat_l*parameters.x_a.wz))                          #Rear Left
        parameters.slip_y[2] = parameters.last_delta - np.arctan((parameters.x_a.vx + long_f*parameters.x_a.wz)/(parameters.x_a.vy - lat_r*parameters.x_a.wz))  #Front Right
        parameters.slip_y[3] = - np.arctan((parameters.x_a.vx + long_r*parameters.x_a.wz)/(parameters.x_a.vy - lat_r*parameters.x_a.wz))                          #Rear Right
               
        #parameters.slip_y = 0
    logger.debug(f"SLIP X  {parameters.slip_x}")
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
