from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.AngularWheelPosition import AngularWheelPosition
from vehicle_dynamics.utils.Initialization import Initialization


import numpy as np
import logging
import yaml


def wheel_angular(parameters: Initialization, logger: logging.Logger):
    """
    wheel_angular velocities velocities are used to calcule slip

    Required Parameters from Param:
        1. r_dyn #dynamic radius
        2. iw # wheel inertia

    Required Arguments:
        1. powertrain_net_torque
        2.x_rf 
           2.01 fx 
       3. x_rr
           3.01 pho_r_2dot # wheel angular acceleration
       4. wheel_w_vel # wheel angular velocity

    Returns:
        1. wheel_w_vel
        2. x_rr

    """

    # EULER’s Equations of the Wheels: Solve  11-25 Bardini pag 266
    # Torque, fx_car, fy_car, Inertias wheel,  output wheel Wv, dot_wv
    parameters.x_rr.pho_r_2dot = (parameters.powertrain_net_torque / parameters.car_parameters.wheel_inertia) - ((parameters.x_rf.fx * parameters.car_parameters.r_dyn) / parameters.car_parameters.wheel_inertia) 

    logger.debug(f'torque fx { parameters.x_rf.fx / parameters.car_parameters.r_dyn}')
    logger.debug(f'powertrain net torque {parameters.powertrain_net_torque}')

    logger.debug(f"pho_r_2dot     {parameters.x_rr.pho_r_2dot}")
    logger.debug(f"FX_________ {parameters.x_rf.fx}")

    # Calculate the integral of the wheel_acc to calculate slip_x
    parameters.wheel_w_vel = parameters.wheel_w_vel + (parameters.x_rr.pho_r_2dot * parameters.time_step)  # rad/s

    return parameters, logger


def main():
    SIM_ITER = 1000
    test_function = wheel_angular
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
