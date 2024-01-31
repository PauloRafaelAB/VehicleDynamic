from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.AngularWheelPosition import AngularWheelPosition

from vehicle_dynamics.utils.Initialization import Initialization
from vehicle_dynamics.utils.import_data_CM import import_data_CM


from vehicle_dynamics.utils.LocalLogger import LocalLogger
import matplotlib.pyplot as plt
import numpy as np
import logging
import yaml


def wheel_angular(parameters: Initialization, logger: logging.Logger):
    """
    wheel_angular velocities velocities are used to calcule slip

    Required Parameters from car_parameters:
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
    parameters.x_rr.pho_r_2dot = (parameters.powertrain_net_torque - (parameters.x_rf.fx * parameters.car_parameters.r_dyn / 2)) / (parameters.car_parameters.wheel_inertia)

    logger.debug(f'torque fx { parameters.x_rf.fx / parameters.car_parameters.r_dyn}')

    logger.debug(f"FX{parameters.x_rf.fx}")

    # Calculate the integral of the wheel_acc to calculate slip_x
    parameters.wheel_w_vel = parameters.wheel_w_vel + (parameters.x_rr.pho_r_2dot * parameters.time_step)  # rad/s

    return parameters, logger


def main():
    SIM_TIME = 22
    test_function = wheel_angular  
    function_name = test_function.__name__

    logger = LocalLogger(function_name).logger

    parameters = Initialization("../../Audi_r8.yaml", logger=logger)
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/chassis debug data/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []
    for i in range(len(sim_data)):

        parameters.x_rf.fx = np.array([sim_data[i].Vhcl_Wheel_FL_Fx,
                                       sim_data[i].Vhcl_Wheel_RL_Fx,
                                       sim_data[i].Vhcl_Wheel_FR_Fx,
                                       sim_data[i].Vhcl_Wheel_RR_Fx])

        parameters.powertrain_net_torque = np.array([sim_data[i].wheel_torque_FL, 
                                                     sim_data[i].wheel_torque_RL, 
                                                     sim_data[i].wheel_torque_FR, 
                                                     sim_data[i].wheel_torque_RR])

        data.append(test_function(parameters, logger)[0].get_data())  

    plt.figure()
    plt.title(function_name)
    plt.subplot(221)
    var_name = "Wheel_w_vel_FL"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label = var_name)
    plt.plot([i["wheel_w_vel"][0] for i in data], "--", label="wheel_w_vel 0 ")
    plt.legend()
    plt.subplot(222)
    var_name = "Wheel_w_vel_RL"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label = var_name)
    plt.plot([i["wheel_w_vel"][1] for i in data], "--", label="wheel_w_vel 1")
    plt.legend()
    plt.subplot(223)
    var_name = "Wheel_w_vel_FR"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label = var_name)
    plt.plot([i["wheel_w_vel"][2] for i in data], "--", label="wheel_w_vel 2")
    plt.legend()
    plt.subplot(224)
    var_name = "Wheel_w_vel_RR"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label = var_name)
    plt.plot([i["wheel_w_vel"][3] for i in data], "--", label="wheel_w_vel 3 ")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
