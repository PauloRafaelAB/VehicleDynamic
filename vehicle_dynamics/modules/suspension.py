from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.Displacement import Displacement
from vehicle_dynamics.utils.Initialization import Initialization
from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger

import matplotlib.pyplot as plt
import numpy as np
import logging
import yaml


def suspension(parameters: Initialization, logger: logging.Logger):
    """
     suspension is a function that calculates the current wheel loads (z)

     Required Parameters from car_parameters:
         1. eq_stiff
         2. dumper

     Required Arguments:
         1. f_zr
             1.01 wheel_load_z
        2.displacement
            2.01 za    :  position of the connection point of the suspension with the dampner spring system
            2.02 zs    :  Z velocity of the wheel
            2.03 l_stat:  constant
            2.04 za_dot:  velocity of the chassis point
        3.vehicle_fixed2inertial_system
     Returns:
         1. f_zr.wheel_load_z

     """
    #parameters.x_a.yaw = ((np.pi+parameters.x_a.yaw)%(2*np.pi))-np.pi
    # TODO check mat mul ordem
    parameters.displacement.za[0] = (- parameters.car_parameters.lv * np.sin(parameters.x_a.pitch)) + (parameters.car_parameters.sl * np.sin(parameters.x_a.roll))
    parameters.displacement.za[1] = (+ parameters.car_parameters.lh * np.sin(parameters.x_a.pitch)) + (parameters.car_parameters.sl * np.sin(parameters.x_a.roll))
    parameters.displacement.za[2] = (- parameters.car_parameters.lv * np.sin(parameters.x_a.pitch)) - (parameters.car_parameters.sr * np.sin(parameters.x_a.roll))
    parameters.displacement.za[3] = (+ parameters.car_parameters.lh * np.sin(parameters.x_a.pitch)) - (parameters.car_parameters.sr * np.sin(parameters.x_a.roll))
    

    # Forces on the vehicle chassis at the pivot points Ai
    # Bardini pag. 265 eq. 11-21


    A = (parameters.car_parameters.eq_stiff * (-parameters.displacement.za + parameters.displacement.zs +
         parameters.displacement.l_stat)) + (parameters.car_parameters.dumper * parameters.displacement.za_dot)
    B = parameters.vehicle_fixed2inertial_system @ np.array([[0], [0], [1]])
    parameters.f_zr.wheel_load_z = (A * B)[2]
    parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[2, :] = parameters.f_zr.wheel_load_z

    logger.debug(f"wheel load z {parameters.f_zr.wheel_load_z}")

    return parameters, logger



def main():
    SIM_TIME = 22
    test_function = suspension
    function_name = test_function.__name__
    logger = LocalLogger(function_name).logger
    logger.setLevel('INFO')

    parameters = Initialization("../../Audi_r8.yaml", logger=logger)
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/chassis debug data/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []
    for i in range(len(sim_data)):
        parameters.x_a.roll = sim_data[i].Vhcl_Roll
        parameters.x_a.pitch = sim_data[i].Vhcl_Pitch
        parameters.x_a.yaw = sim_data[i].Vhcl_Yaw

        wheel_load_z = [sim_data[i].wheel_load_z_FL, sim_data[i].wheel_load_z_RL,
                        sim_data[i].wheel_load_z_FR, sim_data[i].wheel_load_z_RR]

        parameters.displacement = Displacement(l_stat=(parameters.car_parameters.m * parameters.car_parameters.wd * parameters.gravity) / parameters.car_parameters.eq_stiff,
                                               za=wheel_load_z / parameters.car_parameters.eq_stiff,
                                               za_dot=np.zeros(4),
                                               zr_dot=np.zeros(4),
                                               zr_2dot=np.zeros(4))
        parameters.vehicle_fixed2inertial_system = np.array([[np.cos(parameters.x_a.pitch) * np.cos(parameters.x_a.yaw), np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.cos(parameters.x_a.yaw) - np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.yaw), np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.cos(parameters.x_a.yaw) + np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.yaw)],
                                                             [np.cos(parameters.x_a.pitch) * np.sin(parameters.x_a.yaw), np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.sin(parameters.x_a.yaw) + np.cos(parameters.x_a.roll) * np.cos(
                                                                 parameters.x_a.yaw), np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.sin(parameters.x_a.yaw) - np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.yaw)],
                                                             [-np.sin(parameters.x_a.pitch), np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.pitch), np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.pitch)]])  # Bardini pag 260

        data.append(test_function(parameters, logger)[0].get_data())

    plt.figure()
    plt.title(function_name)
    plt.subplot(221)
    var_name = "wheel_load_z_FL"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    plt.plot([i["wheel_load_z"][0]
             for i in data], "--", label="wheel_load_z 0 ")
    plt.legend()
    plt.subplot(222)
    var_name = "wheel_load_z_RL"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    plt.plot([i["wheel_load_z"][1]
             for i in data], "--", label="wheel_load_z 1")
    plt.legend()
    plt.subplot(223)
    var_name = "wheel_load_z_FR"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    plt.plot([i["wheel_load_z"][2]
             for i in data], "--", label="wheel_load_z 2")
    plt.legend()
    plt.subplot(224)
    var_name = "wheel_load_z_RR"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    plt.plot([i["wheel_load_z"][3]
             for i in data], "--", label="wheel_load_z 3 ")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
