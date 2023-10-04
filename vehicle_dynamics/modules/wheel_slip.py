from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.utils.Initialization import Initialization
from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger

import matplotlib.pyplot as plt
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

    # x-position from Vehicle CoG to the front axle [m]
    long_f = parameters.car_parameters.lv
    # x-position from Vehicle Cog to the rear axle [m]
    long_r = - parameters.car_parameters.lh
    # y-position from Vehicle CoG to the left chassis point [m]
    lat_l = - parameters.car_parameters.sl
    # y-position from Vehicle CoG to the right chassis point  [m]
    lat_r = parameters.car_parameters.sr

    if (abs(parameters.car_parameters.r_dyn * parameters.wheel_w_vel).all() == 0) and (abs(parameters.x_a.vx) == 0):
        parameters.slip_x = np.zeros(4)
    else:
        # equation 11.30 Bardini
        parameters.slip_x = ((((parameters.car_parameters.r_dyn * parameters.wheel_w_vel) - parameters.x_a.vx) / np.maximum(
            np.absolute(parameters.car_parameters.r_dyn * parameters.wheel_w_vel), np.absolute(parameters.x_a.vx))))
        # equation 11.31 Bardini
        #parameters.slip_y = - np.arctan(parameters.x_a.vy / np.maximum(abs(parameters.car_parameters.r_dyn * parameters.wheel_w_vel), abs(parameters.x_a.vx)))

    # Refenrence from VTI Y Slip angle
    if (parameters.x_a.vx == 0 and parameters.x_a.wz == 0):
        parameters.slip_y = np.zeros(4)
    else:
        parameters.slip_y[0] = parameters.delta - np.arctan(
            (parameters.x_a.vy + long_f * parameters.x_a.wz) /
            (parameters.x_a.vx - lat_l * parameters.x_a.wz))  # Front Left
        
        parameters.slip_y[1] = - np.arctan((parameters.x_a.vy + long_r * parameters.x_a.wz) / (
            parameters.x_a.vx - lat_l * parameters.x_a.wz))  # Rear Left
        
        parameters.slip_y[2] = parameters.delta - np.arctan((parameters.x_a.vy + long_f*parameters.x_a.wz)/(
            parameters.x_a.vx - lat_r*parameters.x_a.wz))  # Front Right
        
        parameters.slip_y[3] = - np.arctan((parameters.x_a.vy + long_r*parameters.x_a.wz)/(
            parameters.x_a.vx - lat_r*parameters.x_a.wz))  # Rear Right

        #parameters.slip_y = 0
    logger.debug(f"SLIP X  {parameters.slip_x}")
    return parameters, logger


def main():

    test_function = wheel_slip
    function_name = test_function.__name__

    logger = LocalLogger(function_name).logger

    parameters = Initialization("../../Audi_r8.yaml", logger=logger)
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/Lanechange_new/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []

    for i in range(len(sim_data)):
        parameters.wheel_w_vel = np.array(
            [sim_data[i].Wheel_w_vel_FL, sim_data[i].Wheel_w_vel_RL, sim_data[i].Wheel_w_vel_FR, sim_data[i].Wheel_w_vel_RR])
        parameters.x_a.vx = sim_data[i].Vhcl_PoI_Vel_1_x
        parameters.x_a.vy = sim_data[i].Vhcl_PoI_Vel_1_y
        parameters.x_a.wz = sim_data[i].Vhcl_YawVel
        parameters.delta = sim_data[i].Steering_angle / \
            parameters.car_parameters.steering_ratio

        data.append(test_function(parameters, logger)[0].get_data())

    plt.figure()
    plt.title(function_name)
    #plt.plot([i["delta"] for i in data], label="delta")
    plt.plot([i["x_a.vx"] for i in data], "-", label="Vx")
    plt.plot([i["wheel_w_vel0"] * parameters.car_parameters.r_dyn[0] for i in data],
             "-", label="wheel_w_vel0")
    plt.plot([i["wheel_w_vel1"] * parameters.car_parameters.r_dyn[1] for i in data],
             "-", label="wheel_w_vel1")
    plt.legend(loc=0)
    plt.twinx()

    plt.plot([i["slip_x0"] for i in data], "--", label="slip x FL")
    plt.plot([i["slip_x1"] for i in data], "--", label="slip x RL")
    plt.plot([i["slip_x2"] for i in data], "--", label="slip x FR")
    plt.plot([i["slip_x3"] for i in data], "--", label="slip x RR")
    #plt.plot([i["x_a.vy"] for i in data], "--", label="Vy")
    #plt.plot([i["wheel_w_vel0"] for i in data], "--", label="slip x RR")
    plt.legend()

    #plt.plot(steering_angle_v, label ="steering_angle_calc")

    plt.figure()
    plt.title(function_name)
    #plt.plot([i["slip_x"] for i in data], "--", label="slip x")
    var_name = "Vhcl_Wheel_FL_LongSlip"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    var_name = "Vhcl_Wheel_RL_LongSlip"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    var_name = "Vhcl_Wheel_FR_LongSlip"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    var_name = "Vhcl_Wheel_RR_LongSlip"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)

    plt.step([i["slip_x0"] for i in data], "--", label="slip x FL")
    plt.step([i["slip_x1"] for i in data], "--", label="slip x RL")
    plt.step([i["slip_x2"] for i in data], "--", label="slip x FR")
    plt.step([i["slip_x3"] for i in data], "--", label="slip x RR")
    plt.legend()

    plt.figure()
    plt.title("Lateral slip")
    #plt.plot([i["slip_y"] for i in data], "--", label="slip y")
    var_name = "Vhcl_Wheel_FL_Sidelip"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    var_name = "Vhcl_Wheel_RL_Sidelip"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    var_name = "Vhcl_Wheel_FR_Sidelip"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    var_name = "Vhcl_Wheel_RR_Sidelip"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)

    plt.step([i["slip_y0"] for i in data], "--", label="slip y FL")
    plt.step([i["slip_y1"] for i in data], "--", label="slip y RL")
    plt.step([i["slip_y2"] for i in data], "--", label="slip y FR")
    plt.step([i["slip_y3"] for i in data], "--", label="slip y RR")

    plt.plot([i["x_a.vy"] for i in data], "-", label="Vy")

    plt.legend()

    plt.show()


if __name__ == '__main__':
    main()
