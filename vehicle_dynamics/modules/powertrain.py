from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.Initialization import Initialization
from vehicle_dynamics.utils.import_data_CM import import_data_CM

import logging
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
import yaml


def find_nearest(array, value):
    """find nearest from: https://stackoverflow.com/a/2566508 by Mateen Ulhaq"""
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]


def powertrain(parameters: Initialization, logger: logging.Logger, throttle: float, brake: float):
    """
    Powertrain is a function that calculates the current Torque delivered by the engine to the wheels

    Required Parameters from Param:
        1.rpm_table,
        2.torque_max_table,
        3.gear_ratio,
        4.diff,
        5.diff_ni,
        6.transmition_ni,
        7.gear_selection,
        8.engine_inertia,
        9.axel_inertia,
        10.gearbox_inertia,
        11.shaft_inertia,
        12.wheel_inertia,
        13.max_brake_torque,
        14.brake_bias,

    Required Arguments:
        1. throttle
        2. brake
        3. acc_x
        4. wheel_w_vel
        5. gear[t-1]
        6. vx

    Returns:
        1. Engine_rpm
        2. Gear
        3. Torque of the engine on the wheels

    """

    # Update Engine RPM

    parameters.rpm = parameters.car_parameters.gear_ratio[parameters.gear] * parameters.car_parameters.diff * np.mean(parameters.wheel_w_vel) * 30 / np.pi  # Wheel vx to engine RPM

    if parameters.rpm < parameters.car_parameters.rpm_table[0]:
        parameters.rpm = parameters.car_parameters.rpm_table[0] 
    elif parameters.rpm > parameters.car_parameters.rpm_table[-1]:
        parameters.rpm = parameters.car_parameters.rpm_table[-1]

    # Calculate torque provided by the engine based on the engine RPM
    # how much torque is available thoughout the rpm range
    torque_interpolation = interp1d(parameters.car_parameters.rpm_table, parameters.car_parameters.torque_max_table)
    # Max torque available at rpm
    torque_available = torque_interpolation(parameters.rpm)
    # find the torque delivered by te engine
    engine_torque = throttle * torque_available
    # Gearbox up or down shifting

    # print(f"{parameters.car_parameters.gear_selection[int(throttle * 10)][parameters.gear]} {parameters.x_a.vx} {parameters.gear} {int(throttle * 10)} ")

    if parameters.x_a.vx > parameters.car_parameters.gear_selection[int(throttle * 10)][parameters.gear]:
        parameters.gear = parameters.gear + 1
        if parameters.gear >= parameters.car_parameters.gear_ratio.size:
            parameters.gear = parameters.car_parameters.gear_ratio.size - 1
    elif parameters.x_a.vx <= 0.75 * parameters.car_parameters.gear_selection[int(throttle * 10)][parameters.gear - 1]:
        parameters.gear = parameters.gear - 1
        if parameters.gear < 1:
            parameters.gear = 1

    traction_torque = (engine_torque * parameters.car_parameters.gear_ratio[parameters.gear] * parameters.car_parameters.diff * parameters.car_parameters.diff_ni * parameters.car_parameters.transmition_ni) - ((
        ((parameters.car_parameters.engine_inertia + parameters.car_parameters.axel_inertia + parameters.car_parameters.gearbox_inertia) * (parameters.gear ** 2)) + (
            parameters.car_parameters.shaft_inertia * parameters.car_parameters.gear_ratio[parameters.gear] * (parameters.car_parameters.diff ** 2)) + parameters.car_parameters.wheel_inertia) * parameters.x_a.acc_x)

    # --------------------Break Torque -------------------------
    brake_torque = brake * parameters.car_parameters.max_brake_torque

    # -------------------- Total Torque -------------------------

    if np.mean((traction_torque - brake_torque)) <= 0 and parameters.x_a.vx <= 0:
        parameters.powertrain_net_torque = np.zeros(4)
    else:
        parameters.powertrain_net_torque = (traction_torque - brake_torque) * parameters.car_parameters.brake_bias

    return parameters, logger


"""
VhclCtrl Gas  # engine rpm
VhclCtrl Brake  # Brake pedal
Vhcl PoI_Acc_1 x  # x acceleration
PT IF W < pos > rotv  # wheel angular velocity
Vehicle.GearNo  # gear
# Vehicle.Engine_rotv # eng rpm >>engine_rotv 
"""


def main():
    SIM_TIME = 22
    test_function = powertrain
    function_name = test_function.__name__

    logger = LocalLogger(function_name).logger

    parameters = Initialization("../../Audi_r8.yaml", logger=logger)
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/Powertrain testing/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []
    for i in range(22001):
        parameters.x_a.vx = sim_data[i].Vhcl_PoI_Vel_1_x
        parameters.wheel_w_vel = np.array([sim_data[i].Wheel_w_vel_FL,
                                           sim_data[i].Wheel_w_vel_RL,
                                           sim_data[i].Wheel_w_vel_FR,
                                           sim_data[i].Wheel_w_vel_RR])
        data.append(test_function(parameters, logger, throttle = sim_data[i].gas_pedal, brake = sim_data[i].brake_pedal)[0].get_data())  

    plt.figure()
    plt.step(range(22001), [i["gear"] for i in data], "g", label="gear_no_calcu")
    var_name = "gear_no"
    plt.step([i for j, i in enumerate(sim_data.keys()) if j % 100 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 100 == 0], label = var_name)
    plt.legend()
    plt.figure()
    plt.plot([i["x_a.vx"] for i in data])

    plt.show()


if __name__ == '__main__':
    main()
