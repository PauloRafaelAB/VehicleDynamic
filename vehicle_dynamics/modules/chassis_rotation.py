# -*- coding: utf-8 -*-

from vehicle_dynamics.structures.Displacement import Displacement
from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.Initialization import Initialization
import numpy as np
import logging


def chassis_rotation(parameters: Initialization, logger: logging.Logger):
    """
    Chassis_rotation is a function that calculates the current status of the chassis Euler angles 

    Required Parameters from car_parameters:
        1. m
        2. lv
        3. lh
        4. sl
        5. sr
        6. i_x_s
        7. i_y_s
        8. i_z
        9. c_roll
        10. k_roll
        11. c_pitch
        12. k_pitch
    Required Arguments:
        1. x_a
            1.01 roll
            1.02 pitch
            1.03 yaw
            1.04 wz
            1.05 wy
            1.06 wx
            1.07 vx
            1.08 vy
            1.09 vz
            1.10 acc
            1.11 acc_angular_v
        2. time_step
        3. x_rf.wheel_forces_transformed_force2vehicle_sys
        4. drag
        5. position_chassi_force
        6. strut2chassi_xyz
        7. angular_rates
        8. polar_inertia_v
        9. gravity
        10. logger

    Returns: (Parameters)?????
        1. x_a
            1. x_a.acc_angular_v
            2. x_a.wx
            3. x_a.wy
            4. x_a.wz
            5. x_a.roll
            6. x_a.pitch
            7. x_a.yaw
        2. displacement
            1. displacement.za

    """

    sum_f_wheel = np.sum(
        parameters.x_rf.wheel_forces_transformed_force2vehicle_sys, axis=1)

# =============================================================================
# TODO: make the delta angle an input for chassis
# =============================================================================

    # Angular position, velocity, acceleration of the chassis
    delta = parameters.last_delta       # wheel angle
    Ix = parameters.car_parameters.i_x_s
    Iy = parameters.car_parameters.i_y_s
    Iz = parameters.car_parameters.i_z
    h = parameters.car_parameters.sz
    l = parameters.car_parameters.sr    # halftrack
    
    # Abel Castro, pag. 6 eq-17 
    Fx = sum_f_wheel[0]
    Fy = sum_f_wheel[1]
    fy_fl = parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[1, 0]
    fy_rl = parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[1, 1]
    fy_fr = parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[1, 2]
    fy_rr = parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[1, 3]
    fx_fl = parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 0]
    fx_rl = parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 1]
    fx_fr = parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 2]
    fx_rr = parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 3]

    
    Mz = ((fy_fl + fy_fr) * parameters.car_parameters.lv * np.cos(delta) - (fy_rl + fy_rr) * parameters.car_parameters.lh + (fx_fl + fx_fr) * parameters.car_parameters.lv * np.sin(delta) + (fx_rr + fx_fr * np.cos(delta) + fy_fl * np.sin(delta) - fx_rl - fx_fl * np.cos(delta) - fy_fr * np.sin(delta)) * l)

    a = Fy * np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.pitch) + parameters.car_parameters.m * parameters.gravity * np.sin(parameters.x_a.roll)
    b = -parameters.car_parameters.c_roll * parameters.x_a.roll - parameters.car_parameters.k_roll * parameters.x_a.wx 

    c = parameters.x_a.wz * (Iy - Iz) * (parameters.x_a.wz * np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.pitch) + parameters.x_a.wx * np.sin(parameters.x_a.pitch) * np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.roll))

    d = parameters.x_a.wz * parameters.x_a.wx * (Iy * (np.cos(parameters.x_a.roll)**2) + Iz * (np.sin(parameters.x_a.roll)**2))

    e = (Ix * (np.cos(parameters.x_a.pitch)**2) + Iy * (np.sin(parameters.x_a.pitch)**2) * (np.sin(parameters.x_a.roll)**2) + Iz * (np.sin(parameters.x_a.pitch)**2) * (np.cos(parameters.x_a.roll)**2))

    wx_dot = (h * a + b + c + d) / e

    aa = h * (parameters.car_parameters.m * parameters.gravity * np.sin(parameters.x_a.pitch) * np.cos(parameters.x_a.roll) - Fx * np.cos(parameters.x_a.pitch) * np.cos(parameters.x_a.roll)) - parameters.car_parameters.c_pitch * parameters.x_a.pitch - parameters.car_parameters.k_pitch * parameters.x_a.wy
    bb = parameters.x_a.wz * (parameters.x_a.wz * np.sin(parameters.x_a.pitch) * np.cos(parameters.x_a.pitch) * (Ix - Iy + np.cos(parameters.x_a.roll)**2 * (Iy - Iz)) - parameters.x_a.wx * np.cos(parameters.x_a.pitch)**2 * Ix + np.sin(parameters.x_a.roll)**2 * np.sin(parameters.x_a.pitch)**2 * Iy + np.sin(parameters.x_a.pitch)**2 * np.cos(parameters.x_a.roll)**2 * Iz)
    cc = parameters.x_a.wy * (np.sin(parameters.x_a.pitch) * np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.roll) * (Iy - Iz))
    dd = (Iy * np.cos(parameters.x_a.roll)**2 + Iz * np.sin(parameters.x_a.roll)**2)

    wy_dot = (aa + bb - cc) / dd
    wz_dot = (Mz - h * (Fx * np.sin(parameters.x_a.roll) + Fy * np.sin(parameters.x_a.pitch) * np.cos(parameters.x_a.roll))) / (Ix * (np.sin(parameters.x_a.pitch)**2) + (np.cos(parameters.x_a.pitch)**2) * (Iy * (np.sin(parameters.x_a.roll)**2) + Iz * (np.cos(parameters.x_a.roll)**2)))

    parameters.x_a.wx = parameters.x_a.wx + wx_dot * parameters.time_step
    parameters.x_a.wy = parameters.x_a.wy + wy_dot * parameters.time_step
    parameters.x_a.wz = parameters.x_a.wz + wz_dot * parameters.time_step

    # Angular position
    parameters.x_a.roll = (parameters.x_a.wx * parameters.time_step) + parameters.x_a.roll
    parameters.x_a.pitch = (parameters.x_a.wy * parameters.time_step) + parameters.x_a.pitch
    parameters.x_a.yaw = (parameters.x_a.wz * parameters.time_step) + parameters.x_a.yaw
    parameters.x_a.yaw = ((np.pi+parameters.x_a.yaw)%(2*np.pi))-np.pi
    # TODO check mat mul ordem
    parameters.displacement.za[0] = (- parameters.car_parameters.lv * np.sin(parameters.x_a.pitch)) + (parameters.car_parameters.sl * np.sin(parameters.x_a.roll))
    parameters.displacement.za[1] = (+ parameters.car_parameters.lh * np.sin(parameters.x_a.pitch)) + (parameters.car_parameters.sl * np.sin(parameters.x_a.roll))
    parameters.displacement.za[2] = (- parameters.car_parameters.lv * np.sin(parameters.x_a.pitch)) - (parameters.car_parameters.sr * np.sin(parameters.x_a.roll))
    parameters.displacement.za[3] = (+ parameters.car_parameters.lh * np.sin(parameters.x_a.pitch)) - (parameters.car_parameters.sr * np.sin(parameters.x_a.roll))

    return parameters, logger


def main():
    from tqdm import tqdm
    import matplotlib.pyplot as plt
    import pdb
    test_function = chassis_rotation
    function_name = test_function.__name__

    logger = LocalLogger(function_name).logger
    logger.setLevel('INFO')
    parameters = Initialization("../../Audi_r8.yaml", logger=logger)
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/Lane_change_continue/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []

    parameters.x_a.roll = sim_data[0].Vhcl_Roll
    parameters.x_a.pitch = 0
    pitch_trans = [sim_data[i].Vhcl_Pitch - sim_data[0].Vhcl_Pitch for i in range(len(sim_data))]
    parameters.x_a.yaw = 0
    yaw_trans = [sim_data[i].Vhcl_Yaw - sim_data[0].Vhcl_Yaw for i in range(len(sim_data))]
    cm_z_force = parameters.f_za.f_za
    parameters.x_a.acc_angular_v = [
        sim_data[0].Vhcl_RollVel, sim_data[0].Vhcl_PitchVel, sim_data[0].Vhcl_YawVel]

    sum_wheels = []
    range_calc = range(1, 18000)
    for i in tqdm(range_calc):
        forces_in_x = [sim_data[i].wheel_load_x_FL, sim_data[i].wheel_load_x_RL,
                       sim_data[i].wheel_load_x_FR, sim_data[i].wheel_load_x_RR]
        forces_in_y = [sim_data[i].wheel_load_y_FL, sim_data[i].wheel_load_y_RL,
                       sim_data[i].wheel_load_y_FR, sim_data[i].wheel_load_y_RR]
        forces_in_z = [sim_data[i].wheel_load_z_FL, sim_data[i].wheel_load_z_RL,
                       sim_data[i].wheel_load_z_FR, sim_data[i].wheel_load_z_RR]
        parameters.x_rf.wheel_forces_transformed_force2vehicle_sys = np.array(
            [forces_in_x, forces_in_y, forces_in_z])

        parameters.last_delta = sim_data[i].Steering_angle

        return_values = test_function(parameters, logger)

        data.append(return_values[0].get_data())

    logger.info("calc end")

    plt.figure()
    plt.title(function_name)

    plt.plot(range_calc, [i["x_a.roll"] for i in data], "--c", label="roll")
    plt.plot(range_calc, [i["x_a.pitch"] for i in data], "--k", label="pitch")
    plt.plot(range_calc, [i["x_a.yaw"] for i in data], "--g", label="yaw")

    var_name = "Vhcl_Roll"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], "c", label=var_name)
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [i for j, i in enumerate(pitch_trans) if j % 10 == 0], "k", label="pitch_trans")
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [i for j, i in enumerate(yaw_trans) if j % 10 == 0], "g", label="yaw_trans")

    plt.legend()

    plt.figure()
    plt.title(function_name)
    plt.plot(range_calc, [i["x_a.wx"] for i in data], "--c", label="wx - roll")
    plt.plot(range_calc, [i["x_a.wy"] for i in data], "--k", label="wy - pitch")
    plt.plot(range_calc, [i["x_a.wz"] for i in data], "--g", label="wz - yaw")

    var_name = "Vhcl_RollVel"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], "c", label=var_name)
    var_name = "Vhcl_PitchVel"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], "k", label=var_name)
    var_name = "Vhcl_YawVel"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], "g", label=var_name)

    plt.legend()

    plt.show()


if __name__ == '__main__':
    main()
