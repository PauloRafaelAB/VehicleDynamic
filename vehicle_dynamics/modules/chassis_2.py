# -*- coding: utf-8 -*-
"""
Created on Thu Jul 20 12:45:55 2023

@author: albertonbloemer
"""

from vehicle_dynamics.utils.ImportParam import ImportParam

from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.Displacement import Displacement

from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.SimulationData import SimulationData
from vehicle_dynamics.utils.Initialization import Initialization

import numpy as np
from tqdm import tqdm

import scipy.integrate as integrate
import matplotlib.pyplot as plt
import logging
import yaml
import pdb


def sub(a):
    return (a[0] - a[1])


def chassis_2(parameters: Initialization, logger: logging.Logger):
    """
    Chassis is a function that calculates the current status of the chassis

    Required Parameters from Param:
        1. m
        2. lv
        3. lh
        4. sl
        5. sr
        6. gravity
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
        9. logger

    Returns:
        1. x_a
            1. [x_a.x, x_a.y, x_a.z] 
            1. x_a.acc
            4. x_a.vx
            5. x_a.vy
            6. x_a.vz
            7. x_a.acc_angular_v
            8. x_a.wx
            9. x_a.wy
            10.x_a.wz
            11.x_a.roll
            12.x_a.pitch
            13.x_a.yaw
        2. displacement
            1. displacement.za

    """
    "Equations of motion Bardini, pag 272 ---- need initialize values"

    'sum of  wheel forces for calculating translation of the vehicle'

    sum_f_wheel = np.sum(
        parameters.x_rf.wheel_forces_transformed_force2vehicle_sys, axis=1)

    # logger.debug(f"FORCES IN THE VEHICLE  {np.shape(parameters.x_rf.wheel_forces_transformed_force2vehicle_sys)}")
    # logger.debug(f"Sum wheel {sum_f_wheel}") # Sum wheel (3,)
    # Equation 11-46 >> 11-12, Pag. 273
    # TODO: check gravity diretion

    drag_x = (-parameters.drag * (np.sqrt(parameters.x_a.vx **
              2 + parameters.x_a.vy**2) * parameters.x_a.vx))
    # + ((parameters.x_a.wz * parameters.x_a.vy) - (parameters.x_a.wy * parameters.x_a.vz))
    parameters.x_a.acc_x = (
        (sum_f_wheel[0] + drag_x) / parameters.car_parameters.m)
    # + ((parameters.x_a.wx * parameters.x_a.vz) - (parameters.x_a.wz * parameters.x_a.vx))
    parameters.x_a.acc_y = ((sum_f_wheel[1]) / parameters.car_parameters.m)
    # ((parameters.x_a.wy * parameters.x_a.vx) - (parameters.x_a.wx * parameters.x_a.vy))
    parameters.x_a.acc_z = (
        (sum_f_wheel[2] - parameters.car_parameters.m * parameters.gravity) / parameters.car_parameters.m)

    parameters.x_a.vx = parameters.x_a.vx + \
        (parameters.x_a.acc_x * parameters.time_step)
    parameters.x_a.vy = parameters.x_a.vy + \
        (parameters.x_a.acc_y * parameters.time_step)
    parameters.x_a.vz = parameters.x_a.vz + \
        (parameters.x_a.acc_z * parameters.time_step)

    parameters.x_a.x = parameters.x_a.x + \
        (parameters.x_a.vx * parameters.time_step)
    parameters.x_a.y = parameters.x_a.y + \
        (parameters.x_a.vy * parameters.time_step)
    parameters.x_a.z = parameters.x_a.z + \
        (parameters.x_a.vz * parameters.time_step)
    # logger.debug(f"shape position_chassi_force {np.shape(parameters.position_chassi_force)} {np.shape(parameters.strut2chassi_xyz)}")

    # parameters.strut2chassi_xyz = parameters.strut2chassi_xyz.T  # na duvida do que isso significa, conferir a matemagica

    # TODO Check eq 11 - 47
    crossproduct_r_f = np.zeros(((3, 4)))
    logger.debug(
        f" position_chassi_force {np.shape(parameters.position_chassi_force[0,:])} strut2chassi_xyz {parameters.strut2chassi_xyz[:,0]}")

# =============================================================================
# TODO: make the delta angle an input for chassis
# =============================================================================

    # Angular position, velocity, acceleration of the chassis
    delta = 0  # wheel angle
    Ix = parameters.car_parameters.i_x_s
    Iy = parameters.car_parameters.i_y_s
    Iz = parameters.car_parameters.i_z
    h = parameters.car_parameters.sz
    l = 0.6  # halftrack

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

    Mz = ((fy_fl + fy_fr) * parameters.car_parameters.lv * np.cos(delta) -
          (fy_rl + fy_rr) * parameters.car_parameters.lh +
          (fx_fl + fx_fr) * parameters.car_parameters.lv * np.sin(delta) +
          (fx_rr + fx_fr * np.cos(delta) + fy_fl * np.sin(delta) -
           fx_rl - fx_fl * np.cos(delta) - fy_fr * np.sin(delta))*l)

    a = Fy * np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.pitch) + \
        parameters.car_parameters.m * \
        parameters.gravity * np.sin(parameters.x_a.roll)
    b = parameters.car_parameters.c_roll * parameters.x_a.roll - \
        parameters.car_parameters.k_roll * \
        parameters.x_a.wx + parameters.x_a.wz * (Iy - Iz)
    c = parameters.x_a.wz * np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.pitch) + \
        parameters.x_a.wx * np.sin(parameters.x_a.pitch) * \
        np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.roll)
    d = parameters.x_a.wz * parameters.x_a.wx * \
        (Iy*np.cos(parameters.x_a.roll)**2 + Iz * np.sin(parameters.x_a.roll)**2)

    e = (Ix * np.cos(parameters.x_a.pitch)**2 + Iy * np.sin(parameters.x_a.pitch) ** 2 *
         np.sin(parameters.x_a.roll)**2 + Iz * np.sin(parameters.x_a.pitch)**2 * np.cos(parameters.x_a.roll)**2)

    wx_dot = (h * a - b * c + d)/e
    wx_dot = 0

    aa = h * (parameters.car_parameters.m*parameters.gravity*np.sin(parameters.x_a.pitch)*np.cos(parameters.x_a.roll) - Fx * np.cos(parameters.x_a.pitch)*np.cos(parameters.x_a.roll)) - parameters.car_parameters.c_pitch * parameters.x_a.pitch - parameters.car_parameters.k_pitch * parameters.x_a.wx 
    bb = parameters.x_a.wz * (parameters.x_a.wz * np.sin(parameters.x_a.pitch)*np.cos(parameters.x_a.pitch)*(Ix-Iy + np.cos(parameters.x_a.roll)**2 * (Iy-Iz))-parameters.x_a.wx*np.cos(parameters.x_a.pitch)**2*Ix + np.sin(parameters.x_a.roll)**2 * np.sin(parameters.x_a.pitch)**2 * Iy + np.sin(parameters.x_a.pitch)**2*np.cos(parameters.x_a.roll)**2*Iz)
    cc = parameters.x_a.wy * (np.sin(parameters.x_a.pitch) ** 2 * np.sin(parameters.x_a.roll)*np.cos(parameters.x_a.roll)*(Iy-Iz))
    dd = (Iy*np.cos(parameters.x_a.roll)**2+Iz*np.sin(parameters.x_a.roll)**2)
    
    wy_dot = (aa + bb - cc )/dd

    wz_dot = (Mz - h * (Fx * np.sin(parameters.x_a.pitch) + Fy * np.sin(parameters.x_a.pitch)*np.cos(parameters.x_a.roll)))/(
        Ix*np.sin(parameters.x_a.pitch)**2 + np.cos(parameters.x_a.pitch)**2 * (Iy * np.sin(parameters.x_a.roll)**2 + Iz * np.cos(parameters.x_a.roll)**2))
    wz_dot = 0
    parameters.x_a.wx = parameters.x_a.wx + wx_dot * parameters.time_step
    parameters.x_a.wy = parameters.x_a.wy + wy_dot * parameters.time_step
    parameters.x_a.wz = parameters.x_a.wz + wz_dot * parameters.time_step

    # Angular position
    parameters.x_a.roll = (parameters.x_a.wx * parameters.time_step) + parameters.x_a.roll
    parameters.x_a.pitch = (parameters.x_a.wy * parameters.time_step) + parameters.x_a.pitch
    parameters.x_a.yaw = (parameters.x_a.wz * parameters.time_step) + parameters.x_a.yaw

    # TODO: updated transformation to vehicle system

    vehicle_fixed2inertial_system = np.array([[np.cos(parameters.x_a.pitch) * np.cos(parameters.x_a.yaw), np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.cos(parameters.x_a.yaw) - np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.yaw), np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.cos(parameters.x_a.yaw) + np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.yaw)],
                                              [np.cos(parameters.x_a.pitch) * np.sin(parameters.x_a.yaw), np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.sin(parameters.x_a.yaw) + np.cos(parameters.x_a.roll) * np.cos(
                                                  parameters.x_a.yaw), np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.sin(parameters.x_a.yaw) - np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.yaw)],
                                              [-np.sin(parameters.x_a.pitch), np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.pitch), np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.pitch)]])

    # TODO check mat mul ordem

    displacement = Displacement()

    parameters.displacement.za[0] = (- parameters.car_parameters.lv * np.sin(
        parameters.x_a.pitch)) + (parameters.car_parameters.sl * np.sin(parameters.x_a.roll))
    parameters.displacement.za[1] = (+ parameters.car_parameters.lh * np.sin(
        parameters.x_a.pitch)) + (parameters.car_parameters.sl * np.sin(parameters.x_a.roll))
    parameters.displacement.za[2] = (- parameters.car_parameters.lv * np.sin(
        parameters.x_a.pitch)) - (parameters.car_parameters.sr * np.sin(parameters.x_a.roll))
    parameters.displacement.za[3] = (+ parameters.car_parameters.lh * np.sin(
        parameters.x_a.pitch)) - (parameters.car_parameters.sr * np.sin(parameters.x_a.roll))

    return parameters, logger, sum_f_wheel


def main():
    SIM_TIME = 22
    test_function = chassis_2
    function_name = test_function.__name__

    logger = LocalLogger(function_name).logger
    logger.setLevel('INFO')
    parameters = Initialization(
        "C:/Users/albertonbloemer/Documents/VehicleDynamic/Audi_r8.yaml", logger=logger)
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/acc_brake/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []

    parameters.x_a.vx = sim_data[0].Vhcl_PoI_Vel_x
    parameters.x_a.vy = sim_data[0].Vhcl_PoI_Vel_y
    parameters.x_a.vz = sim_data[0].Vhcl_PoI_Vel_z
    parameters.x_a.acc_x = sim_data[0].Vhcl_PoI_Acc_x
    parameters.x_a.acc_y = sim_data[0].Vhcl_PoI_Acc_y
    parameters.x_a.acc_z = sim_data[0].Vhcl_PoI_Acc_z
    parameters.x_a.roll = sim_data[0].Vhcl_Roll
    parameters.x_a.pitch = sim_data[0].Vhcl_Pitch
    parameters.x_a.yaw = sim_data[0].Vhcl_Yaw
    cm_z_force = parameters.f_za.f_za
    parameters.x_a.acc_angular_v = [
        sim_data[0].Vhcl_RollVel, sim_data[0].Vhcl_PitchVel, sim_data[0].Vhcl_YawVel]

    sum_wheels = []
    range_calc = range(1, 40000)
    for i in tqdm(range_calc):
        forces_in_x = [sim_data[i].wheel_load_x_FL, sim_data[i].wheel_load_x_RL,
                       sim_data[i].wheel_load_x_FR, sim_data[i].wheel_load_x_RR]
        forces_in_y = [sim_data[i].wheel_load_y_FL, sim_data[i].wheel_load_y_RL,
                       sim_data[i].wheel_load_y_FR, sim_data[i].wheel_load_y_RR]
        forces_in_z = [sim_data[i].wheel_load_z_FL, sim_data[i].wheel_load_z_RL,
                       sim_data[i].wheel_load_z_FR, sim_data[i].wheel_load_z_RR]
        parameters.x_rf.wheel_forces_transformed_force2vehicle_sys = np.array(
            [forces_in_x, forces_in_y, cm_z_force])
        parameters.strut2chassi_xyz = parameters.x_rf.wheel_forces_transformed_force2vehicle_sys
        parameters.angular_rates = np.array([parameters.x_a.wx,
                                             parameters.x_a.wy,
                                             parameters.x_a.wz])

        return_values = test_function(parameters, logger)

        sum_wheels.append(return_values[2].tolist())
        data.append(return_values[0].get_data())

    logger.info("calc end")

    plt.figure()
    plt.title(function_name)
    plt.plot(range_calc, [i["x_a.pitch"] for i in data], "--", label="pitch")
    plt.plot(range_calc, [i["x_a.yaw"] for i in data], "x", label="yaw")
    plt.plot(range_calc, [i["x_a.roll"] for i in data], "+", label="roll")
    var_name = "Vhcl_Pitch"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    var_name = "Vhcl_Yaw"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    var_name = "Vhcl_Roll"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)

    plt.legend()

    plt.figure()
    plt.title(function_name)
    plt.plot(range_calc, [i["x_a.wx"] for i in data], "--", label="wx - roll")
    plt.plot(range_calc, [i["x_a.wy"] for i in data], "x", label="wy - pitch")
    plt.plot(range_calc, [i["x_a.wz"] for i in data], "+", label="wz - yaw")
    var_name = "Vhcl_PitchVel"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    var_name = "Vhcl_YawVel"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
    var_name = "Vhcl_RollVel"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)

    plt.legend()

    if True:
        plt.figure()
        plt.title(function_name)
        plt.plot(range_calc, [i["x_a.acc_x"]
                 for i in data], "--", label="acc_x")
        plt.plot(range_calc, [i["x_a.acc_y"]
                 for i in data], "*", label="acc_y")
        # plt.plot(range_calc, [i["x_a.acc_z"] for i in data], "o", label="acc_z")
        var_name = "Vhcl_PoI_Acc_x"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
        var_name = "Vhcl_PoI_Acc_y"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)
        var_name = "Vhcl_PoI_Acc_z"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)

        plt.legend()

        plt.figure()
        plt.title(function_name)
        plt.plot([i["x_a.vx"] for i in data], "--", label="x_a.vx")
        var_name = "Vhcl_PoI_Vel_x"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)

        plt.plot([i["x_a.vy"] for i in data], "--", label="x_a.vy")
        var_name = "Vhcl_PoI_Vel_y"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)

        plt.plot([i["x_a.vz"] for i in data], "--", label="x_a.vz")
        var_name = "Vhcl_PoI_Vel_z"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label=var_name)

        plt.legend()

        plt.figure()

        plt.title("forces")
        sum_wheels = np.array(sum_wheels)
        plt.plot(sum_wheels[:, 0], label="x")
        plt.plot(sum_wheels[:, 1], label="y")
        plt.plot(sum_wheels[:, 2], label="z")

        plt.legend()

    plt.show()


if __name__ == '__main__':
    main()
