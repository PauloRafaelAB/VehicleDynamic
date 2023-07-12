from vehicle_dynamics.utils.ImportParam import ImportParam

from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.Displacement import Displacement

from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.SimulationData import SimulationData
from vehicle_dynamics.utils.Initialization import Initialization

import numpy as np
import matplotlib.pyplot as plt
import logging
import yaml


def chassis(parameters: Initialization, logger: logging.Logger):
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

    sum_f_wheel = np.sum(parameters.x_rf.wheel_forces_transformed_force2vehicle_sys, axis = 1)

    logger.debug(f"FORCES IN THE VEHICLE  {np.shape(parameters.x_rf.wheel_forces_transformed_force2vehicle_sys)}")
    logger.debug(f"Sum wheel {np.shape(sum_f_wheel)}")
    # Equation 11-46 >> 11-12, Pag. 273
    # TODO: check gravity diretion

# =============================================================================
#     # O que faz esses valores atras da acceleracao?????????????? 11.12-13-14
#       vx must be the vehicle speed one dimenention only
# =============================================================================
    parameters.x_a.acc_x = ((sum_f_wheel[0] + (parameters.drag * (parameters.x_a.vx ** 2))) / parameters.car_parameters.m) + ((parameters.x_a.wz * parameters.x_a.vy) - (parameters.x_a.wy * parameters.x_a.vz))
    parameters.x_a.acc_y = (sum_f_wheel[1] / parameters.car_parameters.m) + ((parameters.x_a.wx * parameters.x_a.vz) - (parameters.x_a.wz * parameters.x_a.vx))
    parameters.x_a.acc_z = ((sum_f_wheel[2] - (parameters.car_parameters.m * parameters.gravity)) / parameters.car_parameters.m) + ((parameters.x_a.wy * parameters.x_a.vx) - (parameters.x_a.wx * parameters.x_a.vy))

    # vehicle velocity calculation
    parameters.x_a.vx = parameters.x_a.vx + parameters.x_a.acc_x * parameters.time_step
    parameters.x_a.vy = parameters.x_a.vy + parameters.x_a.acc_y * parameters.time_step
    parameters.x_a.vz = parameters.x_a.vz + parameters.x_a.acc_z * parameters.time_step

    # TODO:Check rolling resistance            
    # rolling_resist = (parameters.car_parameters.fr * parameters.car_parameters.m * parameters.car_parameters.gravity * np.cos(0.) - 0.5 * parameters.car_parameters.row * parameters.car_parameters.Cl * parameters.car_parameters.area * speed ** 2)                              # Rolling Resistance with air lift
    crossproduct_r_f = np.zeros((4, 3))
    logger.debug(f"shape position_chassi_force {np.shape(parameters.position_chassi_force)} {np.shape(parameters.strut2chassi_xyz)}")

    parameters.strut2chassi_xyz = parameters.strut2chassi_xyz.T  # na duvida do que isso significa, conferir a matemagica

    for i in range(4):
        crossproduct_r_f[i][0] = parameters.position_chassi_force[i][1] * parameters.strut2chassi_xyz[i][2] - parameters.position_chassi_force[i][2] * parameters.strut2chassi_xyz[i][1] 
        crossproduct_r_f[i][1] = parameters.position_chassi_force[i][2] * parameters.strut2chassi_xyz[i][0] - parameters.position_chassi_force[i][0] * parameters.strut2chassi_xyz[i][2]
        crossproduct_r_f[i][2] = parameters.position_chassi_force[i][0] * parameters.strut2chassi_xyz[i][1] - parameters.position_chassi_force[i][1] * parameters.strut2chassi_xyz[i][0]

    # TODO Check eq 11 - 47
    sum_crossproduct_r_f = np.sum(crossproduct_r_f, axis = 0)
    # TODO make the return of acc angular be type (3,0)

    parameters.x_a.acc_angular_v = (sum_crossproduct_r_f - np.cross(parameters.angular_rates, (parameters.polar_inertia_v @ parameters.angular_rates))) @ np.linalg.inv(parameters.polar_inertia_v)

    logger.debug(f"sum_crossproduct {sum_crossproduct_r_f}")
    # logger.debug('parameters.angular_rates', parameters.angular_rates)
    # logger.debug('polar inertia',parameters.polar_inertia_v)

    # logger.debug('parameters.x_a.acc_angular_v',parameters.x_a.acc_angular_v,type(parameters.x_a.acc_angular_v))
    # TODO: add multiplication of tranpose polar inertia to eq above>>   * parameters.polar_inertia_v.T)

    # Angular velocity of the chassis

    angular_velocity = parameters.x_a.acc_angular_v * parameters.time_step
    parameters.x_a.wx = parameters.x_a.wx + angular_velocity[0] 
    parameters.x_a.wy = parameters.x_a.wy + angular_velocity[1]
    parameters.x_a.wz = parameters.x_a.wz + angular_velocity[2]

    # Angular position   
    parameters.x_a.roll = (parameters.x_a.wx * parameters.time_step) + parameters.x_a.roll
    parameters.x_a.pitch = (parameters.x_a.wy * parameters.time_step) + parameters.x_a.pitch
    parameters.x_a.yaw = (parameters.x_a.wz * parameters.time_step) + parameters.x_a.yaw

    # TODO: updated transformation to vehicle system

    vehicle_fixed2inertial_system = np.array([[np.cos(parameters.x_a.pitch) * np.cos(parameters.x_a.yaw), np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.cos(parameters.x_a.yaw) - np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.yaw), np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.cos(parameters.x_a.yaw) + np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.yaw)],
                                              [np.cos(parameters.x_a.pitch) * np.sin(parameters.x_a.yaw), np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.sin(parameters.x_a.yaw) + np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.yaw), np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.sin(parameters.x_a.yaw) - np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.yaw)],
                                              [-np.sin(parameters.x_a.pitch), np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.pitch), np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.pitch)]])

    # vehicle position calculation
    movement_vehicle = (parameters.x_a.vx * parameters.time_step, parameters.x_a.vy * parameters.time_step, parameters.x_a.vz * parameters.time_step) 

    # TODO check mat mul ordem
    [parameters.x_a.x, parameters.x_a.y, parameters.x_a.z] = [parameters.x_a.x, parameters.x_a.y, parameters.x_a.z] + (movement_vehicle @ vehicle_fixed2inertial_system)

    displacement = Displacement()

    parameters.displacement.za[0] = (- parameters.car_parameters.lv * np.sin(parameters.x_a.pitch)) + (parameters.car_parameters.sl * np.sin(parameters.x_a.roll))
    parameters.displacement.za[1] = (+ parameters.car_parameters.lh * np.sin(parameters.x_a.pitch)) + (parameters.car_parameters.sl * np.sin(parameters.x_a.roll))
    parameters.displacement.za[2] = (- parameters.car_parameters.lv * np.sin(parameters.x_a.pitch)) - (parameters.car_parameters.sr * np.sin(parameters.x_a.roll))
    parameters.displacement.za[3] = (+ parameters.car_parameters.lh * np.sin(parameters.x_a.pitch)) - (parameters.car_parameters.sr * np.sin(parameters.x_a.roll))

    return parameters, logger 


def main():
    SIM_ITER = 1000
    test_function = chassis
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
