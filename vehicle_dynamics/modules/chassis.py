from vehicle_dynamics.utils.ImportParam import ImportParam

from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.Displacement import Displacement

from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.SimulationData import SimulationData

import numpy as np
import matplotlib.pyplot as plt
import logging
import yaml


def chassis(param: ImportParam,
            x_a: StateVector,
            time_step: float,
            x_rf: TireForces,
            drag: float,
            position_chassi_force: np.ndarray,
            strut2chassi_xyz: np.ndarray,
            angular_rates: np.ndarray,
            polar_inertia_v: np.ndarray,
            logger: logging.Logger,
            *xargs, **kwargs) -> np.ndarray: 
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
        2. displacement

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
        14. displacement.za

    """
    "Equations of motion Bardini, pag 272 ---- need initialize values"

    'sum of  wheel forces for calculating translation of the vehicle'

    sum_f_wheel = np.sum(x_rf.wheel_forces_transformed_force2vehicle_sys, axis=1)

    logger.debug("FORCES IN THE VEHICLE ", x_rf.wheel_forces_transformed_force2vehicle_sys)
    logger.debug("Sum wheel ", sum_f_wheel)
    # Equation 11-46 >> 11-12, Pag. 273
    # TODO: check gavity diretion

    # O que faz esses valores atras da acceleracao?????????????? 11.12-13-14
    x_a.acc[0] = ((sum_f_wheel[0] + (drag * (x_a.vx ** 2))) / param.m) + ((x_a.wz * x_a.vy) - (x_a.wy * x_a.vz))
    x_a.acc[1] = (sum_f_wheel[1] / param.m) + ((x_a.wx * x_a.vz) - (x_a.wz * x_a.vx))
    x_a.acc[2] = ((sum_f_wheel[2] - (param.m * param.gravity)) / param.m) + ((x_a.wy * x_a.vx) - (x_a.wx * x_a.vy))

    # vehicle velocity calculation
    velocity = x_a.acc * time_step
    x_a.vx = x_a.vx + velocity[0] 
    x_a.vy = x_a.vy + velocity[1] 
    x_a.vz = x_a.vz + velocity[2] 

    # TODO:Check rolling resistance            
    # rolling_resist = (param.fr * param.m * param.gravity * np.cos(0.) - 0.5 * param.row * param.Cl * param.area * speed ** 2)                              # Rolling Resistance with air lift

    # 11-47

    for i in range(4):
        crossproduct_r_f[i][0] = position_chassi_force[i][1] * strut2chassi_xyz[i][2] - position_chassi_force[i][2] * strut2chassi_xyz[i][1] 
        crossproduct_r_f[i][1] = position_chassi_force[i][2] * strut2chassi_xyz[i][0] - position_chassi_force[i][0] * strut2chassi_xyz[i][2]
        crossproduct_r_f[i][2] = position_chassi_force[i][0] * strut2chassi_xyz[i][1] - position_chassi_force[i][1] * strut2chassi_xyz[i][0]
    # TODO Check eq 11 - 47
    sum_crossproduct_r_f = np.sum(crossproduct_r_f, axis=0)
    # logger.debug('sum_crossproduct',sum_crossproduct_r_f)
    # TODO make the return of acc angular be type (3,0)

    x_a.acc_angular_v = (sum_crossproduct_r_f - np.cross(angular_rates, (polar_inertia_v @ angular_rates))) @ np.linalg.inv(polar_inertia_v)

    logger.debug('sum_crossproduct', sum_crossproduct_r_f)
    # logger.debug('angular_rates', angular_rates)
    # logger.debug('polar inertia',polar_inertia_v)

    # logger.debug('x_a.acc_angular_v',x_a.acc_angular_v,type(x_a.acc_angular_v))
    # TODO: add multiplication of tranpose polar inertia to eq above>>   * polar_inertia_v.T)

    # Angular velocity of the chassis

    angular_velocity = x_a.acc_angular_v * time_step
    x_a.wx = x_a.wx + angular_velocity[0] 
    x_a.wy = x_a.wy + angular_velocity[1]
    x_a.wz = x_a.wz + angular_velocity[2]

    # Angular position   
    x_a.roll = (x_a.wx * time_step) + x_a.roll
    x_a.pitch = (x_a.wy * time_step) + x_a.pitch
    x_a.yaw = (x_a.wz * time_step) + x_a.yaw

    # TODO: updated transformation to vehicle system

    vehicle_fixed2inertial_system = np.array([[np.cos(x_a.pitch) * np.cos(x_a.yaw), np.sin(x_a.roll) * np.sin(x_a.pitch) * np.cos(x_a.yaw) - np.cos(x_a.roll) * np.sin(x_a.yaw), np.cos(x_a.roll) * np.sin(x_a.pitch) * np.cos(x_a.yaw) + np.sin(x_a.roll) * np.sin(x_a.yaw)],
                                              [np.cos(x_a.pitch) * np.sin(x_a.yaw), np.sin(x_a.roll) * np.sin(x_a.pitch) * np.sin(x_a.yaw) + np.cos(x_a.roll) * np.cos(x_a.yaw), np.cos(x_a.roll) * np.sin(x_a.pitch) * np.sin(x_a.yaw) - np.sin(x_a.roll) * np.cos(x_a.yaw)],
                                              [-np.sin(x_a.pitch), np.sin(x_a.roll) * np.cos(x_a.pitch), np.cos(x_a.roll) * np.cos(x_a.pitch)]])

    # vehicle position calculation
    movement_vehicle = (x_a.vx * time_step, x_a.vy * time_step, x_a.vz * time_step) 

    # TODO check mat mul ordem
    [x_a.x, x_a.y, x_a.z] = [x_a.x, x_a.y, x_a.z] + (movement_vehicle @ vehicle_fixed2inertial_system)

    displacement = Displacement()

    displacement.za[0] = (- param.lv * np.sin(x_a.pitch)) + (param.sl * np.sin(x_a.roll))
    displacement.za[1] = (+ param.lh * np.sin(x_a.pitch)) + (param.sl * np.sin(x_a.roll))
    displacement.za[2] = (- param.lv * np.sin(x_a.pitch)) - (param.sr * np.sin(x_a.roll))
    displacement.za[3] = (+ param.lh * np.sin(x_a.pitch)) - (param.sr * np.sin(x_a.roll))

    return x_a, displacement, movement_vehicle


def variable_initialization(param, data, logger):

    return param, x_a, time_step, x_rf, drag, position_chassi_force, strut2chassi_xyz, angular_rates, polar_inertia_v, logger


def main():
    SIM_ITER = 1000
    logger = LocalLogger("Chassis").logger

    path = "../../exampledata/2_acc_brake/SimulationData.pickle"
    param = ImportParam("../../bmw_m8.yaml")
    logger.info("loaded Params")

    data = import_data_CM(path)
    logger.info("loaded SimulationData")

    exit()

    chassis_variables = [chassis(*variable_initialization(param, data)) for i in range(SIM_ITER)]

    plt.title("chassis")
    plt.plot(chassis_variables)
    plt.show()


if __name__ == '__main__':
    main()
