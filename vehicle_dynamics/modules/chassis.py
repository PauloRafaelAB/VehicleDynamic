from ..utils.ImportParam import ImportParam
from ..structures.TireForces import TireForces
from ..structures.StateVector import StateVector
import numpy as np
from ..utils.LocalLogger import LocalLogger
import matplotlib.pyplot as plt


def chassis(param: ImportParam,
            x_a: StateVector,
            time_step: float,
            x_rf: TireForces,
            drag: float,
            position_chassi_force: np.ndarray,
            strut2chassi_xyz: np.ndarray,
            angular_rates: np.ndarray,
            polar_inertia_v: np.ndarray,
            logger: LocalLogger) -> np.ndarray: 
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
        2. x_a.acc
        3. x_a.vx
        4. x_a.vy
        5. x_a.vz
        6. displacement.za

    """
    "Equations of motion Bardini, pag 272 ---- need initialize values"

    'sum of  wheel forces for calculating translation of the vehicle'

    sum_f_wheel = np.sum(x_rf.wheel_forces_transformed_force2vehicle_sys, axis=1)

    logger.debug("FORCES IN THE VEHICLE ", x_rf.wheel_forces_transformed_force2vehicle_sys)
    logger.debug("Sum wheel ", sum_f_wheel)
    # Equation 11-46 >> 11-12, Pag. 273
    # TODO: check gavity diretion

    # O que faz esses valores atras da acceleracao??????????????
    x_a.acc[0] = ((sum_f_wheel[0] - (drag * (x_a.vx ** 2))) / param.m) + ((x_a.wz * x_a.vy) - (x_a.wy * x_a.vz))
    x_a.acc[1] = ((sum_f_wheel[1] - (x_a.vy ** 2)) / param.m) + ((x_a.wx * x_a.vz) - (x_a.wz * x_a.vx))
    x_a.acc[2] = ((sum_f_wheel[2] - (param.m * param.gravity)) / param.m) + ((x_a.wy * x_a.vx) - (x_a.wx * x_a.vy))

    # vehicle velocity calculation
    velocity = x_a.acc * time_step
    x_a.vx = x_a.vx + velocity[0] 
    x_a.vy = x_a.vy + velocity[1] 
    x_a.vz = x_a.vz + velocity[2] 

    # TODO:Check rolling resistance            
    # rolling_resist = (param.fr * param.m * param.gravity * np.cos(0.) - 0.5 * param.row * param.Cl * param.area * speed ** 2)                              # Rolling Resistance with air lift

    '''Equation 11-46, vehicle velocity xyz calculation: for angular velocities and postion the last step is used
                Values calculate for positions, atittude and its derivates needs to be saved
            '''
    for (chassis_force, strut2chass, cross_product) in zip(position_chassi_force, strut2chassi_xyz, crossproduct_r_f):
        cross_product[0] = (chassis_force[1] * strut2chass[2]) - (chassis_force[2] * strut2chass[1])
        cross_product[1] = (chassis_force[2] * strut2chass[0]) - (chassis_force[0] * strut2chass[2])
        cross_product[2] = (chassis_force[0] * strut2chass[1]) - (chassis_force[1] * strut2chass[0])

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
    x_a.wx = x_a.wx + x_a.acc_angular_v[0]
    x_a.wy = x_a.wy + x_a.acc_angular_v[1]
    x_a.wz = x_a.wz + x_a.acc_angular_v[2]

    # Angular position   
    x_a.roll = (x_a.wx * time_step) + x_a.roll
    x_a.pitch = (x_a.wy * time_step) + x_a.pitch
    x_a.yaw = (x_a.wz * time_step) + x_a.yaw

    # TODO: updated transformation to vehicle system

    # vehicle_fixed2inertial_system = np.array([[np.cos(x_a.pitch) * np.cos(x_a.yaw), np.sin(x_a.roll) * np.sin(x_a.pitch) * np.cos(x_a.yaw) - np.cos(x_a.roll) * np.sin(x_a.yaw),     np.cos(x_a.roll) * np.sin(x_a.pitch) * np.cos(x_a.yaw) + np.sin(x_a.roll) * np.sin(x_a.yaw)],
    #                                       [np.cos(x_a.pitch) * np.sin(x_a.yaw), np.sin(x_a.roll) * np.sin(x_a.pitch) * np.sin(x_a.yaw) + np.cos(x_a.roll) * np.sin(x_a.yaw),     np.cos(x_a.roll) * np.sin(x_a.pitch) * np.sin(x_a.yaw) - np.sin(x_a.roll) * np.cos(x_a.yaw)],
    #                                       [-np.sin(x_a.pitch),                         np.sin(x_a.roll) * np.cos(x_a.pitch),                                                      np.cos(x_a.roll) * np.cos(x_a.pitch)]])

    # bardini pag 260 -- use vector of torque x euler angle rate

    # vehicle position calculation
    movement_vehicle = (x_a.vx * time_step, x_a.vy * time_step, x_a.vz * time_step) 

    # TODO check mat mul ordem
    [x_a.x, x_a.y, x_a.z] = [x_a.x, x_a.y, x_a.z] + (movement_vehicle @ vehicle_fixed2inertial_system)

    displacement.za[0] = (- param.lv * np.sin(x_a.pitch)) + (param.sl * np.sin(x_a.roll))
    displacement.za[1] = (+ param.lh * np.sin(x_a.pitch)) + (param.sl * np.sin(x_a.roll))
    displacement.za[2] = (- param.lv * np.sin(x_a.pitch)) - (param.sr * np.sin(x_a.roll))
    displacement.za[3] = (+ param.lh * np.sin(x_a.pitch)) - (param.sr * np.sin(x_a.roll))

    return np.array([x_a, displacement])


def main():
    SIM_ITER = 1000
    logger = LocalLogger()
    param = ImportParam()
    data = importdataCM()

    plt.title("chassis")
    plt.plot([chassis(param, x_a, time_step, x_rf, drag, position_chassi_force, strut2chassi_xyz, angular_rates, polar_inertia_v, logger) for i in range(SIM_ITER)])
    plt.show()


if __name__ == '__main__':
    main()
