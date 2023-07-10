from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.utils.LocalLogger import LocalLogger

from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
import yaml


def powertrain(param: ImportParam,
               throttle: float,
               brake: float,
               acc_x: float,
               wheel_w_vel: float,
               gear: float,
               vx: float,
               logger: LocalLogger) -> list:
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
    rpm = param.gear_ratio[gear] * param.diff * \
        np.mean(wheel_w_vel)   # Wheel vx to engine RPM

    rpm = param.rpm_table[0] if rpm < param.rpm_table[0] else param.rpm_table[-1] 

    # Calculate torque provided by the engine based on the engine RPM
    # how much torque is available thoughout the rpm range
    torque_interpolation = interp1d(param.rpm_table, param.torque_max_table)
    # Max torque available at rpm
    torque_available = torque_interpolation(rpm)
    # find the torque delivered by te engine
    engine_torque = throttle * torque_available

    # Gearbox up or down shifting
    if vx > param.gear_selection[int(throttle * 10)][gear]:
        gear = gear + 1
        if gear >= param.gear_ratio.size:
            gear = param.gear_ratio.size - 1
    elif vx < 0.8 * param.gear_selection[int(throttle * 10)][gear - 1]:
        gear = gear - 1
        if gear < 1:
            gear = 1

    traction_torque = (engine_torque * param.gear_ratio[gear] * param.diff * param.diff_ni * param.transmition_ni) - ((
        ((param.engine_inertia + param.axel_inertia + param.gearbox_inertia) * (gear ** 2)) + (
            param.shaft_inertia * param.gear_ratio[gear] * (param.diff ** 2)) + param.wheel_inertia) * acc_x)

    # --------------------Break Torque -------------------------
    brake_torque = brake * param.max_brake_torque

    # -------------------- Total Torque -------------------------
    powertrain_net_torque = (traction_torque - brake_torque) * param.brake_bias

    return [rpm, gear, powertrain_net_torque]


def variable_initialization(param, data, logger):

    return param, rpm_table, torque_max_table, gear_ratio, diff,diff_ni, transmition_ni, gear_selection, engine_inertia, axel_inertia, gearbox_inertia, shaft_inertia, wheel_inertia, max_brake_torque, brake_bias,time_step, logger

def main():
    SIM_ITER = 1000
    logger = LocalLogger()
    param = ImportParam()
    data = importdataCM()

    plt.title("powertrain")
    data = np.array([powertrain(param, throttle, brake, acc_x, wheel_w_vel, gear, vx, logger) for i in range(SIM_ITER)])
    plt.plot(data[:, 0], label="rpm")
    plt.plot(data[:, 1], label="gear")
    plt.plot(data[:, 2], label="powertrain_net_torque")
    plt.label()
    plt.show()


if __name__ == '__main__':
    main()
