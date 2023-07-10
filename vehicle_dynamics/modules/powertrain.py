from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.utils.LocalLogger import LocalLogger

from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
import yaml


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
    rpm = parameters.car_parameters.gear_ratio[parameters.gear] * parameters.car_parameters.diff * \
        np.mean(parameters.wheel_w_vel)   # Wheel vx to engine RPM

    rpm = parameters.car_parameters.rpm_table[0] if rpm < parameters.car_parameters.rpm_table[0] else parameters.car_parameters.rpm_table[-1] 

    # Calculate torque provided by the engine based on the engine RPM
    # how much torque is available thoughout the rpm range
    torque_interpolation = interp1d(parameters.car_parameters.rpm_table, parameters.car_parameters.torque_max_table)
    # Max torque available at rpm
    torque_available = torque_interpolation(rpm)
    # find the torque delivered by te engine
    engine_torque = throttle * torque_available

    # Gearbox up or down shifting
    if parameters.vx > parameters.car_parameters.gear_selection[int(throttle * 10)][parameters.gear]:
        parameters.gear = parameters.gear + 1
        if parameters.gear >= parameters.car_parameters.gear_ratio.size:
            parameters.gear = parameters.car_parameters.gear_ratio.size - 1
    elif parameters.vx < 0.8 * parameters.car_parameters.gear_selection[int(parameters.throttle * 10)][parameters.gear - 1]:
        parameters.gear = parameters.gear - 1
        if parameters.gear < 1:
            parameters.gear = 1

    traction_torque = (engine_torque * parameters.car_parameters.gear_ratio[parameters.gear] * parameters.car_parameters.diff * parameters.car_parameters.diff_ni * parameters.car_parameters.transmition_ni) - ((
        ((parameters.car_parameters.engine_inertia + parameters.car_parameters.axel_inertia + parameters.car_parameters.gearbox_inertia) * (parameters.gear ** 2)) + (
            parameters.car_parameters.shaft_inertia * parameters.car_parameters.gear_ratio[parameters.gear] * (parameters.car_parameters.diff ** 2)) + parameters.car_parameters.wheel_inertia) * parameters.acc_x)

    # --------------------Break Torque -------------------------
    brake_torque = brake * parameters.car_parameters.max_brake_torque

    # -------------------- Total Torque -------------------------
    powertrain_net_torque = (traction_torque - brake_torque) * parameters.car_parameters.brake_bias
    return parameters, logger 


def main():
    SIM_ITER = 1000
    test_function = powertrain
    function_name = function.__name__

    logger = LocalLogger(function_name).logger

    parameters = Initialization("../../bmw_m8.yaml")
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/2_acc_brake/SimulationData.pickle"

    data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")

    data = [test_function(parameters, logger, throttle = throttle, brake = brake)[0] for i in range(SIM_ITER)]

    plt.title(function_name)
    plt.plot(data.rpm[:, 0], label="rpm")
    plt.plot(data.gear[:, 1], label="gear")
    plt.plot(data.powertrain_net_torque[:, 2], label="powertrain_net_torque")
    plt.label()
    plt.show()


if __name__ == '__main__':
    main()
