from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.StateVector import StateVector
import numpy as np
import yaml
import logging


def steering(parameters: Initialization, logger: logging.Logger, steering_angle: float):
    """
    steering is a function that calculates the current angle
    of the frontal wheel based on the vehicle coordinate system,
    where the output delta is steering angle of the frontal wheels.

    Required Parameters from Param:
        1. steering_ratio
        2. steering_min
        3. steering_max
        4. steering_v_min
        5. steering_v_max
    Required Arguments:
        1. x_a
            1.01 yaw
            1.02 pitch
        4. last_delta
        5. time_step
        6. wheel_angle_front
        7. wheel_angle_rear
        8. VTR_front_axel
        9. VTR_rear_axel

    Returns:
        2. wheel_angle_front
        3. wheel_angle_rear
        4. VTR_front_axle
        5. VTR_front_axle
        6. last_delta

    """
    delta = steering_angle * parameters.car_parameters.steering_ratio  # converts input [0,1] to steering angle
    delta_dot = float((last_delta - delta) / time_step)  # Delta derivated

    # Restrain the limite of turning angle

    if (delta <= parameters.car_parameters.steering_min and delta_dot <= 0) or (delta >= parameters.car_parameters.steering_max and delta_dot >= 0):  # steering wheel limits
        delta_dot = 0
    elif delta_dot >= parameters.car_parameters.steering_v_max:
        delta_dot = parameters.car_parameters.steering_v_max
    # elif delta_dot <= parameters.car_parameters.steering_v_min:## Unknown why
    #    delta_dot = parameters.car_parameters.steering_v_min

    parameters.last_delta = delta

    # Matrix E_T_R (wheel angles) is calculate at steering fuction      
    # Bardini pag 261 eq. 11-6 (TR1, TR3)
    parameters.wheel_angle_front = [[np.cos(parameters.x_a.yaw + delta), -np.sin(parameters.x_a.yaw + delta), 0],
                                    [np.sin(parameters.x_a.yaw + delta), np.cos(parameters.x_a.yaw + delta), 0],
                                    [0, 0, 1]]

    # Eq.11-8    Schramm and Bardini Pag 261 (TR2, TR4)
    parameters.wheel_angle_rear = [[np.cos(parameters.x_a.yaw), - np.sin(parameters.x_a.yaw), 0],
                                   [np.sin(parameters.x_a.yaw), np.cos(parameters.x_a.yaw), 0],
                                   [0, 0, 1]] 

    non_rotate_steering_wheel = True
    if non_rotate_steering_wheel:
        parameters.VTR_front_axle = np.identity(3)  # VTR Vehicle to Wheel
        parameters.VTR_rear_axle = np.identity(3)
    else:
        # Wheel fixed coordinate(KR) rotation relativ to Kv(vehicle system) Bardni pag. 260 eq. 11-9
        parameters.VTR_front_axle = np.array([[np.cos(delta) * np.cos(parameters.x_a.pitch), -np.sin(delta) * np.cos(parameters.x_a.pitch), -np.sin(parameters.x_a.pitch)],
                                              [(np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.cos(delta)) + (np.cos(parameters.x_a.roll) * np.sin(delta)), (-np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.sin(delta)) + (np.cos(parameters.x_a.roll) * np.cos(delta)), (np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.pitch))],
                                              [(np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.cos(delta)) - (np.sin(parameters.x_a.roll) * np.sin(delta)), (-np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.sin(delta)) - (np.sin(parameters.x_a.roll) * np.cos(delta)), np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.pitch)]])

    # # Bardni. Pag. 260 eq. 11-10
        parameters.VTR_rear_axle = np.array([[np.cos(parameters.x_a.pitch), 0, -np.sin(parameters.x_a.pitch)],
                                             [np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch), np.cos(parameters.x_a.roll), np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.pitch)],
                                             [np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch), - np.sin(parameters.x_a.roll), np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.pitch)]])

    "says rotational transformation to Kv is necessary (VTR_front_axel)>> def vehiclefixed2inertial_system"

    return parameters, logger 


def main():
    SIM_ITER = 1000
    test_function = steering
    function_name = function.__name__

    logger = LocalLogger(function_name).logger

    parameters = Initialization("../../bmw_m8.yaml")
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/2_acc_brake/SimulationData.pickle"

    data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")

    data = [test_function(parameters, logger, steering_angle)[0] for i in range(SIM_ITER)]

    plt.title(function_name)
    plt.plot(data)
    plt.show()


if __name__ == '__main__':
    main()
