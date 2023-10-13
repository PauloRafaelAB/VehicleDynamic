from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.utils.Initialization import Initialization
from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger

import matplotlib.pyplot as plt
import numpy as np
import logging
import yaml 


def steering(parameters: Initialization, logger: logging.Logger, steering_input: float):
    """
    steering is a function that calculates the current angle
    of the frontal wheel based on the vehicle coordinate system,
    where the output delta is steering angle of the frontal wheels.

    Required Parameters from car_parameters:
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
    
    
    # Convert Steering input [-1,1] to whell steering (delta)
    steering_angle = steering_input * parameters.car_parameters.steering_lock 
    parameters.delta = steering_angle / parameters.car_parameters.steering_ratio

    '''# Calculate the variation of the delta
    delta_dot = float((parameters.last_delta - delta) / parameters.time_step)  # Delta derivated

    
    # Restrain the limite of turning angle
    if (steering_angle <= - parameters.car_parameters.steering_lock and delta_dot <= 0):  # steering wheel limits
        delta_dot = 0
        delta = - parameters.car_parameters.steering_lock/parameters.car_parameters.steering_ratio
    elif (delta >= parameters.car_parameters.steering_lock and delta_dot >= 0): 
        delta_dot = 0
        delta = - parameters.car_parameters.steering_lock/parameters.car_parameters.steering_ratio
    
    #Retrain the velocity of the turning angle
    elif delta_dot >= parameters.car_parameters.steering_v_max/parameters.car_parameters.steering_ratio:
        delta_dot = parameters.car_parameters.steering_v_max/parameters.car_parameters.steering_ratio
        delta = steering_angle/parameters.car_parameters.steering_ratio + delta_dot *parameters.time_step
    
    elif delta_dot <= -parameters.car_parameters.steering_v_max/parameters.car_parameters.steering_ratio:
        delta_dot = -parameters.car_parameters.steering_v_max/parameters.car_parameters.steering_ratio
        delta = steering_angle/parameters.car_parameters.steering_ratio + delta_dot *parameters.time_step

    parameters.last_delta = delta
    '''    

    # Matrix E_T_R (wheel angles) is calculate at steering fuction      
    # Bardini pag 261 eq. 11-6 (TR1, TR3)
    
    '''parameters.wheel_angle_front = [[np.cos(parameters.x_a.yaw + delta), -np.sin(parameters.x_a.yaw + delta), 0],
                                    [np.sin(parameters.x_a.yaw + delta), np.cos(parameters.x_a.yaw + delta), 0],
                                    [0, 0, 1]]

    # Eq.11-8    Schramm and Bardini Pag 261 (TR2, TR4)
    parameters.wheel_angle_rear = [[np.cos(parameters.x_a.yaw), - np.sin(parameters.x_a.yaw), 0],
                                   [np.sin(parameters.x_a.yaw), np.cos(parameters.x_a.yaw), 0],
                                   [0, 0, 1]] '''

    # Wheel fixed coordinate(KR) rotation relativ to Kv(vehicle system) Bardni pag. 260 eq. 11-9
    """parameters.VTR_front_axle = np.array([[np.cos(parameters.delta) * np.cos(parameters.x_a.pitch), -np.sin(parameters.delta) * np.cos(parameters.x_a.pitch), -np.sin(parameters.x_a.pitch)],
                                          [(np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.cos(parameters.delta)) + (np.cos(parameters.x_a.roll) * np.sin(parameters.delta)), (-np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.sin(parameters.delta)) + (np.cos(parameters.x_a.roll) * np.cos(parameters.delta)), (np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.pitch))],
                                          [(np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.cos(parameters.delta)) - (np.sin(parameters.x_a.roll) * np.sin(parameters.delta)), (-np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch) * np.sin(parameters.delta)) - (np.sin(parameters.x_a.roll) * np.cos(parameters.delta)), np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.pitch)]])

    # Bardni. Pag. 260 eq. 11-10
    parameters.VTR_rear_axle = np.array([[np.cos(parameters.x_a.pitch), 0, -np.sin(parameters.x_a.pitch)],
                                         [np.sin(parameters.x_a.roll) * np.sin(parameters.x_a.pitch), np.cos(parameters.x_a.roll), np.sin(parameters.x_a.roll) * np.cos(parameters.x_a.pitch)],
                                         [np.cos(parameters.x_a.roll) * np.sin(parameters.x_a.pitch), - np.sin(parameters.x_a.roll), np.cos(parameters.x_a.roll) * np.cos(parameters.x_a.pitch)]])

    "says rotational transformation to Kv is necessary (VTR_front_axel)>> def vehiclefixed2inertial_system"
    """
    return parameters, logger 


def main():
    test_function = steering
    function_name = test_function.__name__

    logger = LocalLogger(function_name).logger

    parameters = Initialization("../../Audi_r8.yaml", logger=logger)
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/steering data/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []
    steering_angle_v = []
    for i in range(len(sim_data)):
        parameters.x_a.roll = sim_data[i].Vhcl_Roll
        parameters.x_a.pitch = sim_data[i].Vhcl_Pitch
        parameters.x_a.yaw = sim_data[i].Vhcl_Yaw
        steering_angle_calc = sim_data[i].Steering_angle/(4 * np.pi)
        data.append(test_function(parameters, logger, steering_angle_calc)[0].get_data())  
        steering_angle_v.append(steering_angle_calc)
    plt.figure()
    plt.title(function_name)
    plt.plot([i["last_delta"] for i in data], "--", label="delta")
    plt.plot(steering_angle_v, label ="steering_angle_calc")

    var_name = "Steering_angle"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], label = var_name)
    plt.legend()

    plt.show()


if __name__ == '__main__':
    main()
