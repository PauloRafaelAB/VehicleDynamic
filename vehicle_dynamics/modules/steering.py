from ..utils.ImportParam import ImportParam
from ..structures.StateVector import StateVector
import numpy as np


def steering(param,x_a: StateVector,
             steering:float,
             delta_dot:float,
             last_delta:float,
             time_step:float,
             wheel_angle_front: np.ndarray,
             wheel_angle_rear: np.ndarray,
             VTR_front_axel: np.ndarray,
             VTR_rear_axel: np.ndarray):
    
    """
    steering is a function that calculates the current angle
    of the frontal wheel based on the vehicle coordinate system,
    where the output delta is steering angle of the frontal wheels.

    vertical_loads, ax, ay, t_step

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
        3. delta_dot
        4. last_delta
        5. time_step
        6. wheel_angle_front
        7. wheel_angle_rear
        8. VTR_front_axel
        9. VTR_rear_axel

    Returns:
        1. delta
        2. wheel_angle_front
        3. wheel_angle_rear

    """
    delta = steering * param.steering_ratio 
    delta_dot = float((last_delta - delta) / time_step)
    
    # Restrain the limite of turning angle

    if (delta <= param.steering_min and delta_dot <= 0) or (delta >= param.steering_max and delta_dot >= 0):
        delta_dot = 0
    elif delta_dot <= param.steering_v_min:
        delta_dot = param.steering_v_min
    elif delta_dot >= param.steering_v_max:
        delta_dot = param.steering_v_max

    last_delta = delta
 
    # Matrix E_T_R (wheel angles) is calculate at steering fuction      
    # Bardini pag 261 eq. 11-6 (TR1, TR3)
    wheel_angle_front = [[np.cos(x_a.yaw + delta), -np.sin(x_a.yaw + delta), 0],
                              [np.sin(x_a.yaw + delta), np.cos(x_a.yaw + delta), 0],
                              [0, 0, 1]]

    # Eq.11-6    Schramm and Bardini Pag 261 (TR2, TR4)
    wheel_angle_rear = [[np.cos(x_a.yaw), - np.sin(x_a.yaw), 0],
                             [np.sin(x_a.yaw), np.cos(x_a.yaw), 0],
                             [0, 0, 1]] 

    VTR_front_axel = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    VTR_rear_axel = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])

    # Wheel fixed coordinate(KR) rotation relativ to Kv(vehicle system) Bardni pag. 260 eq. 11-9
    # VTR_front_axel = np.array([[                                                 np.cos(delta) * np.cos(x_a.pitch),                                                -np.sin(delta) * np.cos(x_a.pitch),                          -np.sin(x_a.pitch)],
    #                                [ np.sin(x_a.yaw) * np.sin(x_a.pitch) * np.cos(delta) + np.cos(x_a.yaw) * np.sin(delta),     -np.sin(x_a.yaw) * np.sin(x_a.pitch) * np.sin(delta) + np.cos(x_a.yaw) * np.cos(delta),            np.sin(x_a.yaw)* np.cos(x_a.pitch)],
    #                                [ np.cos(x_a.yaw) * np.sin(x_a.pitch) * np.cos(delta) - np.sin(x_a.yaw) * np.sin(delta),     -np.cos(x_a.yaw) * np.sin(x_a.pitch) * np.sin(delta) - np.sin(x_a.yaw) * np.cos(delta),            np.cos(x_a.yaw)* np.cos(x_a.pitch)]])

    # # Bardni. Pag. 260 eq. 11-10
    # VTR_rear_axel = np.array([[                  np.cos(x_a.pitch),                                  0,                                   -np.sin(x_a.pitch)],
    #                               [ np.sin(x_a.yaw) * np.sin(x_a.pitch),     np.cos(x_a.yaw),            np.sin(x_a.yaw)* np.cos(x_a.pitch)],
    #                               [ np.cos(x_a.yaw) * np.sin(x_a.pitch),   - np.sin(x_a.yaw),            np.cos(x_a.yaw)* np.cos(x_a.pitch)]])

    "says rotational transformation to Kv is necessary (VTR_front_axel)>> def vehiclefixed2inertial_system"

    # return self.wheel_angle_front, self.wheel_angle_rear
