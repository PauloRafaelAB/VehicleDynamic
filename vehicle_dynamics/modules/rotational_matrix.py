from ..utils.ImportParam import ImportParam
from ..structures.StateVector import StateVector
import numpy as np
import logging


def rotational_matrix(x_a: StateVector,
                      angular_vel_2inercial_sys_in_vehicle_coord: np.ndarray,
                      rotationalmatrix: np.ndarray,
                      angular_rates: np.ndarray,
                      logger: logging.Logger) -> np.ndarray:   # ONDE ESTÁ ATUALIZANDO OS ANGULOS?-------------------------------------------------------------
    """
    rotational_matrix calculates the Eutler angle conversion from vehicle system to inertial system. 

    Required Arguments:
        1. x_a
            1.01 roll
            1.02 pitch
        2. angular_vel_2inercial_sys_in_vehicle_coord
        3. rotationalmatrix
        4. angular_rates

    Returns:
        1. rotationalmatrix
        2. angular_vel_2inercial_sys_in_vehicle_coord

    """

# For the angular velocity of the chassis we have:   Bardini Pag. 261 Eq. 11.5
    rotationalmatrix = [[-np.sin(x_a.pitch), 0, 1],
                        [np.cos(x_a.pitch) * np.sin(x_a.roll), np.cos(x_a.roll), 0],
                        [np.cos(x_a.pitch) * np.cos(x_a.roll), - np.sin(x_a.roll), 0]]

    # Pitch_dot, roll_dot, yaw_dot -- Bardini Pag. 261 Eq. 11.4
    angular_rates = np.array([x_a.wx,
                              x_a.wy,
                              x_a.wz])  # It is already calculated in the rotationalmatrix CALCULATE Only once please

    # Coordinate representation of the absolute velocity of point v with respect to coordinate system “E”, described in coordinates of coordinate system “v” bardni. Pag 260
    angular_vel_2inercial_sys_in_vehicle_coord = rotationalmatrix @ angular_rates  # Bardini Pag. 261 Eq. 11.4    

    return angular_vel_2inercial_sys_in_vehicle_coord, rotationalmatrix
