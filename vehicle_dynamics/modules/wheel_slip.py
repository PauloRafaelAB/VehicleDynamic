from ..utils.ImportParam import ImportParam
from ..structures.StateVector import StateVector
import numpy as np
import logging


def wheel_slip(param: ImportParam,
               x_a: StateVector,
               wheel_w_vel: np.ndarray,
               logger: logging.Logger) -> tuple:
    # input: fz, mi, wheel_angle,vx,vy   output:fz,slix,slip_y
    """ This method calculate the wheel slip on each wheel. This is done using the relative 
    velocity between the wheel angular speed and the vehicle speed.

        Required Parameters from Param:
            1. r_dyn
        Required Arguments:
            1. x_a
                1.01 vx
                1.02 vy
            2. slip_x
            3. slip_y
            4. wheel_w_vel

        Returns:
            1. slip_x
            2. slip_y


    """
    slip_x, slip_y = np.zeros([1, 4]), np.zeros([1, 4])

    vx_4lines = np.ones([1, 4]) * x_a.vx
    vy_4lines = np.ones([1, 4]) * x_a.vy

    if (abs(param.r_dyn * wheel_w_vel) == 0) and (abs(x_a.vx) == 0):
        slip_x = 0
        slip_y = 0
    else:
        # equation 11.30 Bardini
        slip_x = ((((param.r_dyn * wheel_w_vel) - x_a.vx) / max([abs(param.r_dyn * wheel_w_vel), abs(x_a.vx)])))         
        # equation 11.31 Bardini
        slip_y = - np.arctan(x_a.vy / max([abs(param.r_dyn * wheel_w_vel), abs(x_a.vx)]))  

    logger.debug("SLIP X ", slip_x)
    return (slip_x, slip_y)
