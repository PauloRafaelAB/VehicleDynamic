from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.AngularWheelPosition import AngularWheelPosition

import numpy as np
import logging
import yaml


def wheel_angular(param: ImportParam,
                  powertrain_net_torque: np.ndarray,
                  x_rf: TireForces,
                  x_rr: AngularWheelPosition,
                  wheel_w_vel: np.ndarray,
                  time_step: float,
                  logger: logging.Logger) -> np.ndarray:
    """
    wheel_angular velocities velocities are used to calcule slip

    Required Parameters from Param:
        1. r_dyn #dynamic radius
        2. iw # wheel inertia

    Required Arguments:
        1. powertrain_net_torque
        2.x_rf 
           2.01 fx 
       3. x_rr
           3.01 pho_r_2dot # wheel angular acceleration
       4. wheel_w_vel # wheel angular velocity

    Returns:
        1. wheel_w_vel
        2. x_rr

    """

    # EULER’s Equations of the Wheels: Solve  11-25 Bardini pag 266
    # Torque, fx_car, fy_car, Inertias wheel,  output wheel Wv, dot_wv
    x_rr.pho_r_2dot = (powertrain_net_torque / param.iw) - ((x_rf.fx * r_dyn) / param.iw) 

    logger.debug('torque fx', x_rf.fx / param.r_dyn)
    logger.debug('powertrain net torque', powertrain_net_torque)

    logger.debug("pho_r_2dot     ", x_rr.pho_r_2dot)
    logger.debug("FX_________", x_rf.fx)

    # Calculate the integral of the wheel_acc to calculate slip_x
    wheel_w_vel = wheel_w_vel + (x_rr.pho_r_2dot * time_step)  # rad/s

    return wheel_w_vel, x_rr
