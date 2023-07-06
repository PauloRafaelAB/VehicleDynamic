from ..utils.ImportParam import ImportParam
from ..structures.TireForces import TireForces
from ..structures.StateVector import StateVector
import numpy as np


def wheel_angular(param: ImportParam,
                  powertrain_net_torque: np.ndarray
                  x_rf: TireForces,
                  x_rr: AngularWheelPosition,
                  wheel_w_vel:np,ndarray,
                  time_step: float
                  logger.debug): -> np.ndarray:
    """
    wheel_angular velocities velocities are used to calcule slip

    Required Parameters from Param:
        1. r_dyn
        2. iw

    Required Arguments:
        1. powertrain_net_torque
        2.x_rf 
           2.01 fx 
       3. x_rr
           3.01 pho_r_2dot
       4. wheel_w_vel
           
    Returns:
        1. 
   
    """
   
    # EULER’s Equations of the Wheels: Solve  11-25 Bardini pag 266
    # Torque, fx_car, fy_car, Inertias wheel,  output wheel Wv, dot_wv
    if powertrain_net_torque[0] > (x_rf.fx[0] / param.r_dyn[0]):
        x_rr.pho_r_2dot = (powertrain_net_torque - x_rf.fx / param.r_dyn) / param.iw
    else:
        x_rr.pho_r_2dot = powertrain_net_torque / param.iw

    logger.debug('torque fx', x_rf.fx / param.r_dyn)
    logger.debug('powertrain net torque', powertrain_net_torque)

    logger.debug("pho_r_2dot     ", x_rr.pho_r_2dot)
    logger.debug("FX_________", x_rf.fx)

    # Calculate the intregral of the wheel_acc to calculate slip_x
    wheel_w_vel = wheel_w_vel + (x_rr.pho_r_2dot * time_step)  # rad/s