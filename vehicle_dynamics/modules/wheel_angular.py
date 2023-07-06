from ..utils.ImportParam import ImportParam
from ..structures.TireForces import TireForces
from ..structures.StateVector import StateVector
import numpy as np


def wheel_angular(param: ImportParam,
                  x_rf: TireForces,
                  powertrain_net_torque: np.ndarray,
                  x_rr:np.ndarray,
                  wheel_w_vel,
                  
        ):
    """
    wheel_angular velocities velocities are used to calcule slip

    Required Parameters from Param:
        1. 
        2. 

    Required Arguments:
        1. 
            1.01 
       2. 
           2.01 
           2.02 
           2.03 
           2.04 
       3. 
    Returns:
        1. 
   
    """
   
    # EULER’s Equations of the Wheels: Solve  11-25 Bardini pag 266
    # Torque, fx_car, fy_car, Inertias wheel,  output wheel Wv, dot_wv
    if self.powertrain_net_torque[0] > (self.x_rf.fx[0] / self.param.r_dyn[0]):
        self.x_rr.pho_r_2dot = (self.powertrain_net_torque - self.x_rf.fx / self.param.r_dyn) / self.param.iw
    else:
        self.x_rr.pho_r_2dot = self.powertrain_net_torque / self.param.iw

    print('torque fx', self.x_rf.fx / self.param.r_dyn)
    print('powertrain net torque', self.powertrain_net_torque)

    print("pho_r_2dot     ", self.x_rr.pho_r_2dot)
    print("FX_________", self.x_rf.fx)

    # Calculate the intregral of the wheel_acc to calculate slip_x
    self.wheel_w_vel = self.wheel_w_vel + self.x_rr.pho_r_2dot * self.time_step  # rad/s
    # ___________________________________________________________________________________________________________
