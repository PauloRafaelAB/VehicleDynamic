from ..utils.ImportParam import ImportParam
from ..structures.StateVector import StateVector
import numpy as np

def wheel_slip(param:ImportParam,
               x_a:StateVector,
               wheel_w_vel: np.ndarray) -> np.ndarray:
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

        initial states of tire load are mandatory [Wheel torque], [Wheel_Fz], slip_x, slip_angle]
        calculate the forces.
        
    """
    
    for i in range(4):

        # Slip calculation - Wheel slip is calculated by wheel fixed system

        # if param.r_dyn[i] * wheel_w_vel[i] == abs(x_a.vx):
        #     slip_x[i] = .0
        # elif abs(param.r_dyn[i] * wheel_w_vel[i]) > abs(x_a.vx):
        #     slip_x[i] = 1 - abs(x_a.vx/(param.r_dyn[i] * wheel_w_vel[i] + 1e-52))
        # else:
        #     slip_x[i] = -1 + abs(param.r_dyn[i] * wheel_w_vel[i]/(x_a.vx + 1e-52))

        slip_x[i] = (((param.r_dyn[i] * wheel_w_vel[i] - x_a.vx) / max([abs(param.r_dyn[i] * wheel_w_vel[i] + 1e-26), abs(1e-26 + x_a.vx)])))         # equation 11.30 Bardini
        # REMOVED THE ABS()
        # print('wheel_w_vel[i]',wheel_w_vel[i])
        # print(x_a.vx,'vx')

        # Lateral slip: define wheel_vy Equacao 11_28 >> converte to tire direction
        slip_y[i] = - np.arctan(x_a.vy / max([abs(param.r_dyn[i] * wheel_w_vel[i] + 1e-16), abs(1e-16 + x_a.vx)]))  # equation 11.31 Bardini

        # _______________________________________________________________________________________________
        # TODO: Check first argument by displaying (velocity in wheel fixed coord) Bardini eq 11-29

        # Bardini pag.268 eq 11-33
        ' Replace with followig code to take antiroll bar in to account'
        # TODO: Make F_stv diferent than 0; make F_st input on tire fuction
        # wheel_load_z[0] = - max(param.cr[0]*(displacement.zr[0]-displacement.zs[0]+lr_stat[0]) + F_stv , 0) # Bardini pag.268 eq 11-33
    print("SLIP X ", slip_x)
