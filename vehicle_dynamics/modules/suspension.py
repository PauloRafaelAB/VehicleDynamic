from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.Displacement import Displacement

import numpy as np
import logging
import yaml


def suspension(param: ImportParam,
               f_zr: WheelHubForce,
               displacement: Displacement,
               vehicle_fixed2inertial_system: np.ndarray,
               logger: logging.Logger) -> np.ndarray:  # previsious forces, ax,ay 
    """
     suspension is a function that calculates the current wheel loads (z)

     Required Parameters from Param:
         1. eq_stiff
         2. dumper

     Required Arguments:
         1. f_zr
             1.01 wheel_load_z
        2.displacement
            2.01 za
            2.02 zs
            2.03 l_stat
            2.04 za_dot
        3.vehicle_fixed2inertial_system
     Returns:
         1. f_zr.wheel_load_z

     """
    # Forces on the vehicle chassis at the pivot points Ai
    # Bardini pag. 265 eq. 11-21  

    f_zr.wheel_load_z = -(param.eq_stiff * (displacement.za - displacement.zs + displacement.l_stat) + param.dumper * (displacement.za_dot)) * vehicle_fixed2inertial_system @ np.array([[0], [0], [1]])[2]

    logger.debug("whell load z", f_zr.wheel_load_z)

    return f_zr
