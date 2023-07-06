from ..utils.ImportParam import ImportParam
from ..structures.StateVector import StateVector
import numpy as np

def suspension(param:ImportParam,
               f_zr:WheelHubForce,
               displacement:Displacement,
               logger:logging.Logger) -> np.ndarray::  # previsious forces, ax,ay 
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
     Returns:
         1. f_zr.wheel_load_z
    
     """

    # Forces on the vehicle chassis at the pivot points Ai
    # Bardini pag. 265 eq. 11-21  

    # TODO adicionar zs dot VERIFICAR SE É ZA_DOT-ZS_DOT ou se é do outro jeito 
    # displacement.za = np.array([0,0,0,0]) # CONSIDERANDO QUE NAO HÁ TRANSFERENCIA DE CARGA
    f_zr.wheel_load_z = (param.eq_stiff * (-displacement.za + displacement.zs + displacement.l_stat
                                                     ) + param.dumper * (displacement.za_dot)) * np.matmul(transpose_vehicle_fixed2inertial_system, np.array([[0], [0], [1]]))[2]

    # NO MEU PONTO DE VISTA AQUI VOCE CALCULARIA COMO AS FORCAS QUE AGEM NO CG ATUAM NOS PO
    logger.debug("displacement.za", displacement.za)
    logger.debug("whell load z", f_zr.wheel_load_z)
    # V_F_fi - Forces on tires action on the chassis at the pivot points Ai of the four wheel >> Bardini pag 263

    # TODO: check transpose_vehicle_fixed2inertial_system operation  
