from ..utils.ImportParam import ImportParam
from ..structures.TireForces import TireForces
from ..structures.WheelHubForce import WheelHubForce
from ..structures.StateVector import StateVector
import numpy as np
import logging


def tire_model(param: ImportParam,   
               x_rf: TireForces,
               f_zr: WheelHubForce,
               position_chassi_force: np.ndarray,
               slip_x: np.ndarray,
               slip_y: np.ndarray,
               compiled_wheel_forces: np.ndarray,
               VTR_front_axle: np.ndarray,
               VTR_rear_axle: np.ndarray,
               logger: logging.Logger) -> np.ndarray: 
    """ Tire model calculates de wheel forces fx and fy
    using the Magic Formula


    Required Parameters from Param:
        1. d # tyre coeficients 
        2. c # 
        3. b # 
    Required Arguments:
        1. x_rf
            2.01 fx
            2.02 fy
            2.03 wheel_forces_transformed_force2vehicle_sys
        2. f_zr
            3.01 wheel_load_z - 4x1
        3. slip_x
        4. slip_y
        5. compiled_wheel_forces 
        6. VTR_front_axle
        7. VTR_rear_axle


    Returns:
        1. x_rf.fx
            1.01 fx
            1.02 fy
            1.03 wheel_forces_transformed_force2vehicle_sys
        2. strut2chassi_xyz

    """    
    x_rf.fx = f_zr.wheel_load_z * param.d * np.sin(param.c * np.arctan(param.b * slip_x - param.e * (param.b * slip_x - np.arctan(param.b * slip_x))))
    x_rf.fy = f_zr.wheel_load_z * param.d * np.sin(param.c * np.arctan(param.b * slip_y - param.e * (param.b * slip_y - np.arctan(param.b * slip_y))))

    compiled_wheel_forces = np.array([x_rf.fx, x_rf.fy, f_zr.wheel_load_z])

    for i in range(4):
        if i % 2 == 0:
            # TODO: check matrix operation 3x3 3x4>> define wheel_forces_transfomed matrix
            self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] = self.VTR_front_axle @ self.compiled_wheel_forces[i]  # Bardini pag. 267 eq.32
        else: 
            self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] = self.VTR_rear_axle @ self.compiled_wheel_forces[i]

        # 3x4 = 3x3 @ 3x4
    # 3x1 = 3x3 @ 3x1

    x_rf.wheel_forces_transformed_force2vehicle_sys[:, 0] = VTR_front_axle @ compiled_wheel_forces.T[:, 0]
    x_rf.wheel_forces_transformed_force2vehicle_sys[:, 1] = VTR_rear_axle @ compiled_wheel_forces.T[:, 1]
    x_rf.wheel_forces_transformed_force2vehicle_sys[:, 2] = VTR_front_axle @ compiled_wheel_forces.T[:, 2]
    x_rf.wheel_forces_transformed_force2vehicle_sys[:, 3] = VTR_rear_axle @ compiled_wheel_forces.T[:, 3]

    # forces on the vehicle chassis (Ai) >> Bardini pag 236  >> horizontal forces pag 264 f_za.f_za
    strut2chassi_xyz = compiled_wheel_forces

    logger.debug("VTR FRONT AXLE", VTR_front_axle)
    logger.debug("Compiled wheel forces ", compiled_wheel_forces)
    logger.debug("Compiled wheel force to vehicle", x_rf.wheel_forces_transformed_force2vehicle_sys)

    return (x_rf, strut2chassi_xyz)
