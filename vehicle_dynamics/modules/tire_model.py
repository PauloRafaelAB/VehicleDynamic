from ..utils.ImportParam import ImportParam
from ..structures.TireForces import TireForces
from ..structures.StateVector import StateVector
import numpy as np


def tire_model(param: ImportParam,   
               x_rf: TireForces,
               f_zr: WheelHubForce,
               position_chassi_force: np.ndarray,
               slip_x: np.ndarray,
               slip_y: np.ndarray,
               compiled_wheel_forces: np.ndarray,
               VTR_front_axel: np.ndarray,
               VTR_rear_axel: np.ndarray,
               logger:logging.Logger) -> np.ndarray: 
    
    """ Tire model calculates de wheel forces fx and fy
    using the Magic Formula


    Required Parameters from Param:
        1. d
        2. c
        3. b
    Required Arguments:
        1. x_rf
            2.01 fx
            2.02 fy
            2.03 wheel_forces_transformed_force2vehicle_sys
        2. f_zr
            3.01 wheel_load_z
        3. slip_x
        4. slip_y
        5. compiled_wheel_forces 
        6. VTR_front_axel
        7. VTR_rear_axel


    Returns:
        1. x_rf.fx
        2. x_rf.fy
        3. x_rf.wheel_forces_transformed_force2vehicle_sys
        4. strut2chassi_xyz

    """
    # Input slip, fz - fx_car, fy_car
    
    logger.debug('wheel_load_z', f_zr.wheel_load_z)
    x_rf.fx = f_zr.wheel_load_z * param.d * np.sin(param.c * np.arctan(param.b * slip_x - param.e * (param.b * slip_x - np.arctan(param.b * slip_x))))
    x_rf.fy = f_zr.wheel_load_z * param.d * np.sin(param.c * np.arctan(param.b * slip_y - param.e * (param.b * slip_y - np.arctan(param.b * slip_y))))

    for i in range(4):

        # 3x4
        compiled_wheel_forces[i] = np.array([x_rf.fx[i], x_rf.fy[i], f_zr.wheel_load_z[i]])
        # logger.debug('compiled_wheel_forces',compiled_wheel_forces,type(compiled_wheel_forces),compiled_wheel_forces.shape)

    """if i % 2 == 0:
        #TODO: check matrix operation 3x3 3x4>> define wheel_forces_transfomed matrix
        self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] =  np.matmul(self.VTR_front_axel,self.compiled_wheel_forces[i]) # Bardini pag. 267 eq.32
        
    else: 
        self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] =  np.matmul(self.VTR_rear_axel,self.compiled_wheel_forces[i])
        
    """
    # 3x4 = 3x3 @ 3x4
    # 3x1 = 3x3 @ 3x1

    x_rf.wheel_forces_transformed_force2vehicle_sys[:, 0] = np.matmul(VTR_front_axel, compiled_wheel_forces.T[:, 0])
    x_rf.wheel_forces_transformed_force2vehicle_sys[:, 1] = np.matmul(VTR_rear_axel, compiled_wheel_forces.T[:, 1])
    x_rf.wheel_forces_transformed_force2vehicle_sys[:, 2] = np.matmul(VTR_front_axel, compiled_wheel_forces.T[:, 2])
    x_rf.wheel_forces_transformed_force2vehicle_sys[:, 3] = np.matmul(VTR_rear_axel, compiled_wheel_forces.T[:, 3])

    # forces on the vehicle chassis (Ai) >> Bardini pag 236  >> horizontal forces pag 264 f_za.f_za
    strut2chassi_xyz = compiled_wheel_forces

    logger.debug("VTR FRONT AXLE", VTR_front_axel)
    logger.debug("Compiled wheel forces ", compiled_wheel_forces)
    logger.debug("Compiled wheel force to vehicle", x_rf.wheel_forces_transformed_force2vehicle_sys)
