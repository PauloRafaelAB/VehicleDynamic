from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.utils.Initialization import Initialization


import numpy as np
import logging
import yaml


def tire_model(parameters: Initialization, logger: logging.Logger):
    """ Tire model calculates de wheel forces fx and fy
    using the Magic Formula


    Required Parameters from car_parameters:
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
        6. parameters.VTR_front_axle
        7. parameters.VTR_rear_axle


    Returns:
        1. parameters.x_rf.fx
            1.01 fx
            1.02 fy
            1.03 wheel_forces_transformed_force2vehicle_sys
        2. strut2chassi_xyz

    """    
    parameters.x_rf.fx = parameters.f_zr.wheel_load_z * parameters.car_parameters.d * np.sin(parameters.car_parameters.c * np.arctan(parameters.car_parameters.b * parameters.slip_x - parameters.car_parameters.e * (parameters.car_parameters.b * parameters.slip_x - np.arctan(parameters.car_parameters.b * parameters.slip_x))))
    parameters.x_rf.fy = parameters.f_zr.wheel_load_z * parameters.car_parameters.d * np.sin(parameters.car_parameters.c * np.arctan(parameters.car_parameters.b * parameters.slip_y - parameters.car_parameters.e * (parameters.car_parameters.b * parameters.slip_y - np.arctan(parameters.car_parameters.b * parameters.slip_y))))

    parameters.compiled_wheel_forces = np.array([parameters.x_rf.fx, parameters.x_rf.fy, parameters.f_zr.wheel_load_z])

    delta = np.array([parameters.delta,0.0,parameters.delta,0.0])## FL, RL, FR, RR
    for i in range(4):
        parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[0, i] = parameters.x_rf.fx[i] * np.cos(delta[i]) + parameters.x_rf.fy[i] * np.sin(delta[i])
        parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[1, i] = parameters.x_rf.fy[i] * np.cos(delta[i]) + parameters.x_rf.fx[i] * np.sin(delta[i]) 

    """ for i in range(4):
        if i % 2 == 0:
            # TODO: check matrix operation 3x3 3x4>> define wheel_forces_transfomed matrix
            parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[:, i] = (parameters.VTR_front_axle @ np.reshape(parameters.compiled_wheel_forces[:, i], (3, 1))).T  # Bardini pag. 267 eq.32


    parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 0] = parameters.x_rf.fx[0]  
    parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 1] = 
    parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 2] =  
    parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 3] = 

    for i in range(4):
        if i % 2 == 0:
            # TODO: check matrix operation 3x3 3x4>> define wheel_forces_transfomed matrix
            parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[:, i] = (parameters.VTR_front_axle @ np.reshape(parameters.compiled_wheel_forces[:, i], (3, 1))).T  # Bardini pag. 267 eq.32
        else: 
            parameters.x_rf.wheel_forces_transformed_force2vehicle_sys[:, i] = (parameters.VTR_rear_axle @ np.reshape(parameters.compiled_wheel_forces[:, i], (3, 1))).T
            	"""
    # 3x4 = 3x3 @ 3x4
    # 3x1 = 3x3 @ 3x1
    """logger.debug(f"{parameters.VTR_front_axle},{parameters.compiled_wheel_forces.T[:, 0]}")

    # forces on the vehicle chassis (Ai) >> Bardini pag 236  >> horizontal forces pag 264 f_za.f_za
    #parameters.strut2chassi_xyz = parameters.compiled_wheel_forces
    
    logger.debug(f"VTR FRONT AXLE {parameters.VTR_front_axle}")
    logger.debug(f"Compiled wheel forces  {parameters.compiled_wheel_forces}")
    logger.debug(f"Compiled wheel force to vehicle {parameters.x_rf.wheel_forces_transformed_force2vehicle_sys}")
    """
    return parameters, logger 


def main():
    SIM_ITER = 1000
    test_function = tire_model
    function_name = function.__name__

    logger = LocalLogger(function_name).logger

    parameters = Initialization("../../bmw_m8.yaml")
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/2_acc_brake/SimulationData.pickle"

    data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")

    data = [test_function(parameters, logger)[0] for i in range(SIM_ITER)]

    plt.title(function_name)
    plt.plot(data)
    plt.show()


if __name__ == '__main__':
    main()
