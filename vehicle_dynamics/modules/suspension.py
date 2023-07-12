from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.Displacement import Displacement
from vehicle_dynamics.utils.Initialization import Initialization


import numpy as np
import logging
import yaml


def suspension(parameters: Initialization, logger: logging.Logger):
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

    A = -(parameters.car_parameters.eq_stiff * (-parameters.displacement.za + parameters.displacement.zs + parameters.displacement.l_stat)) + (parameters.car_parameters.dumper * parameters.displacement.za_dot)
    B = parameters.vehicle_fixed2inertial_system @ np.array([[0], [0], [1]])
    parameters.f_zr.wheel_load_z = (A * B)[2]

    logger.debug(f"whell load z {parameters.f_zr.wheel_load_z}")

    return parameters, logger
    return f_zr


def main():
    SIM_TIME = 22
    test_function = powertrain
    function_name = test_function.__name__

    logger = LocalLogger(function_name).logger

    parameters = Initialization("../../Audi_r8.yaml", logger=logger)
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/Powertrain testing/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []
    for i in range(22001):
        parameters.f_zr.wheel_load_z = sim_data[i].wheel_load_z_FL
        parameters.displacement.za = sim_data[i].wheel_load_z_FL
        # parameters.displacement.l_stat =
        # parameters.displacement.za_dot = 
        # parameters.vehicle_fixed2inertial_system =

        data.append(test_function(parameters, logger, throttle = sim_data[i].gas_pedal, brake = sim_data[i].brake_pedal)[0].get_data())  

    plt.figure()
    plt.step(range(22001), [i["gear"] for i in data], "g", label="gear_no_calcu")
    var_name = "gear_no"
    plt.step([i for j, i in enumerate(sim_data.keys()) if j % 100 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 100 == 0], label = var_name)
    plt.legend()
    plt.figure()
    plt.plot([i["x_a.vx"] for i in data])

    plt.show()


if __name__ == '__main__':
    main()
()
