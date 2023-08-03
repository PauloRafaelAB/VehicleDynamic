from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.Initialization import Initialization
from vehicle_dynamics.utils.import_data_CM import import_data_CM

import logging
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
import yaml
from tqdm import tqdm


def find_nearest(array, value):
    """find nearest from: https://stackoverflow.com/a/2566508 by Mateen Ulhaq"""
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]


def powertrain(parameters: Initialization, logger: logging.Logger, throttle: float, brake: float):
    """
    Powertrain is a function that calculates the current Torque delivered by the engine to the wheels

    Required Parameters from Param:
        1.rpm_table,
        2.torque_max_table,
        3.gear_ratio,
        4.diff,
        5.diff_ni,
        6.transmition_ni,
        7.gear_selection,
        8.engine_inertia,
        9.axel_inertia,
        10.gearbox_inertia,
        11.shaft_inertia,
        12.wheel_inertia,
        13.max_brake_torque,
        14.brake_bias,

    Required Arguments:
        1. throttle
        2. brake
        3. acc_x
        4. wheel_w_vel
        5. gear[t-1]
        6. vx

    Returns:
        1. Engine_rpm
        2. Gear
        3. Torque of the engine on the wheels

    """

    # Update Converter RPM
    # Based on page 103/104 from Vehicle Dynamics and control (Rajesh Rajamani)
    ## Based on the whell velocity and engine rpm the torque on the 
    converter_w = parameters.car_parameters.gear_ratio[parameters.gear] * parameters.car_parameters.diff * np.mean(parameters.wheel_w_vel)  # Wheel vx to converter angular velocity
    
    engine_w = parameters.rpm*np.pi/30

        
    if converter_w/engine_w  <0.9:
    
        ## I do not know if this parameters are fixed of they change from car to car. I believe they change.
        tp_cta= 3.4325 # es that I do not know what mean
        tp_ctb=-3.0
        tp_ctc=2.22e-3
        tp_ctd=-4.6041e-3
    
        pump_torque = tp_cta+tp_ctb*engine_w**2 +tp_ctc*engine_w*converter_w+tp_ctd*converter_w**2
        
        tt_cta = 5.7656e-3
        tt_ctb = 2.2210e-3
        tt_ctc = -5.4323e-3
        
        converter_torque = tt_cta*engine_w**2 +tt_ctb*engine_w*converter_w+tt_ctc*converter_w**2

    else:
        
        tt_cta = -6.7644e-3
        tt_ctb = 0.3107e-3
        tt_ctc = -25.2441e-3
        
        converter_torque = tt_cta*engine_w**2 +tt_ctb*engine_w*converter_w+tt_ctc*converter_w**2
        pump_torque = converter_torque


    # Calculate torque provided by the engine based on the engine RPM
    # how much torque is available thoughout the rpm range
    torque_interpolation = interp1d(parameters.car_parameters.rpm_table, parameters.car_parameters.torque_max_table)
    # Max torque available at rpm
    torque_available = torque_interpolation(parameters.rpm)
    # find the torque delivered by te engine
    engine_torque = throttle * torque_available
    
    engine_wdot = (engine_torque-pump_torque)/parameters.car_parameters.engine_inertia
    
    parameters.rpm = (engine_w + engine_wdot*parameters.time_step)*30/np.pi
    
    # Check engine RPM 
    if parameters.rpm < parameters.car_parameters.rpm_table[0]:
        parameters.rpm = parameters.car_parameters.rpm_table[0] 
    elif parameters.rpm > parameters.car_parameters.rpm_table[-1]:
        parameters.rpm = parameters.car_parameters.rpm_table[-1]
       
    
    # Gearbox up or down shifting
    if parameters.x_a.vx > parameters.car_parameters.gear_selection[int(throttle * 10)][parameters.gear]:
        parameters.gear = parameters.gear + 1
        if parameters.gear >= parameters.car_parameters.gear_ratio.size:
            parameters.gear = parameters.car_parameters.gear_ratio.size - 1
    elif parameters.x_a.vx <= 0.75 * parameters.car_parameters.gear_selection[int(throttle * 10)][parameters.gear - 1]:
        parameters.gear = parameters.gear - 1
        if parameters.gear < 1:
            parameters.gear = 1

    traction_torque = (converter_torque * parameters.car_parameters.gear_ratio[parameters.gear] * parameters.car_parameters.diff * parameters.car_parameters.diff_ni * parameters.car_parameters.transmition_ni) - ((
        ((parameters.car_parameters.axel_inertia + parameters.car_parameters.gearbox_inertia) * (parameters.gear ** 2)) + (
            parameters.car_parameters.shaft_inertia * parameters.car_parameters.gear_ratio[parameters.gear] * (parameters.car_parameters.diff ** 2)) + parameters.car_parameters.wheel_inertia) * parameters.x_a.acc_x)

    # --------------------Break Torque -------------------------
    brake_torque = brake * parameters.car_parameters.max_brake_torque

    # -------------------- Total Torque -------------------------

    if np.mean((traction_torque - brake_torque)) <= 0 and parameters.x_a.vx <= 0:
        parameters.powertrain_net_torque = np.zeros(4)
    else:
        parameters.powertrain_net_torque = (traction_torque - brake_torque) * parameters.car_parameters.brake_bias

    return parameters, logger


"""
VhclCtrl Gas  # engine rpm
VhclCtrl Brake  # Brake pedal
Vhcl PoI_Acc_1 x  # x acceleration
PT IF W < pos > rotv  # wheel angular velocity
Vehicle.GearNo  # gear
# Vehicle.Engine_rotv # eng rpm >>engine_rotv 
"""


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
    for i in tqdm(range(len(sim_data))):
        parameters.x_a.vx = sim_data[i].Vhcl_PoI_Vel_1_x
        parameters.wheel_w_vel = np.array([sim_data[i].Wheel_w_vel_FL,
                                           sim_data[i].Wheel_w_vel_RL,
                                           sim_data[i].Wheel_w_vel_FR,
                                           sim_data[i].Wheel_w_vel_RR])
        data.append(test_function(parameters, logger, throttle = sim_data[i].gas_pedal, brake = sim_data[i].brake_pedal)[0].get_data())  

    plt.figure()
    plt.title(function_name)
    plt.step([i["gear"] for i in data], "--g", label="gear_no_calcu")
    var_name = "gear_no"
    plt.step([i for j, i in enumerate(sim_data.keys()) if j % 100 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 100 == 0], label = var_name)
    plt.legend()

    plt.figure()
    plt.title(function_name)
    # var_name = "Vhcl_PoI_Vel_1_x"
    # plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 100 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 100 == 0], label = var_name)
    plt.plot([i["powertrain_net_torque"] for i in data], "--", label="powertrain_net_torque")
    plt.twinx()
    plt.plot([sim_data[i].gas_pedal for i in range(len(sim_data))], label="gas pedal")

    plt.legend()
    plt.figure()
    plt.title(function_name)
    var_name = "engine_rotv"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 100 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 100 == 0], label = var_name)
    plt.plot([(i["rpm"] / 30) * np.pi for i in data], "--", label="rpm")
    # plt.twinx()
    #plt.plot([sim_data[i].gas_pedal for i in range(len(sim_data))], label="gas pedal")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
