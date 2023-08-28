from vehicle_dynamics.structures.Displacement import Displacement
from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.Initialization import Initialization
import numpy as np
import logging


def chassis_translation(parameters: Initialization, logger: logging.Logger):
    """
    Chassis is a function that calculates the current status of the chassis

    Required Parameters from Param:
        1. m
        2. lv
        3. lh
        4. sl
        5. sr
        6. gravity
    Required Arguments:
        1. x_a
            1.01 roll
            1.02 pitch
            1.03 yaw
            1.04 wz
            1.05 wy
            1.06 wx
            1.07 vx
            1.08 vy
            1.09 vz
            1.10 acc
            1.11 acc_angular_v
        2. time_step
        3. x_rf.wheel_forces_transformed_force2vehicle_sys
        4. drag
        5. position_chassi_force
        6. strut2chassi_xyz
        7. angular_rates
        8. polar_inertia_v
        9. logger

    Returns:
        1. x_a
            1. [x_a.x, x_a.y, x_a.z] 
            1. x_a.acc
            4. x_a.vx
            5. x_a.vy
            6. x_a.vz
            7. x_a.acc_angular_v
            8. x_a.wx
            9. x_a.wy
            10.x_a.wz
            11.x_a.roll
            12.x_a.pitch
            13.x_a.yaw
        2. displacement
            1. displacement.za

    """
    "Equations of motion Bardini, pag 272 ---- need initialize values"

    'sum of  wheel forces for calculating translation of the vehicle'

    sum_f_wheel = np.sum(
        parameters.x_rf.wheel_forces_transformed_force2vehicle_sys, axis=1)

    # logger.debug(f"FORCES IN THE VEHICLE  {np.shape(parameters.x_rf.wheel_forces_transformed_force2vehicle_sys)}")
    # logger.debug(f"Sum wheel {sum_f_wheel}") # Sum wheel (3,)
    # Equation 11-46 >> 11-12, Pag. 273
    # TODO: check gravity diretion

    drag_x =(-parameters.drag * (np.sqrt(parameters.x_a.vx **2 + parameters.x_a.vy**2) * parameters.x_a.vx))
    + ((parameters.x_a.wz * parameters.x_a.vy) - (parameters.x_a.wy * parameters.x_a.vz))
    parameters.x_a.acc_x = (
        (sum_f_wheel[0] + drag_x) / parameters.car_parameters.m) + ((parameters.x_a.wx * parameters.x_a.vz) - (parameters.x_a.wz * parameters.x_a.vx))
    parameters.x_a.acc_y = ((sum_f_wheel[1]) / parameters.car_parameters.m) #+ ((parameters.x_a.wy * parameters.x_a.vx) - (parameters.x_a.wx * parameters.x_a.vy))
    parameters.x_a.acc_z = (
        (sum_f_wheel[2] - parameters.car_parameters.m * parameters.gravity) / parameters.car_parameters.m)

    parameters.x_a.vx = parameters.x_a.vx + \
        (parameters.x_a.acc_x * parameters.time_step)
    parameters.x_a.vy = parameters.x_a.vy + \
        (parameters.x_a.acc_y * parameters.time_step)
    parameters.x_a.vz = parameters.x_a.vz + \
        (parameters.x_a.acc_z * parameters.time_step)

    parameters.x_a.x = parameters.x_a.x + \
        (parameters.x_a.vx * parameters.time_step)
    parameters.x_a.y = parameters.x_a.y + \
        (parameters.x_a.vy * parameters.time_step)
    parameters.x_a.z = parameters.x_a.z + \
        (parameters.x_a.vz * parameters.time_step)
    # logger.debug(f"shape position_chassi_force {np.shape(parameters.position_chassi_force)} {np.shape(parameters.strut2chassi_xyz)}")

    # parameters.strut2chassi_xyz = parameters.strut2chassi_xyz.T  # na duvida do que isso significa, conferir a matemagica

    # TODO Check eq 11 - 47
    crossproduct_r_f = np.zeros(((3, 4)))
    return parameters, logger


def main():
    from tqdm import tqdm
    import matplotlib.pyplot as plt
    import pdb
    test_function = chassis_translation
    function_name = test_function.__name__

    logger = LocalLogger(function_name).logger
    logger.setLevel('INFO')
    parameters = Initialization("../../Audi_r8.yaml", logger=logger)
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/Lanechange_new/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []

    parameters.x_a.vx = sim_data[0].Vhcl_PoI_Vel_x
    parameters.x_a.vy = sim_data[0].Vhcl_PoI_Vel_y
    parameters.x_a.vz = sim_data[0].Vhcl_PoI_Vel_z
    parameters.x_a.acc_x = sim_data[0].Vhcl_PoI_Acc_x
    parameters.x_a.acc_y = sim_data[0].Vhcl_PoI_Acc_y
    parameters.x_a.acc_z = sim_data[0].Vhcl_PoI_Acc_z
    parameters.x_a.roll = sim_data[0].Vhcl_Roll
    parameters.x_a.pitch = 0
    pitch_trans = [sim_data[i].Vhcl_Pitch - sim_data[0].Vhcl_Pitch for i in range(len(sim_data))]
    parameters.x_a.yaw = 0
    yaw_trans = [sim_data[i].Vhcl_Yaw - sim_data[0].Vhcl_Yaw for i in range(len(sim_data))]
    cm_z_force = parameters.f_za.f_za
    parameters.x_a.acc_angular_v = [
        sim_data[0].Vhcl_RollVel, sim_data[0].Vhcl_PitchVel, sim_data[0].Vhcl_YawVel]

    sum_wheels = []
    range_calc = range(1, 15000)

    for i in tqdm(range_calc):
        forces_in_x = [sim_data[i].wheel_load_x_FL, sim_data[i].wheel_load_x_RL,
                       sim_data[i].wheel_load_x_FR, sim_data[i].wheel_load_x_RR]
        forces_in_y = [sim_data[i].wheel_load_y_FL, sim_data[i].wheel_load_y_RL,
                       sim_data[i].wheel_load_y_FR, sim_data[i].wheel_load_y_RR]
        forces_in_z = [sim_data[i].wheel_load_z_FL, sim_data[i].wheel_load_z_RL,
                       sim_data[i].wheel_load_z_FR, sim_data[i].wheel_load_z_RR]
        parameters.x_rf.wheel_forces_transformed_force2vehicle_sys = np.array(
            [forces_in_x, forces_in_y, cm_z_force])
        parameters.last_delta = sim_data[i].Steering_angle
        return_values = test_function(parameters, logger)

        data.append(return_values[0].get_data())

    logger.info("calc end")

    if True:
        plt.figure()
        plt.title(function_name)
        var_name = "wheel_load_y_FL"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], "b",label=var_name)

        var_name = "wheel_load_y_FR"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], "m",label=var_name)
        
        plt.title(function_name)
        var_name = "wheel_load_y_RL"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], "c",label=var_name)

        var_name = "wheel_load_y_RR"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], "g",label=var_name)

        plt.legend()
    plt.figure()
    plt.title(function_name)
    plt.plot(range_calc, [i["x_a.acc_x"]
                          for i in data], "--b", label="acc_x")
    plt.plot(range_calc, [i["x_a.acc_y"]
                          for i in data], "--m", label="acc_y")
    plt.plot(range_calc, [i["x_a.acc_z"] for i in data], "--c", label="acc_z")
    var_name = "Vhcl_PoI_Acc_x"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"b", label=var_name)
    var_name = "Vhcl_PoI_Acc_y"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"m", label=var_name)
    var_name = "Vhcl_PoI_Acc_z"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"c", label=var_name)

    plt.legend()

    plt.figure()
    plt.title(function_name)
    plt.plot([i["x_a.vx"] for i in data], "--b", label="x_a.vx")
    var_name = "Vhcl_PoI_Vel_x"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], "b",label=var_name)

    plt.plot([i["x_a.vy"] for i in data], "--m", label="x_a.vy")
    var_name = "Vhcl_PoI_Vel_y"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0], "m",label=var_name)

    plt.plot([i["x_a.vz"] for i in data], "--c", label="x_a.vz")
    var_name = "Vhcl_PoI_Vel_z"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(
        sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"c", label=var_name)

    plt.legend()
    
    plt.show()


if __name__ == '__main__':
    main()
