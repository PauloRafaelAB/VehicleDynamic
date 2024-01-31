from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.Initialization import Initialization
from vehicle_dynamics.utils.import_data_CM import import_data_CM

import logging
import matplotlib.scale as mscale
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


class Powertrain(object):
    """
        Powertrain is a class calculates the current Torque delivered by the engine to the wheels.
        For that is necesary select the proper gear and find the torque 
        converter multiplicator.
        gear_change() function uses engine angular velocity and car linear speed (vx)
        torque_converter() multiplication is based on input and output rotation ratio.

        Required Parameters from car_parameters:
            1.engine_w_table,
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
            4.gear[t-1]
            5. vx

        Returns: (parameters)
            1. Engine_w
            2. Gear
            3. Torque of the engine on the wheels
            4. prev_gear
            5. current_gear

    """

    def __init__(self, parameters: Initialization):
        super(Powertrain, self).__init__()
        self.torque_interpolation = interp1d(
            parameters.car_parameters.engine_w_table, parameters.car_parameters.torque_max_table)
        self.torque_drag_interpolation = interp1d(parameters.car_parameters.engine_torque_drag[:, 0], parameters.car_parameters.engine_torque_drag[:, 1])
        self.efficiency_torque_converter = interp1d([0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9],[0.05,0.201504954,0.363800128,0.492261822,0.580910496,0.62228125,0.618194304,0.590525478,0.591976672,0.716846346])
        self.MIN_GEAR_CHANGE_INTERVAL = 700
        self.CONVERTER_SYNC_TIME = 251
        self.current_sync = 0
        self.current_grace_period = 0

        self.μ_funct = interp1d(parameters.car_parameters.speed_ratio_TC, parameters.car_parameters.torque_converter_ratio)
        self.k_in_funct = interp1d(np.array(np.linspace(0.,1.,len(parameters.car_parameters.torque_converter_factor))), np.array(parameters.car_parameters.torque_converter_factor))

    def gear_change(self, parameters: Initialization, logger: logging.Logger, throttle: float):
        if self.current_grace_period > 0:
            self.current_grace_period -= 1
            return False
        
        # Gearbox up or down shifting
        if parameters.x_a.vx > parameters.car_parameters.gear_selection[int(throttle * 10)][parameters.gear]:
            prev_gear = parameters.gear
            parameters.gear = parameters.gear + 1
            current_gear = parameters.gear
            if parameters.gear >= parameters.car_parameters.gear_ratio.size:
                parameters.gear = parameters.car_parameters.gear_ratio.size - 1
                return False
            self.current_grace_period = self.MIN_GEAR_CHANGE_INTERVAL
            return prev_gear, current_gear
        
        elif parameters.x_a.vx <= 0.8 * parameters.car_parameters.gear_selection[int(throttle * 10)][parameters.gear - 1]:
            prev_gear = parameters.gear
            parameters.gear = parameters.gear - 1
            current_gear = parameters.gear
            if parameters.gear < 1:
                parameters.gear = 1
            self.current_grace_period = self.MIN_GEAR_CHANGE_INTERVAL

            return prev_gear, current_gear
        
        return False



    def gear_change_rpm(self, parameters: Initialization, logger: logging.Logger, throttle: float):
        if self.current_grace_period > 0:
            self.current_grace_period -= 1
            return False
        
        # Gearbox up or down shifting
        if parameters.engine_w > parameters.car_parameters.gear_max_rot[round(throttle * 10)][parameters.gear]:
            if parameters.gear + 1 >= parameters.car_parameters.gear_ratio.size:
                return False
            parameters.gear = parameters.gear + 1
            self.current_grace_period = self.MIN_GEAR_CHANGE_INTERVAL
            return True

        elif parameters.engine_w < parameters.car_parameters.gear_min_rot[round(throttle * 10)][parameters.gear]:
            if parameters.gear - 1 < 1:
                return False
            self.current_grace_period = self.MIN_GEAR_CHANGE_INTERVAL
            parameters.gear = parameters.gear - 1
            return True        


    def powertrain(self, parameters: Initialization, logger: logging.Logger, throttle: float, brake: float):
        # Update Converter engine_w
        # Based on page 3 from Generating Proper Integrated Dynamic Models for Vehicle Mobility (Loucas S. Louca)
        # Based on the whell velocity and engine engine_w the torque on the

        if self.gear_change_rpm(parameters, logger, throttle):
            self.current_sync = self.CONVERTER_SYNC_TIME
        

        if self.current_sync > 0:
            torque_converter_out = 0
            self.current_sync-=1
            throttle = 0.2
        
        # Check engine engine_w
        if parameters.engine_w < parameters.car_parameters.min_engine_w:
            parameters.engine_w = parameters.car_parameters.min_engine_w

            
        # Calculate torque provided by the engine based on the engine engine_w
        torque_available = self.torque_interpolation(parameters.engine_w)
        engine_drag = self.torque_drag_interpolation(parameters.engine_w)

        # find the torque delivered by te engine
        engine_torque = (throttle * torque_available)
        
        final_ratio = parameters.car_parameters.gear_ratio[parameters.gear] * parameters.car_parameters.diff
        
        turbine_w = final_ratio * np.mean(parameters.wheel_w_vel)

        torque_converter_ratio = turbine_w / parameters.engine_w
        
        MAX_ENGINE_ACC = 0.0001
        CLUTCH_EFFICIENCY = 0.9
        if torque_converter_ratio >= 0.9:
            # Clutch Mode
            if True:#(parameters.engine_w - turbine_w) / parameters.time_step > MAX_ENGINE_ACC:
                parameters.engine_w = turbine_w + (MAX_ENGINE_ACC * parameters.time_step)
                torque_converter_out = ((engine_torque * parameters.engine_w) / turbine_w ) * CLUTCH_EFFICIENCY
            else:
                parameters.engine_w = turbine_w
                torque_converter_out = engine_torque

        else:
            torque_converter_ratio = 0 if torque_converter_ratio < 0 else torque_converter_ratio
            # Torque Converter
            k_in  = self.k_in_funct (torque_converter_ratio)
            μ = self.μ_funct(torque_converter_ratio)

            torque_converter_in  = k_in  * (parameters.engine_w ** 2)
            #torque_converter_out  = µ  * (turbine_w ** 2)
            torque_converter_out = μ * torque_converter_in
            
            engine_wdot = (engine_torque - torque_converter_in - engine_drag) / parameters.car_parameters.engine_inertia
            parameters.engine_w = (parameters.engine_w + engine_wdot * parameters.time_step)


        # Where traction_troque calculation is coming form? (Gillespie) equation 2-7
        # TODO: Merge diff mi and transmission mi
        a = torque_converter_out * ( parameters.car_parameters.gear_ratio[parameters.gear] * parameters.car_parameters.diff * parameters.car_parameters.diff_ni * parameters.car_parameters.transmition_ni)
        # TODO: Merge axel and gearbox inertia
        c = (parameters.car_parameters.axel_inertia + parameters.car_parameters.gearbox_inertia)
        d = (parameters.car_parameters.gear_ratio[parameters.gear] ** 2)
        e = (parameters.car_parameters.shaft_inertia * parameters.car_parameters.gear_ratio[parameters.gear] * (parameters.car_parameters.diff ** 2))
        b = -(((c * d) + e + parameters.car_parameters.wheel_inertia) * parameters.x_a.acc_x)
        
        traction_torque = (a+b) * np.array([0.25,0.25,0.25,0.25])
        if brake>0:
            traction_torque = np.zeros(4)

        # --------------------Break Torque -------------------------
        brake_torque = brake * parameters.car_parameters.max_brake_torque * parameters.car_parameters.brake_bias

        # -------------------- Total Torque -------------------------
        if np.mean((traction_torque - brake_torque)) <= 0 and parameters.x_a.vx <= 1e-6:
            parameters.powertrain_net_torque = np.zeros(4)
        elif  parameters.x_a.vx <= 1e-6 and brake > 0 :
            # IDLE state
            parameters.powertrain_net_torque = np.zeros(4)
        else:
            parameters.powertrain_net_torque = traction_torque - brake_torque 
        return parameters, logger

def main():
    function_name = "Powertrain"
    logger = LocalLogger(function_name).logger
    logger.info("loaded Parameters")

    parameters = Initialization("../../Audi_r8.yaml", logger=logger)
    parameters.OPTIMIZATION_MODE = False

    powertrain = Powertrain(parameters)
    test_function = powertrain.powertrain

    path_to_simulation_data = "../../exampledata/aut_straight_shift_log/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []
    simulation_range = range(1, len(sim_data))
    for i in tqdm(simulation_range):
        parameters.x_a.vx = sim_data[i].Vhcl_PoI_Vel_1_x
        parameters.wheel_w_vel = np.array([sim_data[i].Wheel_w_vel_FL, sim_data[i].Wheel_w_vel_RL, sim_data[i].Wheel_w_vel_FR, sim_data[i].Wheel_w_vel_RR])
        if parameters.OPTIMIZATION_MODE:
            parameters.gear = int(sim_data[i].gear_no)
            parameters.prev_gear = int(sim_data[i - 1].gear_no)
        data.append(test_function(parameters, logger,
                    throttle=sim_data[i].gas_pedal, brake=sim_data[i].brake_pedal)[0].get_data())

    if False:
        plt.figure()
        plt.title(function_name)
        plt.step([i["gear"] for i in data], "--g", label="gear_no")
        #plt.plot([sim_data[i].Vhcl_PoI_Vel_x for i in simulation_range],"--", label="vx")
        var_name = "gear_no"
        plt.step([i for j, i in enumerate(sim_data.keys()) if j % 100 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 100 == 0], label = "CM_gear_no")
        #plt.figure()
        #plt.title("engine w(rad/s)")
        plt.xlabel("Time [miliseconds]")
        plt.ylabel("Gear")
        plt.legend(loc=5)
        plt.grid()
        plt.twinx()
        plt.plot([sim_data[i].gas_pedal for i in simulation_range], ":",label="Gas pedal")
        plt.plot([sim_data[i].brake_pedal for i in simulation_range],":", label="Brake Pedal")
        plt.legend(loc=1)
        plt.legend()

    if True:
        plt.figure()
        plt.title("Powertrain Net Torque")
        plt.plot([i["powertrain_net_torque"] for i in data], "g-", label="powertrain_net_torque")
        plt.plot([(i["engine_w"]) for i in data], "r-", label= "engine_w (rad/s)")
        plt.legend()
        plt.grid()
        plt.twinx()
        plt.plot([sim_data[i].gas_pedal for i in simulation_range],"--", label="gas pedal")
        plt.legend()

        plt.figure()
        plt.title("engine w(rad/s)")
        plt.plot([(i["engine_w"]) for i in data], "r-", label= "engine_w (rad/s)")
        plt.legend(loc=5)
        plt.grid()
        plt.twinx()

        var_name = "gear_no"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 100 == 0], 
                [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 100 == 0], "g--", label=var_name)
        plt.step([i["gear"] for i in data], "-g", label= "gear")
        plt.plot([sim_data[i].Vhcl_PoI_Vel_x for i in simulation_range],"--", label="vx")
        plt.plot([sim_data[i].gas_pedal for i in simulation_range], ":",label="gas pedal")
        #plt.plot([sim_data[i].brake_pedal for i in simulation_range],":", label="brake_pedal")
        #plt.plot([sim_data[i].brake_pedal for i in simulation_range],":", label="brake_pedal")
        plt.legend(loc=1)
    
    if True:
        plt.figure()
        plt.title(function_name)
        plt.plot([i["powertrain_net_torque"] for i in data], "g-", label="powertrain_net_torque")
        plt.grid()
        plt.legend()
        plt.twinx()
        plt.plot([sim_data[i].Vhcl_PoI_Vel_x for i in simulation_range],"--", label="vx")
        var_name = "engine_rotv"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 100 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 100 == 0], label=var_name)
        
        
        plt.plot([(i["engine_w"]) for i in data], "--", label="Engine Angular Speed")
        plt.legend()

    plt.figure()
    plt.title("powertrain_net_torque")
    
    plt.plot([i["powertrain_net_torque"] for i in data],":", label = "powertrain_net_torque")
    
    var_name="wheel_torque_FL"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"*-",label=var_name)
    var_name="wheel_torque_FR"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"*-",label=var_name)
    var_name="wheel_torque_RL"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"*-",label=var_name)
    var_name="wheel_torque_RR"
    plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"*-",label=var_name)    
    plt.twinx()
    plt.step([i["gear"] for i in data], "-g", label= "gear")
    plt.plot([sim_data[i].gas_pedal for i in simulation_range], ":",label="gas pedal")
    plt.plot([sim_data[i].brake_pedal for i in simulation_range], ":",label="gas pedal")
    plt.legend()
    
    plt.show()


if __name__ == '__main__':
    main()
