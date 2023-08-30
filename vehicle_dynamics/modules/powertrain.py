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


class Powertrain(object):
    """
        Powertrain is a class calculates the current Torque delivered by the engine to the wheels.
        For that is necesary select the proper gear and find the torque 
        converter multiplicator.
        gear_change() fuction uses engine angular velocity and car linear speed (vx)
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
        self.torque_drag_interpolation = interp1d(parameters.car_parameters.engine_torque_drag[:,0],parameters.car_parameters.engine_torque_drag[:,1])
        self.GRACE_PERIOD = 500
        self.current_grace_period = 0

    def gear_change(self, parameters: Initialization, logger: logging.Logger,throttle:float):
        if self.current_grace_period>0:
            self.current_grace_period -=1
            return False
        # Gearbox up or down shifting
        if parameters.x_a.vx > parameters.car_parameters.gear_selection[int(throttle * 10)][parameters.gear]:
            prev_gear = parameters.gear
            parameters.gear = parameters.gear + 1
            current_gear = parameters.gear
            if parameters.gear >= parameters.car_parameters.gear_ratio.size:
                parameters.gear = parameters.car_parameters.gear_ratio.size - 1
                return False
            self.current_grace_period = self.GRACE_PERIOD
            return prev_gear, current_gear
        elif parameters.x_a.vx <= 0.8 * parameters.car_parameters.gear_selection[int(throttle * 10)][parameters.gear - 1]:
            prev_gear = parameters.gear
            parameters.gear = parameters.gear - 1
            current_gear = parameters.gear
            if parameters.gear < 1:
                parameters.gear = 1
            self.current_grace_period = self.GRACE_PERIOD

            return prev_gear, current_gear
        return False

    def powertrain(self, parameters: Initialization, logger: logging.Logger, throttle: float, brake: float):
        # Update Converter engine_w
        # Based on page 3 from Generating Proper Integrated Dynamic Models for Vehicle Mobility (Loucas S. Louca)
        # Based on the whell velocity and engine engine_w the torque on the
        engine_w = parameters.engine_w

        is_gear_changed = self.gear_change(parameters,logger, throttle)
        if is_gear_changed:
            (prev_gear, current_gear) = is_gear_changed
            
            engine_w = engine_w * (parameters.car_parameters.gear_ratio[current_gear]/parameters.car_parameters.gear_ratio[prev_gear]) 
            # add torque convereter speed ratio

        # Calculate torque provided by the engine based on the engine engine_w
        torque_available = self.torque_interpolation(parameters.engine_w)
        engine_drag = self.torque_drag_interpolation(parameters.engine_w)

        # find the torque delivered by te engine
        engine_torque = (throttle * torque_available)+ engine_drag
        
        # Check engine engine_w
        if parameters.engine_w < parameters.car_parameters.min_engine_w:
            parameters.engine_w = parameters.car_parameters.min_engine_w
        elif parameters.engine_w > parameters.car_parameters.max_engine_w:
            parameters.engine_w = parameters.car_parameters.max_engine_w
            
        # TODO: define the best way to acess speed of output side of the torque converter
        turbine_w = parameters.final_ratio* parameters.x_a.vx
        
        method_carmaker = True # Torque converter method CM

        if method_carmaker:
            k_in_funct = interp1d(parameters.car_parameters.speed_ratio_TC, parameters.car_parameters.torque_converter_ratio)
            k_out_funct = interp1d(parameters.car_parameters.torque_converter_efficiency, np.array(np.linspace(0.,1.,len(parameters.car_parameters.torque_converter_efficiency))))
            s = turbine_w/parameters.engine_w
            k_in = k_in_funct(s)
            k_out = k_out_funct(s)


            converter_torque_in = k_in * (engine_w **2)
            torque_converter_out = k_out *( turbine_w**2)

            
        else:

            if  (turbine_w/ engine_w) < 0.9:
                converter_torque_in = 0.0024* engine_w **2 - 0.00051*engine_w*turbine_w -0.000032*turbine_w**2
                torque_converter_out = -0.0039*engine_w**2 - 0.00237* engine_w*turbine_w -0.000035*turbine_w**2
            else:
                converter_torque_in = -0.0067*engine_w**2 + 0.032*engine_w*turbine_w-0.025*turbine_w
                torque_converter_out = - converter_torque_in
            
        # Engine speed
        engine_wdot = (converter_torque_in) / parameters.car_parameters.engine_inertia
        parameters.engine_w = (engine_w + engine_wdot * parameters.time_step)

        method_a = True
        if method_a:
            # Where traction_troque calculation is coming form? (Gillespie) equation 2-7
            #  converter_torque_multiplicator
            a = torque_converter_out  * ( parameters.car_parameters.gear_ratio[parameters.gear] * parameters.car_parameters.diff * parameters.car_parameters.diff_ni * parameters.car_parameters.transmition_ni)
            c = (parameters.car_parameters.axel_inertia + parameters.car_parameters.gearbox_inertia)
            d = (parameters.car_parameters.gear_ratio[parameters.gear] ** 2)
            e = (parameters.car_parameters.shaft_inertia * parameters.car_parameters.gear_ratio[parameters.gear] * (parameters.car_parameters.diff ** 2))
            b = -(((c * d) + e + parameters.car_parameters.wheel_inertia) * parameters.x_a.acc_x)
            
            traction_torque = a+b
        else:    
            # Traction torque accordint to Bardini pag 270, eq 11-41
            # drive torque applied to the clutch
            m_clutch = engine_torque - parameters.car_parameters.engine_inertia * engine_wdot

            
            traction_torque = -(parameters.car_parameters.diff *((parameters.car_parameters.gear_ratio[parameters.gear] * m_clutch)
                                                                - (1/parameters.car_parameters.gear_ratio[parameters.gear])*
                                                                (parameters.car_parameters.engine_inertia * parameters.car_parameters.gear_ratio[parameters.gear]**2+
                                                                parameters.car_parameters.i_d_shaft) * engine_wdot))
        
        # --------------------Break Torque -------------------------
        brake_torque = brake * parameters.car_parameters.max_brake_torque

        # -------------------- Total Torque -------------------------
        if np.mean((traction_torque - brake_torque)) <= 0 and parameters.x_a.vx <= 0:
            parameters.powertrain_net_torque = np.zeros(4)
        else:
            parameters.powertrain_net_torque = (
                traction_torque - brake_torque) * parameters.car_parameters.brake_bias
        return parameters, logger

def main():
    function_name = "Powertrain"
    logger = LocalLogger(function_name).logger
    logger.info("loaded Parameters")

    parameters = Initialization("../../Audi_r8.yaml", logger=logger)

    powertrain = Powertrain(parameters)
    test_function = powertrain.powertrain

    path_to_simulation_data = "../../exampledata/acc_brake/SimulationData.pickle"
    sim_data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")
    data = []
    simulation_range= range(1050,len(sim_data))
    for i in tqdm(simulation_range):
        parameters.x_a.vx = sim_data[i].Vhcl_PoI_Vel_1_x
        data.append(test_function(parameters, logger,
                    throttle=sim_data[i].gas_pedal, brake=sim_data[i].brake_pedal)[0].get_data())

    if False:
        plt.figure()
        plt.title(function_name)
        plt.step([i["gear"] for i in data], "--g", label="gear_no_calcu")
        var_name = "gear_no"
        plt.step([i for j, i in enumerate(sim_data.keys()) if j % 100 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 100 == 0], label = var_name)
        plt.legend()
    plt.figure()
    plt.title("powertrain_net_torque")

    plt.plot([i["powertrain_net_torque"] for i in data], "g-", label="powertrain_net_torque")
    plt.legend()
    plt.grid()
    plt.twinx()
    plt.plot([sim_data[i].gas_pedal for i in simulation_range],"--", label="gas pedal")
    plt.plot([sim_data[i].brake_pedal for i in simulation_range],"--", label="brake_pedal")
    plt.legend()

    plt.figure()
    plt.title("engine w(rad/s)")
    plt.plot([(i["engine_w"]) for i in data], "r-", label= "engine_w (rad/s)")
    plt.legend(loc= 2)
    plt.grid()
    plt.twinx()
    plt.step([i["gear"] for i in data], "-g", label= "gear")
    plt.plot([sim_data[i].gas_pedal for i in simulation_range], ":",label="gas pedal")
    plt.plot([sim_data[i].brake_pedal for i in simulation_range],":", label="brake_pedal")
    plt.legend()
    
    plt.figure()
    plt.title("net torque x vx")
    plt.plot([i["powertrain_net_torque"] for i in data], "g-", label="powertrain_net_torque")
    plt.grid()
    plt.legend()
    plt.twinx()
    plt.plot([sim_data[i].Vhcl_PoI_Vel_x for i in simulation_range],"--", label="vx")
    plt.legend()
    if False:
        plt.figure()
        plt.title(function_name)
        var_name = "engine_rotv"
        plt.plot([i for j, i in enumerate(sim_data.keys()) if j % 100 == 0], [getattr(
            sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 100 == 0], label=var_name)
        plt.plot([(i["engine_w"]) for i in data], "--", label="engine_w")
        plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
