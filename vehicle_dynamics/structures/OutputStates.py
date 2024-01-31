from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.Displacement import Displacement
from vehicle_dynamics.structures.StrutForce import StrutForce
from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.AngularWheelPosition import AngularWheelPosition
from collections import namedtuple
from copy import deepcopy,copy

OutputState = namedtuple('OutputState', 'brake car_parameters compiled_wheel_forces crossproduct_r_f delta displacement drag engine_w f_za f_zr final_ratio gear get_data gravity last_delta polar_inertia_v position_chassi_force powertrain_net_torque prev_gear slip_x slip_y strut2chassi_xyz sum_f_wheel throttle time_step torque_converter_ratio_inter transpose_vehicle_fixed2inertial_system vehicle_fixed2inertial_system wheel_hub_velocity wheel_vel_fix_coord_sys wheel_w_vel x_a x_rf x_rr')


class OutputStates(object):
    """OutputStates Class
        makes a deep copy of the simulation parameters, and stores them in a list, any parameter is available using the .attribute[] operator"""

    def __init__(self):
        super(OutputStates, self).__init__()
        self.brake = []
        self.car_parameters = []
        self.compiled_wheel_forces = []
        self.crossproduct_r_f = []
        self.delta = []
        self.displacement = []
        self.drag = []
        self.engine_w = []
        self.f_za = []
        self.f_zr = []
        self.final_ratio = []
        self.gear = []
        self.get_data = []
        self.gravity = []
        self.last_delta = []
        self.polar_inertia_v = []
        self.position_chassi_force = []
        self.powertrain_net_torque = []
        self.prev_gear = []
        self.slip_x = []
        self.slip_y = []
        self.strut2chassi_xyz = []
        self.sum_f_wheel = []
        self.throttle = []
        self.time_step = []
        self.torque_converter_ratio_inter = []
        self.transpose_vehicle_fixed2inertial_system = []
        self.vehicle_fixed2inertial_system = []
        self.wheel_hub_velocity = []
        self.wheel_vel_fix_coord_sys = []
        self.wheel_w_vel = []
        self.x_a = []
        self.x_rf = []
        self.x_rr = []

    def set_states(self, parameters):
        self.brake.append(copy(parameters.brake))
        self.car_parameters.append(copy(parameters.car_parameters))
        self.compiled_wheel_forces.append(copy(parameters.compiled_wheel_forces))
        self.crossproduct_r_f.append(copy(parameters.crossproduct_r_f))
        self.delta.append(copy(parameters.delta))
        self.displacement.append(copy(parameters.displacement))
        self.drag.append(copy(parameters.drag))
        self.engine_w.append(copy(parameters.engine_w))
        self.f_za.append(copy(parameters.f_za))
        self.f_zr.append(copy(parameters.f_zr))
        self.final_ratio.append(copy(parameters.final_ratio))
        self.gear.append(copy(parameters.gear))
        self.get_data.append(copy(parameters.get_data))
        self.gravity.append(copy(parameters.gravity))
        self.last_delta.append(copy(parameters.last_delta))
        self.polar_inertia_v.append(copy(parameters.polar_inertia_v))
        self.position_chassi_force.append(copy(parameters.position_chassi_force))
        self.powertrain_net_torque.append(copy(parameters.powertrain_net_torque))
        self.prev_gear.append(copy(parameters.prev_gear))
        self.slip_x.append(copy(parameters.slip_x))
        self.slip_y.append(copy(parameters.slip_y))
        self.strut2chassi_xyz.append(copy(parameters.strut2chassi_xyz))
        self.sum_f_wheel.append(copy(parameters.sum_f_wheel))
        self.throttle.append(copy(parameters.throttle))
        self.time_step.append(copy(parameters.time_step))
        self.torque_converter_ratio_inter.append(copy(parameters.torque_converter_ratio_inter))
        self.transpose_vehicle_fixed2inertial_system.append(copy(parameters.transpose_vehicle_fixed2inertial_system))
        self.vehicle_fixed2inertial_system.append(copy(parameters.vehicle_fixed2inertial_system))
        self.wheel_hub_velocity.append(copy(parameters.wheel_hub_velocity))
        self.wheel_vel_fix_coord_sys.append(copy(parameters.wheel_vel_fix_coord_sys))
        self.wheel_w_vel.append(copy(parameters.wheel_w_vel))
        self.x_a.append(deepcopy(parameters.x_a))
        self.x_rf.append(deepcopy(parameters.x_rf))
        self.x_rr.append(deepcopy(parameters.x_rr))

    def __getitem__(self, items):
        return OutputState(self.brake,
                           self.car_parameters,
                           self.compiled_wheel_forces,
                           self.crossproduct_r_f,
                           self.delta,
                           self.displacement,
                           self.drag,
                           self.engine_w,
                           self.f_za,
                           self.f_zr,
                           self.final_ratio,
                           self.gear,
                           self.get_data,
                           self.gravity,
                           self.last_delta,
                           self.polar_inertia_v,
                           self.position_chassi_force,
                           self.powertrain_net_torque,
                           self.prev_gear,
                           self.slip_x,
                           self.slip_y,
                           self.strut2chassi_xyz,
                           self.sum_f_wheel,
                           self.throttle,
                           self.time_step,
                           self.torque_converter_ratio_inter,
                           self.transpose_vehicle_fixed2inertial_system,
                           self.vehicle_fixed2inertial_system,
                           self.wheel_hub_velocity,
                           self.wheel_vel_fix_coord_sys,
                           self.wheel_w_vel,
                           self.x_a,
                           self.x_rf,
                           self.x_rr)
