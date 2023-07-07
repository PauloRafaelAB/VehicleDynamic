
# -*- coding: utf-8 -*-
"""
created on Thu Oct 13 10:28:58 2022

@author:  Paulo R.A. Bloemer, Maikol Funk Drechsler
Vehicle Dynamic Model

"""

from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.Displacement import Displacement
from vehicle_dynamics.structures.StrutForce import StrutForce
from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.AngularWheelPosition import AngularWheelPosition

from vehicle_dynamics.modules.chassis import chassis
from vehicle_dynamics.modules.powertrain import powertrain
from vehicle_dynamics.modules.road import road
from vehicle_dynamics.modules.rotational_matrix import rotational_matrix
from vehicle_dynamics.modules.steering import steering
from vehicle_dynamics.modules.suspension import suspension
from vehicle_dynamics.modules.tire_model import tire_model
from vehicle_dynamics.modules.wheel_angular import wheel_angular
from vehicle_dynamics.modules.wheel_slip import wheel_slip

from scipy.interpolate import interp1d
from numpy.linalg import inv
import numpy as np
import yaml
import math


class VehicleDynamics(object):
    """ This class initialize the values of a vehicular dynamic model. 
    Calculate longitudinal and lateral dynamics with desired values 
    of brake, steer and thorttle positons.
    """

    def __init__(self, initial_speed = 0., state_0 = [0., 0., 0., 0., 0., 0.], initial_gear = 1, freq=100, param_path = ""):
        if param_path != "":
            self.param = ImportParam(param_path)  # Import all the Parameters 

        self.time_step = 1. / freq

        # Steering_angle [t-1]
        self.last_delta = 0
        # Steering_angle [t]
        self.delta = 0
        self.gravity = 9.81
        self.rpm = self.param.min_rpm
        self.gear = initial_gear                    # gear selector
        self.throttle = 0.0                         # thorttle position (0< throttle <1)
        self.brake = 0.0                            # Brake pedal position (0< brake <1)
        # self.alpha_engine = 0.0                     # Angular acc engine
        self.wheel_w_vel = np.zeros(4)
        self.torque_converter_ratio_inter = interp1d(self.param.speed_ratio_TC, self.param.torque_converter_ratio)
        self.drag = 0.5 * self.param.row * self.param.Cd * self.param.Front_area  # constant for air resistance

        # State initiate with the position, orientation and speed provided by the arguments, acc = 0; 
        self.x_a = StateVector(x=state_0[0], y=state_0[1], z=state_0[2], roll=state_0[3], pitch=state_0[4], yaw=state_0[5], vx =initial_speed, vy=0., vz=0., wx=0., wy=0., wz=0.)  # State_0

        # Wheel initiate stoped 
        self.x_rr = AngularWheelPosition(pho_r=np.zeros(4), pho_r_dot = np.zeros(4), pho_r_2dot =np.zeros(4))
        self.x_rf = TireForces(fx =np.zeros(4), fy = np.zeros(4), wheel_forces_transformed_force2vehicle_sys = np.zeros((3, 4), dtype=float))
        self.displacement = Displacement(l_stat=(self.param.m * self.param.wd * self.gravity) / self.param.eq_stiff, za = np.zeros(4), za_dot=np.zeros(4), zr_dot=np.zeros(4), zr_2dot=np.zeros(4))
        self.f_za = StrutForce(f_za = self.param.m * self.gravity * self.param.wd, f_za_dot = np.zeros(4), spring_force = (self.param.m_s * self.gravity).reshape(-1, 1), dumper_force = np.zeros((4, 1), dtype=float))
        self.f_zr = WheelHubForce(f_zr_dot=np.array([0., 0., 0., 0.]), wheel_load_z = self.param.m * self.gravity * self.param.wd)
        self.wheel_hub_velocity = np.zeros((4, 3))
        self.final_ratio = 1
        self.polar_inertia_v = np.array([[self.param.i_x_s, 0., 0.],
                                         [0., self.param.i_y_s, 0.],
                                         [0., 0., self.param.i_z]])

        #self.hub_distance = np.array([self.param.hub_fl, self.param.hub_rl,self.param.hub_fr,self.param.hub_rr])
        self.position_chassi_force = np.array([[self.param.lv, self.param.sl, -self.param.sz], [-self.param.lh, self.param.sl, -self.param.sz], [self.param.lv, -self.param.sr, -self.param.sz], [-self.param.lh, -self.param.sr, -self.param.sz]])

        self.polar_inertia_v = np.array([[self.param.i_x_s, 0., 0.],
                                         [0., self.param.i_y_s, 0.],
                                         [0., 0., self.param.i_z]])
        # Forces on the chassis
        self.strut2chassi_xyz = np.zeros((4, 3), dtype=float)
        # Forces on the wheel
        self.compiled_wheel_forces = np.zeros((4, 3), dtype= float)

        # Static displacement of the springs and tire
        # self.displacement.l_stat = self.param.m*self.param.wd *self.gravity / self.param.eq_stiff

        # Creating vector for chassis method 
        self.sum_f_wheel = np.zeros(3, dtype=float)  # Sum of wheel forces
        # self.acc = np.zeros((3),dtype=float) # acceleration
        self.crossproduct_r_f = np.zeros((4, 3), dtype=float)

        # Transformation matrix vehicle coord sys(Kv) into inertia sys(Ke) - E_T_V -- Bardini pag 260 eq. 11.3
        self.vehicle_fixed2inertial_system = np.array([[np.cos(self.x_a.pitch) * np.cos(self.x_a.yaw), np.sin(self.x_a.roll) * np.sin(self.x_a.pitch) * np.cos(self.x_a.yaw) - np.cos(self.x_a.roll) * np.sin(self.x_a.yaw), np.cos(self.x_a.roll) * np.sin(self.x_a.pitch) * np.cos(self.x_a.yaw) + np.sin(self.x_a.roll) * np.sin(self.x_a.yaw)],
                                                       [np.cos(self.x_a.pitch) * np.sin(self.x_a.yaw), np.sin(self.x_a.roll) * np.sin(self.x_a.pitch) * np.sin(self.x_a.yaw) + np.cos(self.x_a.roll) * np.cos(self.x_a.yaw), np.cos(self.x_a.roll) * np.sin(self.x_a.pitch) * np.sin(self.x_a.yaw) - np.sin(self.x_a.roll) * np.cos(self.x_a.yaw)],
                                                       [-np.sin(self.x_a.pitch), np.sin(self.x_a.roll) * np.cos(self.x_a.pitch), np.cos(self.x_a.roll) * np.cos(self.x_a.pitch)]])  # Bardini pag 260
        # transformation from Ke to Kv. 
        self.transpose_vehicle_fixed2inertial_system = np.transpose(self.vehicle_fixed2inertial_system)
        self.wheel_vel_fix_coord_sys = np.zeros((4, 3), dtype=float)
        self.slip_x = np.zeros(4)
        self.slip_y = np.zeros(4)
        # wheel hub inital eq-27
        #self.wheel_hub_position = np.zeros((4,3))
        # for i in range(4): 
        #     self.wheel_hub_position[i] = self.position_chassi_force[i] + np.matmul(self.transpose_vehicle_fixed2inertial_system,np.array([0,0,self.displacement.l_stat[i]]))

    def tick(self, gas_pedal, brake, steering, time):
        #self.powertrain(gas_pedal, brake, self.param.rpm_table, self.param.torque_max) # 
        [self.rpm, self.gear, self.powertrain_net_torque] = self.powertrain(gas_pedal, brake, self.param.rpm_table, self.param.torque_max, self.param.gear_ratio, self.param.diff, self.param.diff_ni, self.param.transmition_ni, self.param.gear_selection, self.param.engine_inertia, self.param.ia, self.param.ig, self.param.id, self.param.i_wheel, self.param.max_brake_torque, self.param.b_bias, self.x_a.acc[0], self.wheel_w_vel, self.gear, self.x_a.vx)
        [self.displacement.za, self.gear, self.powertrain_net_torque]
        self.steering(steering)
        self.rotationalmatrix()
        self.wheel_slip()
        self.tire_model()
        self.Wheel_angular()
        self.road()
        self.suspension()
        self.chassis() 

        return [self.x_a.x, self.x_a.y, self.x_a.z, self.x_a.roll, self.x_a.pitch, self.x_a.yaw, self.x_a.vx, self.x_a.vy, self.x_a.vz, self.x_a.wx, self.x_a.wy, self.x_a.wz, self.x_a.acc[0], self.x_a.acc[1], self.x_a.acc[2], self.gear, self.slip_x[0], self.slip_x[1], self.slip_x[2], self.slip_x[3], self.wheel_w_vel[0], self.wheel_w_vel[1]]  # self.rpm self.x_rf.fx[2]
