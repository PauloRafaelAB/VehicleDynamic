from vehicle_dynamics.utils.ImportParam import ImportParam
from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.Displacement import Displacement
from vehicle_dynamics.structures.StrutForce import StrutForce
from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.AngularWheelPosition import AngularWheelPosition

from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.SimulationData import SimulationData

from scipy.interpolate import interp1d
from numpy.linalg import inv
import numpy as np
import yaml
import math


class Initialization(object):
    """This class initialize the values of a vehicular dynamic model. """

    def __init__(self, car_parameters_path, freq, initial_speed, state_0, initial_gear, logger):
        super(Initialization, self).__init__()
        assert car_parameters_path, "Required Car Parameters"

        self.car_parameters = ImportParam(car_parameters_path)
        logger.info("Imported YAML car parameters")    

        self.time_step = 1. / freq

        # Steering_angle [t-1]
        self.last_delta = 0
        # Steering_angle [t]
        self.delta = 0
        self.gravity = 9.81
        self.rpm = self.car_parameters.min_rpm
        self.gear = initial_gear                    # gear selector
        self.throttle = 0.0                         # thorttle position (0< throttle <1)
        self.brake = 0.0                            # Brake pedal position (0< brake <1)
        # self.alpha_engine = 0.0                     # Angular acc engine
        self.wheel_w_vel = np.zeros(4)
        self.torque_converter_ratio_inter = interp1d(self.car_parameters.speed_ratio_TC, self.car_parameters.torque_converter_ratio)
        self.drag = 0.5 * self.car_parameters.row * self.car_parameters.Cd * self.car_parameters.Front_area  # constant for air resistance

        # State initiate with the position, orientation and speed provided by the arguments, acc = 0; 
        self.x_a = StateVector(x=state_0[0], y=state_0[1], z=state_0[2], roll=state_0[3], pitch=state_0[4], yaw=state_0[5], vx =initial_speed, vy=0., vz=0., wx=0., wy=0., wz=0.)  # State_0

        # Wheel initiate stoped 
        self.x_rr = AngularWheelPosition(pho_r=np.zeros(4), pho_r_dot = np.zeros(4), pho_r_2dot =np.zeros(4))
        self.x_rf = TireForces(fx =np.zeros(4), fy = np.zeros(4), wheel_forces_transformed_force2vehicle_sys = np.zeros((3, 4), dtype=float))
        self.displacement = Displacement(l_stat=(self.car_parameters.m * self.car_parameters.wd * self.gravity) / self.car_parameters.eq_stiff, za = np.zeros(4), za_dot=np.zeros(4), zr_dot=np.zeros(4), zr_2dot=np.zeros(4))
        self.f_za = StrutForce(f_za = self.car_parameters.m * self.gravity * self.car_parameters.wd, f_za_dot = np.zeros(4), spring_force = (self.car_parameters.m_s * self.gravity).reshape(-1, 1), dumper_force = np.zeros((4, 1), dtype=float))
        self.f_zr = WheelHubForce(f_zr_dot=np.array([0., 0., 0., 0.]), wheel_load_z = self.car_parameters.m * self.gravity * self.car_parameters.wd)
        self.wheel_hub_velocity = np.zeros((4, 3))
        self.final_ratio = 1
        self.polar_inertia_v = np.array([[self.car_parameters.i_x_s, 0., 0.],
                                         [0., self.car_parameters.i_y_s, 0.],
                                         [0., 0., self.car_parameters.i_z]])

        # self.hub_distance = np.array([self.car_parameters.hub_fl, self.car_parameters.hub_rl,self.car_parameters.hub_fr,self.car_parameters.hub_rr])
        self.position_chassi_force = np.array([[self.car_parameters.lv, self.car_parameters.sl, -self.car_parameters.sz], [-self.car_parameters.lh, self.car_parameters.sl, -self.car_parameters.sz], [self.car_parameters.lv, -self.car_parameters.sr, -self.car_parameters.sz], [-self.car_parameters.lh, -self.car_parameters.sr, -self.car_parameters.sz]])

        self.polar_inertia_v = np.array([[self.car_parameters.i_x_s, 0., 0.],
                                         [0., self.car_parameters.i_y_s, 0.],
                                         [0., 0., self.car_parameters.i_z]])
        # Forces on the chassis
        self.strut2chassi_xyz = np.zeros((4, 3), dtype=float)
        # Forces on the wheel
        self.compiled_wheel_forces = np.zeros((4, 3), dtype= float)

        # Static displacement of the springs and tire
        # self.displacement.l_stat = self.car_parameters.m*self.car_parameters.wd *self.gravity / self.car_parameters.eq_stiff

        # Creating vector for chassis method 
        self.sum_f_wheel = np.zeros(3, dtype=float)  # Sum of wheel forces

        # self.acc = np.zeros((3), dtype=float)  # acceleration

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
        # self.wheel_hub_position = np.zeros((4,3))
        # for i in range(4): 
        #     self.wheel_hub_position[i] = self.position_chassi_force[i] + np.matmul(self.transpose_vehicle_fixed2inertial_system,np.array([0,0,self.displacement.l_stat[i]]))

        self.powertrain_net_torque = 0
