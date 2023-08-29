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

    def __init__(self, car_parameters_path, freq = 1000, state_0 = np.zeros(15), initial_gear = 1, logger = 1):
        super(Initialization, self).__init__()
        assert car_parameters_path, "Required Car Parameters"

        self.car_parameters = ImportParam(car_parameters_path)
        logger.info("Imported YAML car parameters")    

        self.time_step = 1. / freq
        self.OPTIMIZATION_MODE = False
        self.prev_gear = 1
        # Steering_angle [t-1]
        self.last_delta = 0
        # Steering_angle [t]
        self.delta = 0
        self.gravity = 9.81
        self.engine_w = self.car_parameters.min_engine_w
        self.gear = initial_gear                    # gear selector
        self.throttle = 0.0                         # thorttle position (0< throttle <1)
        self.brake = 0.0                            # Brake pedal position (0< brake <1)
        # self.alpha_engine = 0.0                     # Angular acc engine
        self.wheel_w_vel = np.zeros(4)
        self.torque_converter_ratio_inter = interp1d(self.car_parameters.speed_ratio_TC, self.car_parameters.torque_converter_ratio)
        self.drag = 0.5 * self.car_parameters.row * self.car_parameters.Cd * self.car_parameters.Front_area  # constant for air resistance

        # State initiate with the position, orientation and speed provided by the arguments, acc = 0; 
        self.x_a = StateVector(*state_0)  # State_0

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
        # Check this for iterative processes
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
        pass

    def get_data(self):
        return {"x_a.x": self.x_a.x,
                "x_a.y": self.x_a.y,
                "x_a.z": self.x_a.z,
                "x_a.roll": self.x_a.roll,
                "x_a.pitch": self.x_a.pitch,
                "x_a.yaw": self.x_a.yaw,
                "x_a.vx": self.x_a.vx,
                "x_a.vy": self.x_a.vy,
                "x_a.vz": self.x_a.vz,
                "x_a.wx": self.x_a.wx,
                "x_a.wy": self.x_a.wy,
                "x_a.wz": self.x_a.wz,
                "x_a.acc_x": self.x_a.acc_x,
                "x_a.acc_y": self.x_a.acc_y,
                "x_a.acc_z": self.x_a.acc_z,
                "gear": self.gear,
                "slip_x0": self.slip_x[0],
                "slip_x1": self.slip_x[1],
                "slip_x2": self.slip_x[2],
                "slip_x3": self.slip_x[3],
                "wheel_w_vel0": self.wheel_w_vel[0],
                "wheel_w_vel1": self.wheel_w_vel[1],
                "engine_w": self.engine_w,
                "powertrain_net_torque": self.powertrain_net_torque,
                "last_delta": self.last_delta,
                "wheel_load_z": self.f_zr.wheel_load_z,
                "wheel_w_vel": self.wheel_w_vel}
