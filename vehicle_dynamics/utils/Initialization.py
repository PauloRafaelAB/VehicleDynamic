from scipy.interpolate import interp1d
from numpy.linalg import inv
import numpy as np
import yaml
import math


class Initialization(object):
	"""This class initialize the values of a vehicular dynamic model. """

	def __init__(self):
		super(Initialization, self).__init__()
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

        # self.hub_distance = np.array([self.param.hub_fl, self.param.hub_rl,self.param.hub_fr,self.param.hub_rr])
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
        # self.wheel_hub_position = np.zeros((4,3))
        # for i in range(4): 
        #     self.wheel_hub_position[i] = self.position_chassi_force[i] + np.matmul(self.transpose_vehicle_fixed2inertial_system,np.array([0,0,self.displacement.l_stat[i]]))
