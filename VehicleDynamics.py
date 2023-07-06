
# -*- coding: utf-8 -*-
"""
created on Thu Oct 13 10:28:58 2022

@author:  Paulo R.A. Bloemer, Maikol Funk Drechsler
Vehicle Dynamic Model

"""

from scipy.interpolate import interp1d
from numpy.linalg import inv
import numpy as np
import yaml
import math


class VehicleDynamics(object):
    # ^class name  #^ inherits from object

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
        self.x_a = StateVector(x=state_0[0], y=state_0[1], z=state_0[2], roll=state_0[3], picth=state_0[4], yaw=state_0[5], vx =initial_speed, vy=0., vz=0., wx=0., wy=0., wz=0.)  # State_0

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
        self.vehicle_fixed2inertial_system = np.array([[np.cos(self.x_a.picth) * np.cos(self.x_a.yaw), np.sin(self.x_a.roll) * np.sin(self.x_a.picth) * np.cos(self.x_a.yaw) - np.cos(self.x_a.roll) * np.sin(self.x_a.yaw), np.cos(self.x_a.roll) * np.sin(self.x_a.picth) * np.cos(self.x_a.yaw) + np.sin(self.x_a.roll) * np.sin(self.x_a.yaw)],
                                                       [np.cos(self.x_a.picth) * np.sin(self.x_a.yaw), np.sin(self.x_a.roll) * np.sin(self.x_a.picth) * np.sin(self.x_a.yaw) + np.cos(self.x_a.roll) * np.cos(self.x_a.yaw), np.cos(self.x_a.roll) * np.sin(self.x_a.picth) * np.sin(self.x_a.yaw) - np.sin(self.x_a.roll) * np.cos(self.x_a.yaw)],
                                                       [-np.sin(self.x_a.picth), np.sin(self.x_a.roll) * np.cos(self.x_a.picth), np.cos(self.x_a.roll) * np.cos(self.x_a.picth)]])  # Bardini pag 260
        # transformation from Ke to Kv. 
        self.transpose_vehicle_fixed2inertial_system = np.transpose(self.vehicle_fixed2inertial_system)
        self.wheel_vel_fix_coord_sys = np.zeros((4, 3), dtype=float)
        self.slip_x = np.zeros(4)
        self.slip_y = np.zeros(4)
        # wheel hub inital eq-27
        #self.wheel_hub_position = np.zeros((4,3))
        # for i in range(4): 
        #     self.wheel_hub_position[i] = self.position_chassi_force[i] + np.matmul(self.transpose_vehicle_fixed2inertial_system,np.array([0,0,self.displacement.l_stat[i]]))

    def powertrain(self, throttle, brake, rpm_table, torque_max_table, gear_ratio, diff, diff_ni, transmition_ni, gear_selection, engine_inertia, axel_inertia, gearbox_inertia, shaft_inertia, wheel_inertia, max_brake_torque, brake_bias, acc_x, wheel_w_vel, gear, vx):

        # Update Engine RPM
        rpm = gear_ratio[gear] * diff * np.mean(wheel_w_vel)   # Wheel vx to engine RPM
        if rpm < rpm_table[0]:
            rpm = rpm_table[0]
        if rpm > rpm_table[-1]:
            rpm = rpm_table[-1]

        # Calculate torque provided by the engine based on the engine RPM
        torque_interpolation = interp1d(rpm_table, torque_max_table)    # how much torque is available thoughout the rpm range 
        torque_available = torque_interpolation(rpm)                    # Max torque available at rpm
        engine_torque = throttle * torque_available                     # find the torque delivered by te engine

        # Gearbox up or down shifting
        if vx > gear_selection[int(throttle * 10)][gear]:
            gear = gear + 1
            if gear >= gear_ratio.size:
                gear = gear_ratio.size - 1
        elif vx < 0.8 * gear_selection[int(throttle * 10)][gear - 1]:
            gear = gear - 1
            if gear < 1:
                gear = 1

        # Torque converter        
        # TODO: Include the torque converter

        # self.wheel_rpm = 30 * self.x_a.vx/( self.param.r_dyn * np.pi) #RADSEC2RPM = 30/np.pi
        # replaced by self.wheel_w_vel

        # self.tcrpm = self.gear * self.wheel_w_vel[0] * self.torque_converter_ratio_inter
        # self.driveshaft_rpm = self.param.final_ratio * self.wheel_w_vel[0]
        # speed_ratio = self.driveshaft_rpm/self.rpm
        # if speed_ratio > 0.98:
        #     speed_ratio = 1.0
        # torque_convertion = self.torque_converter_ratio_inter(speed_ratio)

        # TODO: Engine torque to wheel, taking inercia into account (separated by gear)

        traction_torque = (engine_torque * gear_ratio[gear] * diff * diff_ni * transmition_ni) - ((engine_inertia + axel_inertia + gearbox_inertia) * gear ** 2 + shaft_inertia * gear_ratio[gear] * diff ** 2 + wheel_inertia) * acc_x

        # --------------------Break Torque -------------------------
        brake_torque = brake * max_brake_torque

        # -------------------- Total Torque -------------------------
        powertrain_net_torque = (traction_torque - brake_torque) * brake_bias

        """
        # TODO: Check Angular accelerations
        self.alpha_wheel = self.acc_x / self.param.r_dyn #(it s gone be used the one in the car) 
        self.alpha_d_shaft = self.gear * self.torque_converter_ratio * self.alpha_wheel
        self.alpha_engine = self.alpha_d_shaft * self.torque_converter_ratio * self.param.diff 
        ratio of the final drive
        # make matricial 
        # self.wheel_wspeed = self.rpm * self.gear * self.torque_converter_ratio_inter  # Wheel speed angular
        
        self.clucht_torque = engine_torque - self.param.engine_inertia * self.alpha_engine * torque_convertion # Gillespi p.26
        #self.alpha_engine = self.alpha_wheel * self.param.gear * self.torque_converter_ratio * self.param.diff
        self.torque_d_shaft = (self.clucht_torque - self.param.i_clutch * self.alpha_egine) * self.gear ## Gillespi p.26
        self.engine2wheel_torque = (self.torque_d_shaft - self.param.i_d_shaft * self.alpha_d_shaft) * self.gear
        """
        return [rpm, gear, powertrain_net_torque]

    def steering(self, steering):

        self.delta = steering * self.param.steering_ratio 
        delta_dot = float((self.last_delta - self.delta) / self.time_step)
        # delta is steering angle of the wheel 
        # ______________________________________________________________________________________________________________________________

        if (self.delta <= self.param.steering_min and delta_dot <= 0) or (self.delta >= self.param.steering_max and delta_dot >= 0):
            delta_dot = 0
        elif delta_dot <= self.param.steering_v_min:
            delta_dot = self.param.steering_v_min
        elif delta_dot >= self.param.steering_v_max:
            delta_dot = self.param.steering_v_max

        self.last_delta = self.delta

        # Where are you including the steering bar ratio? 
        # Matrix E_T_R (wheel angles) is calculate at steering fuction      
        # Bardini pag 261 eq. 11-6 (TR1, TR3)
        self.wheel_angle_front = [[np.cos(self.x_a.roll + self.delta), -np.sin(self.x_a.roll + self.delta), 0],
                                  [np.sin(self.x_a.roll + self.delta), np.cos(self.x_a.roll + self.delta), 0],
                                  [0, 0, 1]]

        # Eq.11-6    Schramm and Bardini Pag 261 (TR2, TR4)
        self.wheel_angle_rear = [[np.cos(self.x_a.roll), - np.sin(self.x_a.roll), 0],
                                 [np.sin(self.x_a.roll), np.cos(self.x_a.roll), 0],
                                 [0, 0, 1]] 

        self.VTR_front_axel = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
        self.VTR_rear_axel = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])

        # Wheel fixed coordinate(KR) rotation relativ to Kv(vehicle system) Bardni pag. 260 eq. 11-9
        # self.VTR_front_axel = np.array([[                                                 np.cos(self.delta) * np.cos(self.x_a.picth),                                                -np.sin(self.delta) * np.cos(self.x_a.picth),                          -np.sin(self.x_a.picth)],
        #                                [ np.sin(self.x_a.yaw) * np.sin(self.x_a.picth) * np.cos(self.delta) + np.cos(self.x_a.yaw) * np.sin(self.delta),     -np.sin(self.x_a.yaw) * np.sin(self.x_a.picth) * np.sin(self.delta) + np.cos(self.x_a.yaw) * np.cos(self.delta),            np.sin(self.x_a.yaw)* np.cos(self.x_a.picth)],
        #                                [ np.cos(self.x_a.yaw) * np.sin(self.x_a.picth) * np.cos(self.delta) - np.sin(self.x_a.yaw) * np.sin(self.delta),     -np.cos(self.x_a.yaw) * np.sin(self.x_a.picth) * np.sin(self.delta) - np.sin(self.x_a.yaw) * np.cos(self.delta),            np.cos(self.x_a.yaw)* np.cos(self.x_a.picth)]])

        # # Bardni. Pag. 260 eq. 11-10
        # self.VTR_rear_axel = np.array([[                  np.cos(self.x_a.picth),                                  0,                                   -np.sin(self.x_a.picth)],
        #                               [ np.sin(self.x_a.yaw) * np.sin(self.x_a.picth),     np.cos(self.x_a.yaw),            np.sin(self.x_a.yaw)* np.cos(self.x_a.picth)],
        #                               [ np.cos(self.x_a.yaw) * np.sin(self.x_a.picth),   - np.sin(self.x_a.yaw),            np.cos(self.x_a.yaw)* np.cos(self.x_a.picth)]])

        "says rotational transformation to Kv is necessary (VTR_front_axel)>> def vehiclefixed2inertial_system"

        # return self.wheel_angle_front, self.wheel_angle_rear

    def wheel_slip(self):  # input: fz, mi, wheel_angle,vx,vy   output:fz,slix,slip_y
        '''initial states of tire load are mandatory [Wheel torque], [Wheel_Fz], slip_x, slip_angle]
            calculate the forces..
        '''
        # wheel states
        # initialize angular velocity of each wheel (relative to vehicle system)
        for i in range(4):

            # Slip calculation - Wheel slip is calculated by wheel fixed system

            # if self.param.r_dyn[i] * self.wheel_w_vel[i] == abs(self.x_a.vx):
            #     self.slip_x[i] = .0
            # elif abs(self.param.r_dyn[i] * self.wheel_w_vel[i]) > abs(self.x_a.vx):
            #     self.slip_x[i] = 1 - abs(self.x_a.vx/(self.param.r_dyn[i] * self.wheel_w_vel[i] + 1e-52))
            # else:
            #     self.slip_x[i] = -1 + abs(self.param.r_dyn[i] * self.wheel_w_vel[i]/(self.x_a.vx + 1e-52))

            self.slip_x[i] = (((self.param.r_dyn[i] * self.wheel_w_vel[i] - self.x_a.vx) / max([abs(self.param.r_dyn[i] * self.wheel_w_vel[i] + 1e-26), abs(1e-26 + self.x_a.vx)])))         # equation 11.30 Bardini
            # REMOVED THE ABS()
            # print('self.wheel_w_vel[i]',self.wheel_w_vel[i])
            # print(self.x_a.vx,'vx')

            # Lateral slip: define wheel_vy Equacao 11_28 >> converte to tire direction
            self.slip_y[i] = - np.arctan(self.x_a.vy / max([abs(self.param.r_dyn[i] * self.wheel_w_vel[i] + 1e-16), abs(1e-16 + self.x_a.vx)]))  # equation 11.31 Bardini

            # _______________________________________________________________________________________________
            # TODO: Check first argument by displaying (velocity in wheel fixed coord) Bardini eq 11-29

            # Bardini pag.268 eq 11-33
            ' Replace with followig code to take antiroll bar in to account'
            # TODO: Make F_stv diferent than 0; make F_st input on tire fuction
            # wheel_load_z[0] = - max(self.param.cr[0]*(self.displacement.zr[0]-self.displacement.zs[0]+lr_stat[0]) + F_stv , 0) # Bardini pag.268 eq 11-33
        print("SLIP X ", self.slip_x)

    def tire_model(self):
        # fz, slip
        # Input slip, fz - fx_car, fy_car
        print('wheel_load_z', self.f_zr.wheel_load_z)
        self.x_rf.fx = self.f_zr.wheel_load_z * self.param.d * np.sin(self.param.c * np.arctan(self.param.b * self.slip_x - self.param.e * (self.param.b * self.slip_x - np.arctan(self.param.b * self.slip_x))))
        self.x_rf.fy = self.f_zr.wheel_load_z * self.param.d * np.sin(self.param.c * np.arctan(self.param.b * self.slip_y - self.param.e * (self.param.b * self.slip_y - np.arctan(self.param.b * self.slip_y))))

        for i in range(4):

            # 3x4
            self.compiled_wheel_forces[i] = np.array([self.x_rf.fx[i], self.x_rf.fy[i], self.f_zr.wheel_load_z[i]])
            # print('self.compiled_wheel_forces',self.compiled_wheel_forces,type(self.compiled_wheel_forces),self.compiled_wheel_forces.shape)

        """if i % 2 == 0:
            #TODO: check matrix operation 3x3 3x4>> define wheel_forces_transfomed matrix
            self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] =  np.matmul(self.VTR_front_axel,self.compiled_wheel_forces[i]) # Bardini pag. 267 eq.32
            
        else: 
            self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] =  np.matmul(self.VTR_rear_axel,self.compiled_wheel_forces[i])
            
        """
        # 3x4 = 3x3 @ 3x4
        # 3x1 = 3x3 @ 3x1

        self.x_rf.wheel_forces_transformed_force2vehicle_sys[:, 0] = np.matmul(self.VTR_front_axel, self.compiled_wheel_forces.T[:, 0])
        self.x_rf.wheel_forces_transformed_force2vehicle_sys[:, 1] = np.matmul(self.VTR_rear_axel, self.compiled_wheel_forces.T[:, 1])
        self.x_rf.wheel_forces_transformed_force2vehicle_sys[:, 2] = np.matmul(self.VTR_front_axel, self.compiled_wheel_forces.T[:, 2])
        self.x_rf.wheel_forces_transformed_force2vehicle_sys[:, 3] = np.matmul(self.VTR_rear_axel, self.compiled_wheel_forces.T[:, 3])

        # forces on the vehicle chassis (Ai) >> Bardini pag 236  >> horizontal forces pag 264 self.f_za.f_za
        self.strut2chassi_xyz = self.compiled_wheel_forces

        print("VTR FRONT AXLE", self.VTR_front_axel)
        print("Compiled wheel forces ", self.compiled_wheel_forces)
        print("Compiled wheel force to vehicle", self.x_rf.wheel_forces_transformed_force2vehicle_sys)

    def Wheel_angular(self):
        ''' wheel velocities are used to calcule slip, than when can futher calculate ax,ay using the forces 
        using the magic formula
        '''
        # EULER’s Equations of the Wheels: Solve  11-25 Bardini pag 266
        # Torque, fx_car, fy_car, Inertias wheel,  output wheel Wv, dot_wv
        if self.powertrain_net_torque[0] > (self.x_rf.fx[0] / self.param.r_dyn[0]):
            self.x_rr.pho_r_2dot = (self.powertrain_net_torque - self.x_rf.fx / self.param.r_dyn) / self.param.iw
        else:
            self.x_rr.pho_r_2dot = self.powertrain_net_torque / self.param.iw

        print('torque fx', self.x_rf.fx / self.param.r_dyn)
        print('powertrain net torque', self.powertrain_net_torque)

        print("pho_r_2dot     ", self.x_rr.pho_r_2dot)
        print("FX_________", self.x_rf.fx)

        # Calculate the intregral of the wheel_acc to calculate slip_x
        self.wheel_w_vel = self.wheel_w_vel + self.x_rr.pho_r_2dot * self.time_step  # rad/s
        # ___________________________________________________________________________________________________________

    def rotational_matrix(self):  # ONDE ESTÁ ATUALIZANDO OS ANGULOS?-------------------------------------------------------------
        " "
        # For the angular velocity of the chassis we have:   Bardini Pag. 261 Eq. 11.5
        rotationalmatrix = [[-np.sin(self.x_a.picth), 0, 1],
                            [np.cos(self.x_a.picth) * np.sin(self.x_a.yaw), np.cos(self.x_a.yaw), 0],
                            [np.cos(self.x_a.picth) * np.cos(self.x_a.yaw), - np.sin(self.x_a.yaw), 0]]

        # yaw_dot,picth_dot, roll_dot -- Bardini Pag. 261 Eq. 11.4
        self.angular_rates = np.array([self.x_a.wx,
                                       self.x_a.wy,
                                       self.x_a.wz])  # It is already calculated in the rotationalmatrix CALCULATE Only once please

        # Coordinate representation of the absolute velocity of point v with respect to coordinate system “E”, described in coordinates of coordinate system “v” bardni. Pag 260
        self.angular_vel_2inercial_sys_in_vehicle_coord = np.dot(rotationalmatrix, self.angular_rates)  # Bardini Pag. 261 Eq. 11.4    
        # return self.angular_vel_2inercial_sys_in_vehicle_coord, self.rotationalmatrix

    def access_z_road(x, y):
       # TODO implement topography
        z = 0.
        return z

    def road(self):
        for k in range(4):
            #     self.displacement.zs[k][2] = self.access_z_road(self.displacement.zs[k][0],self.displacement.zs[k][1])
            self.displacement.zs[k] = 0.

    def suspension(self):  # previsious forces, ax,ay 
        # Forces on the vehicle chassis at the pivot points Ai
        # Bardini pag. 265 eq. 11-21  

        # TODO adicionar zs dot VERIFICAR SE É ZA_DOT-ZS_DOT ou se é do outro jeito 
        # self.displacement.za = np.array([0,0,0,0]) # CONSIDERANDO QUE NAO HÁ TRANSFERENCIA DE CARGA
        self.f_zr.wheel_load_z = (self.param.eq_stiff * (-self.displacement.za + self.displacement.zs + self.displacement.l_stat
                                                         ) + self.param.dumper * (self.displacement.za_dot)) * np.matmul(self.transpose_vehicle_fixed2inertial_system, np.array([[0], [0], [1]]))[2]

        # NO MEU PONTO DE VISTA AQUI VOCE CALCULARIA COMO AS FORCAS QUE AGEM NO CG ATUAM NOS PO
        print("displacement.za", self.displacement.za)
        print("whell load z", self.f_zr.wheel_load_z)
        # V_F_fi - Forces on tires action on the chassis at the pivot points Ai of the four wheel >> Bardini pag 263

        # TODO: check transpose_vehicle_fixed2inertial_system operation  

    def chassis(self):  # vertical_loads, ax, ay, t_step
        "Equations of motion Bardini, pag 272 ---- need initialize values"

        'sum of  wheel forces for calculating translation of the vehicle'

        self.sum_f_wheel = np.sum(self.x_rf.wheel_forces_transformed_force2vehicle_sys, axis=1)

        print("FORCES IN THE VEHICLE ", self.x_rf.wheel_forces_transformed_force2vehicle_sys)
        print("Sum wheel ", self.sum_f_wheel)
        # Equation 11-46 >> 11-12, Pag. 273
        # TODO: check gavity diretion

        """O que faz esses valores atras da acceleracao??????????????"""
        self.x_a.acc[0] = (self.sum_f_wheel[0] - self.drag * self.x_a.vx ** 2
                           ) / self.param.m + self.x_a.wz * self.x_a.vy - self.x_a.wy * self.x_a.vz 
        self.x_a.acc[1] = (self.sum_f_wheel[1] - self.x_a.vy ** 2
                           ) / self.param.m + self.x_a.wx * self.x_a.vz - self.x_a.wz * self.x_a.vx
        self.x_a.acc[2] = (self.sum_f_wheel[2] - self.param.m * self.gravity
                           ) / self.param.m + self.x_a.wy * self.x_a.vx - self.x_a.wx * self.x_a.vy

        # vehicle velocity calculation
        self.x_a.vx = self.x_a.vx + self.x_a.acc[0] * self.time_step 
        self.x_a.vy = self.x_a.vy + self.x_a.acc[1] * self.time_step 
        self.x_a.vz = self.x_a.vz + self.x_a.acc[2] * self.time_step

        # TODO:Check rolling resistance            
        # self.rolling_resist = (self.param.fr * self.param.m * self.param.gravity * np.cos(0.) - 0.5 * self.param.row * self.param.Cl * self.param.area * self.speed ** 2)                              # Rolling Resistance with air lift

        '''Equation 11-46, vehicle velocity xyz calculation: for angular velocities and postion the last step is used
                Values calculate for positions, atittude and its derivates needs to be saved
            '''
        for i in range(4):

            self.crossproduct_r_f[i][0] = self.position_chassi_force[i][1] * self.strut2chassi_xyz[i][2] - self.position_chassi_force[i][2] * self.strut2chassi_xyz[i][1] 
            self.crossproduct_r_f[i][1] = self.position_chassi_force[i][2] * self.strut2chassi_xyz[i][0] - self.position_chassi_force[i][0] * self.strut2chassi_xyz[i][2]
            self.crossproduct_r_f[i][2] = self.position_chassi_force[i][0] * self.strut2chassi_xyz[i][1] - self.position_chassi_force[i][1] * self.strut2chassi_xyz[i][0]

        # TODO Check eq 11 - 47
        self.sum_crossproduct_r_f = np.sum(self.crossproduct_r_f, axis=0)
        # print('sum_crossproduct',self.sum_crossproduct_r_f)
        # TODO make the return of acc angular be type (3,0)

        self.x_a.acc_angular_v = np.matmul((self.sum_crossproduct_r_f - np.cross(self.angular_rates, (np.matmul(self.polar_inertia_v, self.angular_rates)))), inv(self.polar_inertia_v))

        print('sum_crossproduct', self.sum_crossproduct_r_f)
        # print('angular_rates', self.angular_rates)
        # print('polar inertia',self.polar_inertia_v)

        # print('self.x_a.acc_angular_v',self.x_a.acc_angular_v,type(self.x_a.acc_angular_v))
        # TODO: add multiplication of tranpose polar inertia to eq above>>   * self.polar_inertia_v.T)

        # Angular velocity of the chassis
        self.x_a.wx = self.x_a.wx + self.x_a.acc_angular_v[0] * self.time_step
        self.x_a.wy = self.x_a.wy + self.x_a.acc_angular_v[1] * self.time_step

        self.x_a.wz = self.x_a.wz + self.x_a.acc_angular_v[2] * self.time_step

        # Angular position
        self.x_a.roll = self.x_a.wx * self.time_step + self.x_a.roll
        self.x_a.picth = self.x_a.wy * self.time_step + self.x_a.picth
        self.x_a.yaw = self.x_a.wz * self.time_step + self.x_a.yaw

        # TODO: updated transformation to vehicle system

        # self.vehicle_fixed2inertial_system = np.array([[np.cos(self.x_a.picth) * np.cos(self.x_a.yaw), np.sin(self.x_a.roll) * np.sin(self.x_a.picth) * np.cos(self.x_a.yaw) - np.cos(self.x_a.roll) * np.sin(self.x_a.yaw),     np.cos(self.x_a.roll) * np.sin(self.x_a.picth) * np.cos(self.x_a.yaw) + np.sin(self.x_a.roll) * np.sin(self.x_a.yaw)],
        #                                       [np.cos(self.x_a.picth) * np.sin(self.x_a.yaw), np.sin(self.x_a.roll) * np.sin(self.x_a.picth) * np.sin(self.x_a.yaw) + np.cos(self.x_a.roll) * np.sin(self.x_a.yaw),     np.cos(self.x_a.roll) * np.sin(self.x_a.picth) * np.sin(self.x_a.yaw) - np.sin(self.x_a.roll) * np.cos(self.x_a.yaw)],
        #                                       [-np.sin(self.x_a.picth),                         np.sin(self.x_a.roll) * np.cos(self.x_a.picth),                                                      np.cos(self.x_a.roll) * np.cos(self.x_a.picth)]])

        # bardini pag 260 -- use vector of torque x euler angle rate

        # vehicle position calculation
        movement_vehicle = (self.x_a.vx * self.time_step, self.x_a.vy * self.time_step, self.x_a.vz * self.time_step) 

        # TODO check mat mul ordem
        [self.x_a.x, self.x_a.y, self.x_a.z] = [self.x_a.x, self.x_a.y, self.x_a.z] + np.matmul(movement_vehicle, self.vehicle_fixed2inertial_system) 
        print("x", self.x_a.x, "y", self.x_a.y, "z", self.x_a.z)

        # TODO new displacements need taking euler angles into account

        # self.displacement.za[0]= self.x_a.z #- self.param.lv* np.sin(self.x_a.picth)+ self.param.sl*np.sin(self.x_a.roll)
        # self.displacement.za[1]= self.x_a.z #+ self.param.lh* np.sin(self.x_a.picth)+ self.param.sl*np.sin(self.x_a.roll)
        # self.displacement.za[2]= self.x_a.z #- self.param.lv* np.sin(self.x_a.picth)- self.param.sr*np.sin(self.x_a.roll)
        # self.displacement.za[3]= self.x_a.z #+ self.param.lh* np.sin(self.x_a.picth)- self.param.sr*np.sin(self.x_a.roll)

        self.displacement.za[0] = - self.param.lv * np.sin(self.x_a.picth) + self.param.sl * np.sin(self.x_a.roll)
        self.displacement.za[1] = + self.param.lh * np.sin(self.x_a.picth) + self.param.sl * np.sin(self.x_a.roll)
        self.displacement.za[2] = - self.param.lv * np.sin(self.x_a.picth) - self.param.sr * np.sin(self.x_a.roll)
        self.displacement.za[3] = + self.param.lh * np.sin(self.x_a.picth) - self.param.sr * np.sin(self.x_a.roll)

        return 

    def tick(self, gas_pedal, brake, steering, time):
        #self.powertrain(gas_pedal, brake, self.param.rpm_table, self.param.torque_max) # 
        [self.rpm, self.gear, self.powertrain_net_torque] = self.powertrain(gas_pedal, brake, self.param.rpm_table, self.param.torque_max, self.param.gear_ratio, self.param.diff, self.param.diff_ni, self.param.transmition_ni, self.param.gear_selection, self.param.engine_inertia, self.param.ia, self.param.ig, self.param.id, self.param.i_wheel, self.param.max_brake_torque, self.param.b_bias, self.x_a.acc[0], self.wheel_w_vel, self.gear, self.x_a.vx)

        self.steering(steering)
        self.rotationalmatrix()
        self.wheel_slip()
        self.tire_model()
        self.Wheel_angular()
        self.road()
        self.suspension()
        self.chassis() 
        print('                      ')

        return [self.x_a.x, self.x_a.y, self.x_a.z, self.x_a.roll, self.x_a.picth, self.x_a.yaw, self.x_a.vx, self.x_a.vy, self.x_a.vz, self.x_a.wx, self.x_a.wy, self.x_a.wz, self.x_a.acc[0], self.x_a.acc[1], self.x_a.acc[2], self.gear, self.slip_x[0], self.slip_x[1], self.slip_x[2], self.slip_x[3], self.wheel_w_vel[0], self.wheel_w_vel[1]]  # self.rpm self.x_rf.fx[2]
