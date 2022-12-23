# -*- coding: utf-8 -*-
"""
Created on Thu Oct 13 10:28:58 2022

@author:  Paulo R.A. Bloemer, Maikol Drechsler
Vehicle Dynamic Model

"""

from BMW_M8 import Parameters_bmw_m8 as P
from scipy.interpolate import interp1d
from scipy.integrate import odeint
import numpy as np


class VehicleDynamics(object):
    # ^class name  #^ inherits from object

    """ This class initialize the values of a vehicular dynamic model. 
    Calculate longitudinal and lateral dynamics with desired values 
    of brake, steer and thorttle positons.
    """

    def __init__(self, min_rpm=1500., initial_vel=0., acc_x=0., position_0=[0, 0, 0], throttle=0.0, steer=0.0, brake=0.0, gear=0, alpha_engine =0):
        # ^ The first variable is the class instance in methods.
        # ^ double underscore (dunder) methods are usually special.  This one
        #  gets called immediately after a new instance is created.

        # TODO: implement method that import BMW_M8 as P
        self.parameters = P             
        self.rpm = min_rpm              
        self.speed = initial_vel        
        self.acc_x = acc_x              
        self.position = position_0
        self.steer = steer
        self.throttle = throttle
        self.brake = brake                 # Des Brake torque
        self.gear = gear                   # gear selector
        self.alpha_engine = alpha_engine   # Angular acc engine
        


    ### Pre defined vector made by myself
    # input vector

    u = np.zeros(4)
    delta = np.zeros(100)
    throttle = np.linspace(0, 1, 100)
    brake = np.zeros(100)
    G = np.zeros(100)
    
    u[0] = delta
    u[1] = throttle
    u[2] = brake
    u[3] = G
   
   
    def powertrain(self, d_engine_torque, d_brake_torque, time_step = 0.01):

        # This method has arguments.  You would call it like this:  instance.method(1, 2)
        
        self.time_step = 0.01
        
        torque_interpolation = interp1d(P.rpm_table, P.torque_max)
        torque_avaible = torque_interpolation(d_engine_torque)

        # ---------- Max Torque Condition------
        if d_engine_torque > torque_avaible:
            d_engine_torque = torque_avaible
            
        self.throttle = d_engine_torque/torque_avaible   # Set throttle position
        self.throttle_1 = self.thorttle * 10 //1 
        
        # -----------Tranmission ratio----------
        # Gear is defined in the initialization
        for i in range(len(P.gear_ratio)):
            if self.acc_x > 0.0:
                if self.speed > P.gear_selection[self.thortlle_1][self.speed]:
                     self.gear = P.gear_ratio[i+1]
                     
        ### Using an offset of upshift
            if self.acc_x <0:
                if  (self.speed * 0.8) < P.gear_selection[self.thortlle_1][self.speed]:
                    self.gear = P.gear_ratio[i]
                    ### condition works better than interpolation to selec gear
                    
        # TODO: driveshaft rpm
        self.wheel_rpm = 30 * self.speed/( P.r_dyn * np.pi)
        self.rpm = self.gear * self.wheel_rpm * self.torque_converter_ratio_inter
            
        # TODO: Engine torque to wheel, taking inercia into account
       
        self.clucht_torque = d_engine_torque - P.I_engine * self.alpha_engine # Gillespi p.26
        
        # ----------Torque Converter-----------
        # Model of torque converter -speed_ratio calculation
        self.speed_ratio_TC = [0,1]
        self.torque_converter_ratio = [2.3, 1]
        self.torque_converter_ratio_inter = interp1d(self.speed_ratio_TC, self.torque_converter_ratio)
        
        #TODO: check Angular accelerations
        self.alpha_wheel = self.acc_x / P.r_dyn
        self.alpha_d_shaft = self.gear * self.torque_converter_ratio * self.alpha_wheel
        self.alpha_engine = self.alpha_d_shaft * self.torque_converter_ratio * P.diff 
        "ratio of the final drive"
        
        #self.alpha_engine = self.alpha_wheel * P.gear * torque_converter_ratio * P.diff
        self.torque_d_shaft = (self.clucht_torque - P.I_clutch * self.alpha_egine) * self.gear ## Gillespi p.26
        self.engine2wheel_torque = (self.torque_d_shaft - P.I_d_shaft * self.alpha_d_shaft) * self.gear
    
        
        # ---------Tire slip longitudinal-----
        
        # TODO: make matricial 
        self.wheel_wspeed = self.rpm * self.gear * self.torque_converter_ratio_inter  # Wheel speed angular

        if self.acc_x > 0:
            l_slip = (P.r_dy * self.wheel_wspeed - self.speed) / (P.r_dy * self.wheel_wspeed)     # Maikol

        if self.acc_x < 0:
            l_slip = (P.r_dy * self.wheel_wspeed - self.speed) / (self.speed + 10**-16)  # Avoid division by zero
            
        # TODO: set max and min longitudinal accelaration
            "acceleration in this time stemp"
            

        # Resitances
        #self.f_aerodyn = 0.5 * P.row * P.Cd * P.Front_area * self.speed**2              # Aerodynamics Resistance
        self.rolling_resist = (P.fr * P.m * P.g * np.cos(0.) - 0.5 * P.row * P.Cl * P.area * self.speed ** 2)                              # Rolling Resistance: whit air lift
        
        self.f_resist = self.f_aerodyn  + self.rolling_resist                             # Total Resistance
        
        #--------------------Break Torque -------------------------#
        # TODO: calculate max_bake_torque and place as a imported parameter
        
        if d_brake_torque > P.max_brake_torque:
            self.brake = P.max_brake_torque
        else:
            self.brake = d_brake_torque
        self.brake_force = self.brake/ P.r_dyn
        "miltiplied by wheight transference"
        
        # TODO calculate ax positive and negative
        self.traction_force = (d_engine_torque * P.final_ratio * P.diff_ni * P.tranmisson_ni) / P.r_dyn - ((P.I_engine + P.I_transmission) * self.gear ** 2 + P.I_diff * P.final_ratio ** 2 + P.Iw) * self.acc_x/(P.r_dyn**2)
        self.Fx = self.traction_force - self.brake_force - self.f_resist
        self.acc_x = self.Fx/(P.m)   # make generic for 4 wheels 
        
        Fx_powertrain = np.zeros(4)
        # TODO: traction divided between axles
        for i in range(4):
            Fx_powertrain[i] = Fx_powertrain/4
        return Fx_powertrain
         
        
################################################################################################################################
        
    " Implementation with Bardini´s book begin here"
    
    # initial States 
    # pag 272 eq.51
    # Position and velocity of the chassis
    x = 0
    y = 0 
    z = 0 
    phi_v = 0
    theta_v = 0 
    psi_v = 0
    vx = 0
    vy = 0
    vz = 0
    wx = 0 
    wy= 0 
    wz = 0
    
    x_a = np.zeros(12)
    x_a[0] = x                          # x - position
    x_a[1] = y                          # y - position
    x_a[2] = z                          # z - position
    x_a[3] = phi_v                      # yaw angle
    x_a[4] = theta_v                    # pitch angle
    x_a[5] = psi_v                      # roll angle
    x_a[6] = vx                         # x - velocity 
    x_a[7] = vy                         # y - velocity
    x_a[8] = vz                         # z - velocity
    x_a[9] = wx                         # yaw angle rate: phi_v_dot
    x_a[10] = wy                        # pitch angle rate: theta_v_dot
    x_a[11] = wz                        # roll angle rate: psi_v_dot
    
    # pag 272 eq.52
    # Angular position/speed of each wheel
    
    x_rr = np.zeros(8)
    x_rr[0] = 0     # pho_r1
    x_rr[1] = 0     # pho_r2
    x_rr[2] = 0     # pho_r3
    x_rr[3] = 0     # pho_r4
    x_rr[4] = 0     # pho_r1_dot
    x_rr[5] = 0     # pho_r2_dot
    x_rr[6] = 0     # pho_r3_dot
    x_rr[7] = 0     #  pho_r4_dot
    
    # Vertical displacements/velocity of the wheel 
    # pag 272 eq.53
    x_rz = np.zeros(8)
    x_rz[0] = 0     # z_r1
    x_rz[1] = 0     #  z_r2
    x_rz[2] = 0     # z_r3
    x_rz[3] = 0     # z_r4
    x_rz[4] = 0     # z_r1_dot
    x_rz[5] = 0     # z_r2_dot
    x_rz[6] = 0     # z_r3_dot
    x_rz[7] = 0     # z_r4_dot
    
    # Dynamic forces on the tires
    # pag 272 eq.54

    x_rf = np.zeros(8)
    x_rf[0] = 0     # fx1
    x_rf[1] = 0     # fy1
    x_rf[2] = 0     # fx2
    x_rf[3] = 0     # fy2
    x_rf[4] = 0     # fx3
    x_rf[5] = 0     # fy3
    x_rf[6] = 0     # vfx4
    x_rf[7] = 0     # fy4
    
    # pag 272 eq.55
    x_rr = np.zeros(4)
    x[0] = x_a
    x[1] = x_rr
    x[2] = x_rz
    x[3] = x_rf
    
    #########################################
    # Global Coordinates system  -- inertial system (O_E)
    o_e = [0., 0., 0.]
        
    # Coordinates of the vehicle CoG in the inertial system O_V
    o_v = [P.cg_x, P.cg_y, P.cg_height] # [xv, yv,zv] [m]

    # Transformation matrix (rotational matrix) External_T_V -- Bardini pag 260
    E_T_V = [[np.cos(theta_v) * np.cos(psi_v)  ,  np.sin(phi_v) * np.sin(theta_v) * np.cos(psi_v) - np.cos(phi_v) * np.sin(psi_v),     np.cos(phi_v) * np.sin(theta_v) * np.cos(psi_v) + np.sin(phi_v) * np.sin(psi_v)],
             [np.cos(theta_v) * np.sin(psi_v),    np.sin(phi_v) * np.sin(theta_v) * np.sin(psi_v) + np.cos(phi_v) * np.sin(psi_v),     np.cos(phi_v) * np.sin(theta_v) * np.sin(psi_v) - np.sin(phi_v) * np.cos(psi_v)],
             [-np.sin(theta_v),                   np.sin(phi_v) * np.cos(theta_v),                                                      np.cos(phi_v) * np.cos(theta_v)]]     #  Bardini pag 260
        
    # Transposed 
    V_T_E = np.transpose(E_T_V)
    
    # For the angular velocity of the chassis we have:   Bardini Pag. 260
    Tw = [[                -np.sin(theta_v),                 0,            1],
          [ np.cos(theta_v) * np.sin(psi_v),     np.cos(psi_v),            0],
          [ np.cos(theta_v) * np.cos(psi_v),  - np.sin(psi_v),             0]]
        
        
    chassis_w_velocity = [[     x_a[9]],
                          [     x_a[10]],
                          [     x_a[11]]]               # phi_v_dot]; theta_v_dot; psi_v_dot
        
    # Coordinate representation of the absolute velocity of point v with respect to coordinate system “E”, described in coordinates of coordinate system “v”
    # bardni. Pag 260
    w_v = np.dot(Tw, chassis_w_velocity) 
    
    # Matrix E_T_R (wheel angles) is calculate at steering fuction
    
    # wheel rotation relativo to Kv(vehicle system)  Pag 261
    VTR_front_axel = [[                                                 np.cos(delta) * np.cos(theta_v),                                                -np.sin(delta) * np.cos(theta_v),                          -np.sin(theta_v)],
                      [ np.sin(psi_v) * np.sin(theta_v) * np.cos(delta) + np.cos(psi_v) * np.sin(delta),     -np.sin(psi_v) * np.sin(theta_v)*sin(delta) + np.cos(psi_v) * np.cos(delta),            np.sin(psi_v)* np.cos(theta_v)],
                      [ np.cos(psi_v) * np.sin(theta_v) * np.cos(delta) - np.sin(psi_v) * np.sin(delta),     -np.cos(psi_v) * np.sin(theta_v)*sin(delta) - np.sin(psi_v) * np.cos(delta),            np.cos(psi_v)* np.cos(theta_v)]]
    
    VTR_rear_axel = [[                  np.cos(theta_v),                 0,                          -np.sin(theta_v)],
                     [ np.sin(psi_v) * np.sin(theta_v),     np.cos(psi_v),            np.sin(psi_v)* np.cos(theta_v)],
                     [ np.cos(psi_v) * np.sin(theta_v),   - np.sin(psi_v),            np.cos(psi_v)* np.cos(theta_v)]]
        
        
    # Coordinate representation of the point i (corner) with respect to vehicle O_v (origim)  pag 265 Bardini
        
    vehicle_coordinate_A_1 = [[ P.lv],[ P.sl],[-P.sz]]
    vehicle_coordinate_A_2 = [[-P.lh],[ P.sl],[-P.sz]]
    vehicle_coordinate_A_3 = [[ P.lv],[-P.sr],[-P.sz]]
    vehicle_coordinate_A_4 = [[-P.lh],[-P.sr],[-P.sz]]
        
    # Static spring force/ or displacement at the front (bardini Pag 265)
    #TODO: Calculate static displacements
        
    #TODO: Za and zr need calculation 
    for i in range(4):
        l_stat[i] = np.array(P.m_sprung[i]) / P.f_spring_stiff
        V_F_fi = - (P.f_spring_stiff[i] * (Za[i]-Zr[i]) + P.spring_stiff[i] * (l_stat[i])) *  V_T_E * np.array([[0],[0],[1]])
        v_F_D = (P.f_dumper[i] * (Za_dot[i] - Zr_dot[i]) + P.f_dumper[i]) *  V_T_E * np.array([[0],[0],[1]])
            
    # Forces on the vehicle chassis at the pivot points Ai
    # vehicle_F_i = 
        


# input(steering wheel angle)
    def steering(des_curvature, position):
        
        # delta is steering angle 
        if (delta <= p.min and delta_dot <= 0) or (delta >= p.max and delta_dot >= 0):
            delta_dot = 0
        elif delta_dot <= p.v_min:
            delta_dot = p.v_min
        elif delta_dot >= p.v_max:
            delta_dot = p.v_max
                
        # wheel rotation relative to inertial system O_E (without spinnig)    
        wheel_angle_front =  [[np.cos(phi_v + delta),        -np.sin(phi_v + delta),      0],
                              [np.sin(phi_v + delta),         np.cos(phi_v + delta),      0],
                              [                    0,                             0,      1]] # Schramm, Dieter Hiller, ManfredBardini, Roberto Pag 261 (TR1, TR3)
            
        wheel_angle_rear =  [[np.cos(phi_v),    - np.sin( phi_v),       0],
                             [np.sin(phi_v),       np.cos(phi_v),       0],
                             [            0,                   0,       1]] # Schramm, Dieter Hiller, ManfredBardini, Roberto Pag 261 (TR2, TR4)
            
        "says rotational transformation to Kv is necessary"
            
        return wheel_angle

    def tire(wheel_angle, wheel_forces_z,time_stemp):
            
        'initial states of tire load are mandatory [Wheel torque], [Wheel_Fz], slip_x, slip_y]'
        
        # wheel states
        
            
        #  Slip calculation
        #TODO: initialize angular velocity of each wheel (relative to vehicle system)
        wheel_w_vel = [0., 0., 0., 0.]
        wheel_vx = [0., 0., 0., 0.]
        wheel_vy = [0., 0., 0., 0.]
        
        # wheel slip is calculated by wheel fixed system
        for i in range(4):
            longi_slip = (P.dyn * wheel_w_vel[i] - wheel_vx[i] / max([P.dyn * wheel_w_vel[i], wheel_vx[i]]))         # equation 11.30 Bardini (only using 1 r_dyn)
            
            alpha_slip = - np.arctan( wheel_vy[i] / max([P.dyn * wheel_w_vel[i], wheel_vx[i]]))                      # equation 11.31 Bardini
            
        #TODO: Wheelforces calculation 
        
        inital_wheel_force_z = [P.frontal_mass/2, P.rear_mass/2, P.frontal_mass/2, P.rear_mass/2] # pag 267 Bardini
        if t == 0:
            wheel_forces_z = inital_wheel_force_z
                
                
            #TODO: solve  11-25 Bardini pag 266
            
        for i in range(4):
            z2dot[i] =  (wheel_forces_z[i] - wheel_hub_force_z[i] - wheel_mass[i] * P.g) /wheel_mass[i]
                
            #TODO: 11-26   Bardini pag 266
            Wheel_angular_acc = (M_traction[i] - M_brake[i] - P.r_dyn[i] * wheel_x_force[i]) /P.I_wheel[i]
            # take an input to M_traction and M_brake
            # Wheel_x_force take slip  
            # wheel_hub force is F_Ri,z:  Wheel_vertical_acc is Z_2dot_Ri
            #return  z2dot,Wheel_angular_acc
         

         
            return ax,ay
        
        def suspension(ax,ay,heave):
          #v_F_fi = ca[i]  
          
            return vertical_loads
        
        def  chassis(vertical_loads, ax, ay, t_step):
            "Equations of motion Bardini, pag 272 ---- need initialize values"
            
            velocity = [x_vel,
                        [y_vel],
                        [z_vel]]
            
            acc_vector = [x_acc,
                         [y_acc],
                         [z_acc]]
            
            # bardini pag 260 -- use vector of torque x euler angle rate 
            
            vector_w =  [x_w,
                         [y_w],
                         [z_w]]
            
            # forces on the vehicle chassis at the pivot points Ai of the four wheel >> Bardini pag 236
            #horizontal forces pag 264
            fxh = np.zeros(4)
            fyh = np.zeros(4)
            fzh = np.zeros(4)
            
            # Forces of the suspension spring
            
            fxs = np.zeros(4)
            fys = np.zeros(4)
            fzs = np.zeros(4)
            
            # Forces of the suspension dampers
            fxd = np.zeros(4)
            fyd = np.zeros(4)
            fzd = np.zeros(4)
            
            ##################### Wheel forces##############
            
            fx_wheel = np.zeros(4)
            fz_wheel = np.zeros(4)
            fy_wheel = np.zeros(4)
         
            
        
        def road():
            return
        
            
        
        
        
        

        
        
        
        
        
        
        
        # # TODO:integers
        
        # self.acc_x = self.speed * d_engine_torque
        # x_2dot = x_2dot.append(self.acc_x)
        
        # self.velocity = self.time_step * self.acc_x + self.velocity
        # x_dot =x_dot.append(self.velocity)
        
        # self.x_position = self.time_step * self.velocity + self.x_position
    
        # x = x.append(self.x_position)

        # self.y_m = self.m * self.cg_height * self.acc_x/(P.wheel_base * 2 * P.K_sf)
        # self.pitch_angle = np.arctan(P.cg_x/self.y_m)

        # return self.acc_x, self.velocity, self.pitch_angle
        # # TODO: pitch with bar model
        
################################################################################################################################        
        
        

