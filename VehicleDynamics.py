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

    def __init__(self, min_rpm=1500., initial_vel=0., acc_x=0., position_0=[0, 0, 0], throttle=0.0, delta=0.0, brake=0.0, gear=0, alpha_engine =0, time_step =0):
        # ^ The first variable is the class instance in methods.
        # ^ double underscore (dunder) methods are usually special.  This one
        #  gets called immediately after a new instance is created.

        # TODO: implement method that import BMW_M8 as P
        self.parameters = P             
        self.rpm = min_rpm              
        self.speed = initial_vel        
        self.acc_x = acc_x              
        self.position = position_0
        self.steer = delta
        self.throttle = throttle
        self.brake = brake                 # Des Brake torque
        self.gear = gear                   # gear selector
        self.alpha_engine = alpha_engine   # Angular acc engine
        
        # TODO: check time_step use
        self.time_step = time_step                         # seconds


    ### Pre defined vector made by myself
    # input vector

    u = np.zeros(4)
    #TODO: Witch do I use? (vector or value)
    delta = np.zeros(100)
    for i in range(len(delta)):
        delta_dot = delta[i+1]-delta[i]                 # Steering angle ratio
    delta_last = 0                                      # Used in steering angle calculation
    z2dot = np.zeros(4)                                 # Acceleration Z direction
    
    throttle = np.linspace(0, 1, 100)
    brake = np.zeros(100)
    G = np.zeros(100)
    time_step = 0.01
    
    
    u[0] = delta
    u[1] = throttle
    u[2] = brake
    u[3] = G
   
   
    def powertrain(self, d_engine_torque, d_brake_torque, time_step = 0.01):

        # This method has arguments.  You would call it like this:  instance.method(1, 2)
        
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
        
        #TODO: Check Angular accelerations
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
        self.brake_force= self.brake/ P.r_dyn
        "miltiplied by wheight transference"
        
        self.traction_force = (d_engine_torque * P.final_ratio * P.diff_ni * P.tranmisson_ni) / P.r_dyn - ((P.I_engine + P.I_transmission) * self.gear ** 2 + P.I_diff * P.final_ratio ** 2 + P.Iw) * self.acc_x/(P.r_dyn**2)
        self.Fx = self.traction_force - self.brake_force - self.f_resist
        
        
        #TODO: leave as torque on each wheel (without "/r_dyn")
        Fx_powertrain = np.zeros(4)
        # TODO: traction divided between axles
        Fx_powertrain[1] = Fx_powertrain*0.2
        Fx_powertrain[2] = Fx_powertrain*0.4
        Fx_powertrain[3] = Fx_powertrain*0.2
        Fx_powertrain[4] = Fx_powertrain*0.4
        
        return Fx_powertrain
         
        
################################################################################################################################
        
    " Implementation with Bardini??s book begin here"
    
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
    x_rr[7] = 0     # pho_r4_dot
    
    # Vertical displacements/velocity of the wheel 
    # pag 272 eq.53
    x_rz = np.zeros(8)
    x_rz[0] = 0     # z_r1
    x_rz[1] = 0     # z_r2
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
    x_rf[6] = 0     # fx4
    x_rf[7] = 0     # fy4
    
    # pag 272 eq.55
    # State vectors
    x_rr = np.zeros(4)
    x[0] = x_a
    x[1] = x_rr
    x[2] = x_rz
    x[3] = x_rf
    

    # Global Coordinates system  -- inertial system (O_E)
    o_e = [0., 0., 0.]
        
    # Coordinates of the vehicle CoG in the inertial system O_V
    o_v = [P.cg_x, P.cg_y, P.cg_height] # [xv, yv,zv] [m]

    
    # Transformation matrix (rotational matrix) External_T_V -- Bardini pag 260 eq. 11.3
    E_T_V = [[np.cos(theta_v) * np.cos(psi_v)  ,  np.sin(phi_v) * np.sin(theta_v) * np.cos(psi_v) - np.cos(phi_v) * np.sin(psi_v),     np.cos(phi_v) * np.sin(theta_v) * np.cos(psi_v) + np.sin(phi_v) * np.sin(psi_v)],
                 [np.cos(theta_v) * np.sin(psi_v),    np.sin(phi_v) * np.sin(theta_v) * np.sin(psi_v) + np.cos(phi_v) * np.sin(psi_v),     np.cos(phi_v) * np.sin(theta_v) * np.sin(psi_v) - np.sin(phi_v) * np.cos(psi_v)],
                 [-np.sin(theta_v),                   np.sin(phi_v) * np.cos(theta_v),                                                      np.cos(phi_v) * np.cos(theta_v)]]     #  Bardini pag 260
        
    # Transposed 
    V_T_E = np.transpose(E_T_V)
        
    # For the angular velocity of the chassis we have:   Bardini Pag. 261 Eq. 11.5
    Tw = [[                -np.sin(theta_v),                 0,            1],
              [ np.cos(theta_v) * np.sin(psi_v),     np.cos(psi_v),            0],
              [ np.cos(theta_v) * np.cos(psi_v),  - np.sin(psi_v),             0]]
        
    # phi_dot,theta_dot, psi_dot -- Bardini Pag. 261 Eq. 11.4
    chassis_w_velocity = [[  x_a[9]],
                          [ x_a[10]],
                          [ x_a[11]]] 
        
    # Coordinate representation of the absolute velocity of point v with respect to coordinate system ???E???, described in coordinates of coordinate system ???v???
    # bardni. Pag 260
    w_v = np.dot(Tw, chassis_w_velocity) #Bardini Pag. 261 Eq. 11.4
        
    # Matrix E_T_R (wheel angles) is calculate at steering fuction
        
    # Transposed 
    V_T_E = np.transpose(E_T_V)
    
    # For the angular velocity of the chassis we have:   Bardini Pag. 260 eq.11-5
    Tw = [[                -np.sin(theta_v),                 0,            1],
          [ np.cos(theta_v) * np.sin(psi_v),     np.cos(psi_v),            0],
          [ np.cos(theta_v) * np.cos(psi_v),  - np.sin(psi_v),             0]]
        
        
    chassis_w_velocity = [[     x_a[9]],
                          [     x_a[10]],
                          [     x_a[11]]]               # phi_v_dot]; theta_v_dot; psi_v_dot
        
    # Coordinate representation of the absolute velocity of point v with respect to coordinate system ???E???, described in coordinates of coordinate system ???v???
    # Bardni. Pag 260
    w_v = np.dot(Tw, chassis_w_velocity) 
    
    # Matrix E_T_R (wheel angles) is calculate at steering fuction
    
    # Wheel fixed coordinate(KR) rotation relativo to Kv(vehicle system) 
    # Bardni. Pag. 260 eq. 11-9
    VTR_front_axel = [[                                                 np.cos(delta) * np.cos(theta_v),                                                -np.sin(delta) * np.cos(theta_v),                          -np.sin(theta_v)],
                      [ np.sin(psi_v) * np.sin(theta_v) * np.cos(delta) + np.cos(psi_v) * np.sin(delta),     -np.sin(psi_v) * np.sin(theta_v) * np.sin(delta) + np.cos(psi_v) * np.cos(delta),            np.sin(psi_v)* np.cos(theta_v)],
                      [ np.cos(psi_v) * np.sin(theta_v) * np.cos(delta) - np.sin(psi_v) * np.sin(delta),     -np.cos(psi_v) * np.sin(theta_v) * np.sin(delta) - np.sin(psi_v) * np.cos(delta),            np.cos(psi_v)* np.cos(theta_v)]]
    # Bardni. Pag. 260 eq. 11-10
    VTR_rear_axel = [[                  np.cos(theta_v),                 0,                          -np.sin(theta_v)],
                     [ np.sin(psi_v) * np.sin(theta_v),     np.cos(psi_v),            np.sin(psi_v)* np.cos(theta_v)],
                     [ np.cos(psi_v) * np.sin(theta_v),   - np.sin(psi_v),            np.cos(psi_v)* np.cos(theta_v)]]
        
        
    # Coordinate representation of the point i (corner) with respect to vehicle O_v (origim)  pag 265 Bardini
    # Bardni. Pag. 265 eq. 11-20
    V_r_A = np.zeros(4)
    V_r_A[1] = [[ P.lv],[ P.sl],[-P.sz]]
    V_r_A[2] = [[-P.lh],[ P.sl],[-P.sz]]
    V_r_A[3] = [[ P.lv],[-P.sr],[-P.sz]]
    V_r_A[4] = [[-P.lh],[-P.sr],[-P.sz]]
        
    # Static spring force/ or displacement at the front (bardini Pag 265)
    #TODO: Calculate static displacements
        
    #TODO: Za and zr need calculation
    l_stat = np.zeros(4)
    Za = np.zeros(4)
    Zr = np.zeros(4)
    Za_dot = np.zeros(4)
    Zr_dot = np.zeros(4)
    
    inital_wheel_force_z = 9.81 * 0.5 * P.m_s # pag 267 Bardini
    F_R_z = inital_wheel_force_z

# input(steering wheel angle)
    def steering(delta, last_delta, x_a,time_step):
        delta_dot = (last_delta - delta)/time_step
        # delta is steering angle 
        if (delta <= P.steering_min and delta_dot <= 0) or (delta >= P.steering_max and delta_dot >= 0):
            delta_dot = 0
        elif delta_dot <= P.v_min:
            delta_dot = P.v_min
        elif delta_dot >= P.v_max:
            delta_dot = P.v_max
                
        # wheel rotation relative to inertial system O_E (without spinnig)
        # Pag 261 eq. 8 (TR1, TR3)
        wheel_angle_front =  [[np.cos(x_a[3] + delta),        -np.sin(x_a[3] + delta),      0],
                              [np.sin(x_a[3] + delta),         np.cos(x_a[3] + delta),      0],
                              [                    0,                             0,      1]] 
        # Pag 261 eq. 9     
        wheel_angle_rear =  [[np.cos(x_a[3]),     - np.sin(x_a[3]),       0],
                             [np.sin(x_a[3]),       np.cos(x_a[3]),       0],
                             [            0,                   0,       1]] # Schramm, Dieter Hiller, ManfredBardini, Roberto Pag 261 (TR2, TR4)
            
        "says rotational transformation to Kv is necessary (VTR_front_axel)"
            
        return wheel_angle_front, wheel_angle_rear

    def tire(wheel_angle, Zr,Zs, F_R_x,F_R_y, F_R_z, M_traction,V_r_A,V_T_E,R_T_v,fr_z_eff, z2dot,Fx_powertrain,M_brake, time_stemp):
            #F_R_z = wheel forces z
        'initial states of tire load are mandatory [Wheel torque], [Wheel_Fz], slip_x, slip_angle]'
        
        # wheel states
        #TODO: initialize angular velocity of each wheel (relative to vehicle system)
        wheel_w_vel = [0., 0., 0., 0.]
        wheel_vx = [0., 0., 0., 0.]
        wheel_vy = [0., 0., 0., 0.]
        
        #Calculating tire forces F_Ri: pag.267
        V_r_R = V_r_A + V_T_E               # eq 27
        V_v_r = 1                           # eq 28
        R_v_R = np.matmul(R_T_v,V_v_r)      # eq 29
        
        #  Slip calculation 
        # wheel slip is calculated by wheel fixed system
        
        for i in range(4):
            slip_x = (P.r_dyn[i] * wheel_w_vel[i] - wheel_vx[i] / max([P.r_dyn[i] * wheel_w_vel[i], wheel_vx[i]]))         # equation 11.30 Bardini (only using 1 r_dyn)
            
            slip_angle = - np.arctan( wheel_vy[i] / max([P.r_dyn * wheel_w_vel[i], wheel_vx[i]]))                      # equation 11.31 Bardini
        
        #TODO: Check calculus of lr_stat (mass)
        lr_stat = P.m_s / P.cR
        
        #TODO: Wheelforces calculation 
        # Bardini pag.268 eq 11-33
        # TODO:Make F_stv diferent than 0; make F_st input on tire fuction
        # F_R_z[i] = - max(P.cR[0]*(Zr[0]-Zs[0]+lr_stat[0]) + F_stv , 0)
        # F_R_z[i] = - max(P.cR[1]*(Zr[1]-Zs[1]+lr_stat[1]) + F_sth, 0)
        # F_R_z[i] = - max(P.cR[2]*(Zr[2]-Zs[2]+lr_stat[2]) - F_stv, 0)
        # F_R_z[i] = - max(P.cR[3]*(Zr[3]-Zs[3]+lr_stat[3]) - F_sth, 0)
        
        F_R_z[i] = - max(P.cR[0]*(Zr[0]-Zs[0]+lr_stat[0]), 0)
        F_R_z[i] = - max(P.cR[1]*(Zr[1]-Zs[1]+lr_stat[1]), 0)
        F_R_z[i] = - max(P.cR[2]*(Zr[2]-Zs[2]+lr_stat[2]), 0)
        F_R_z[i] = - max(P.cR[3]*(Zr[3]-Zs[3]+lr_stat[3]), 0)
        
       
        
        # wheel hub force z = fz
        # F_R_z = wheel force Z direction
        if time_stemp == 0:
            F_z = P.F_z
        for i in range(4):
            # Longitudinal forces with Fz iteration eq 11-34
            F_R_x[i] = P.mi_x[i] * np.sin(P.cx * np.arctan(P.bx *slip_x[i] /P.mi_x[i])) * fr_z_eff[i]
            F_R_y[i] = P.mi_y[i] * np.sin(P.cy * np.arctan(P.by *slip_x[i] /P.mi_y[i])) * fr_z_eff[i]
            fr_z_eff = F_R_z[i] * (1 - P.ez * (F_R_z[i]/ P.initial_F_R_z[i])**2)
            
            
            
            #NEWTON???s and EULER???s Equations of the Wheels
            #TODO: solve  11-25 Bardini pag 266
            z2dot[i] =  (F_R_z[i] - F_z[i] - P.wheel_mass[i] * P.g) /P.wheel_mass[i]
                
            #TODO: 11-26   Bardini pag 266
            
            M_traction[i] = Fx_powertrain[i] * P.r_dyn[i]  # checar multiplica????o
            Wheel_angular_acc = (M_traction[i] - M_brake[i] - P.r_dyn[i] * F_R_x[i]) /P.I_wheel[i]
        
        # wheel_hub force is F_Ri,z:  Wheel_vertical_acc is Z_2dot_Ri
        #return  z2dot,Wheel_angular_acc
        
        #TODO: Compilewheel forces xyz on one vector  and tranform  to Vechicle coordinate
        # Bardini pag. 267 eq.32
        
        #TODO:  new values calculation
         
            return F_R_x,F_R_y
        
    def suspension(F_R_X,F_R_y,vertical_loads, l_stat, Za, Zr,Za_dot,Zr_dot, V_T_E, V_F_fi,v_F_D): 
        # Za - displacement on chassis to suspention link
        # Zr - displacement on wheel hub
        # V_F_fi - Forces on tires action on the chassis 
        
        for i in range(4):
            l_stat[i] = np.array(P.m_sprung[i]) / P.spring_stiff[i]       # Calculate before?
            V_F_fi[i] = - (P.spring_stiff[i] * ((Za[i]-Zr[i]) + (l_stat[i]))) *  np.matmul(V_T_E, np.array([[0],[0],[1]])) # Bardini pag. 265 eq. 11-21 
            v_F_D[i] = - (P.dumper[i] * (Za_dot[i] - Zr_dot[i]) + P.dumper[i]) *  np.matmul(V_T_E, np.array([[0],[0],[1]])) # Bardini pag. 266 eq. 11-22
            
          # Forces on the vehicle chassis at the pivot points Ai
          # vertical_loads = vehicle_F_i
        #TODO: anti roll ball 
        #### confuso
        for i in range(4):
            vertical_loads [i][0] = F_R_X # comes from tire
            vertical_loads [i][1] = F_R_y # comes from tire
            vertical_loads [i][2] = V_F_fi[i] + v_F_D [i]
                  
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
        
        

