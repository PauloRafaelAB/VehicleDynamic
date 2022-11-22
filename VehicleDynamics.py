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
        #        #^ The first variable is the class instance in methods.
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
### could be des_acc >> our car expects desired torque  

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
        
        
        # TODO: self.acc will be the last t_stemp
        
        # -----------Tranmission ratio----------
        # Gear is defined in the initialization
        # TODO: include delay on gear changing 
        for i in range(len(P.gear_ratio)):
            if self.acc_x > 0.0:
                if self.speed > P.gear_selection[self.thortlle_1][self.speed]:
                     self.gear = P.gear_ratio[i+1]
                     
        ### Using an offset of upshift
        
            if self.acc_x <0:
                if  (self.speed * 0.8) < P.gear_selection[self.thortlle_1][self.speed]:
                    self.gear = P.gear_ratio[i]
                    ### condition works better than interpolation to selec gear
                    
        # TODO: engine rpm   >>  Torque converter need to be considered
        # TODO: rpm at current time step >> which one comes first?
        
        
        # TODO: driveshaft rpm
        self.wheel_rpm = 30 * self.speed/( P.r_dyn * np.pi)
        self.rpm = self.gear * self.wheel_rpm * self.torque_converter_ratio_inter
            
      
        # TODO: Engine torque to wheel: taking inercia into account
       
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
        self.f_aerodyn = 0.5 * P.row * P.Cd * P.Front_area * self.speed**2              # Aerodynamics Resistance
        self.rolling_resist = (P.fr * P.m * P.g * np.cos(0.) - 0.5 * P.row * P.Cl * P.area * self.speed ** 2)                              # Rolling Resistance: whit air lift
        
        self.f_resist = self.f_aerodyn  + self.rolling_resist                             # Total Resistance
        
        #--------------------Break Torque -------------------------#
        # TODO: calculate max_bake_torque and place as a imported parameter
        
        if d_brake_torque > P.max_brake_torque:
            self.brake = P.max_brake_torque
        else:
            self.brake = d_brake_torque
        self.brake_force = self.brake/ P.r_dyn
        "miltiplied by wheight tranference"
        
        # TODO calculate ax positive and negative
        self.traction_force = (d_engine_torque * P.final_ratio * P.diff_ni * P.tranmisson_ni) / P.r_dyn - ((P.I_engine + P.I_transmission) * self.gear ** 2 + P.I_diff * P.final_ratio ** 2 + P.Iw) * self.acc_x/(P.r_dyn**2)
        self.Fx = self.traction_force - self.brake_force - self.f_resist
        self.acc_x = self.Fx/(P.m)   # make generic for 4 wheels 
        
        
        
#############################################################################################################################
        
        
        self.wheight_front = P.m * P.g *((P.wheel_base - P.cg_x)/P.wheel_base + self.acc_x * P.cg_height/(P.g * P.wheel_base))
        self.wheight_rear = P.m * P.g *(P.cg_x/P.wheel_base + self.acc_x * P.cg_height/(P.g * P.wheel_base))
        
        self.brake_ff = self.brake_force * self.wheight_front
        self.brake_rf = self.brake_force * self.wheight_rear
        

        
        # Ntf >> Combined ratio of transmission and final drive
       
    
        # -------------wheel x force-------
       
        # taking 4 wheel into acount
        #wheel_force = engine2wheel_torque/P.r_dym
        #TODO: torque converter with retrofeed

        
            
           
############################################################################################
        # TODO:integers
        
        self.acc_x = self.speed * d_engine_torque
        x_dot_dot = x_dot_dot.append(self.acc_x)
        
        self.velocity = self.time_step * self.acc_x + self.velocity
        x_dot =x_dot.append(self.velocity)
        
        self.x_position = self.time_step * self.velocity + self.x_position
    
        x = x.append(self.x_position)

        self.y_m = self.m * self.cg_height * self.acc_x/(P.wheel_base * 2 * P.K_sf)
        self.pitch_angle = np.arctan(P.cg_x/self.y_m)

        return self.acc_x, self.velocity, self.pitch_angle
        # TODO: pitch with bar model
        
        
        
        
        
        # initial state vector for the kinematic single-track model

        
        # obtain initial states from vector
        u = [delta, des_torque, des_brake]
        x_dot = odeint(x,time_stemp, u)
        
        sx0 = init_state[0] # x1 = x-position in a global coordinate system
        sy0 = init_state[1] # x2 = y-position in a global coordinate system
        delta0 = init_state[2]  # x3 = steering angle of front wheels
        vel0 = init_state[3] # x4 = velocity in x-direction
        Psi0 = init_state[4]  # x5 = yaw angle
        beta0 = init_state[6] # x6 = yaw rate
                            # x7 = roll angle
   # x8 = roll rate
   # x9 = pitch angle
   # x10 = pitch rate
   # x11 = velocity in y-direction
   # x12 = z-position
   # x13 = velocity in z-direction

   # x14 = roll angle front
   # x15 = roll rate front
   # x16 = velocity in y-direction front
   # x17 = z-position front
   # x18 = velocity in z-direction front

   # x19 = roll angle rear
   # x20 = roll rate rear
   # x21 = velocity in y-direction rear
   # x22 = z-position rear
   # x23 = velocity in z-direction rear

   # x24 = left front wheel angular speed
   # x25 = right front wheel angular speed
   # x26 = left rear wheel angular speed
   # x27 = right rear wheel angular speed

   # x28 = delta_y_f
   # x29 = delta_y_r

   # u1 = steering angle velocity of front wheels
   # u2 = acceleration

        ### 
    
        # sprung mass states
        x0 = []  # init initial state vector
        x0.append(sx0)  # x-position in a global coordinate system
        x0.append(sy0)  # y-position in a global coordinate system
        x0.append(delta0)  # steering angle of front wheels
        x0.append(vel0)  # velocity
        x0.append(Psi0)  # yaw angle
        
    


        return x0

        def steering(des_curvature, position): # input(steering wheel angle)
            
            ##allining torque
            #heading angle
            # ------------- BEGIN CODE --------------

            # steering limit reached?
            # if (steering_angle <= p.min and steering_velocity <= 0) or (steering_angle >= p.max and steering_velocity >= 0):
            #     steering_velocity = 0
            # elif steering_velocity <= p.v_min:
            #     steering_velocity = p.v_min
            # elif steering_velocity >= p.v_max:
            #     steering_velocity = p.v_max
                
                
            wheel_angle_front =  [[np.cos(phi_v + delta) ,      -np.sin(phi_v + delta),     0],
                                  [np.sin(phi_v + delta),      np.cos(phi_v + delta),       0],
                                  [0,                        0,                         1]] # Schramm, Dieter Hiller, ManfredBardini, Roberto Pag 261 (TR1, TR3)
            
            wheel_angle_rear =  [[np.cos(phi_v),    - np.sin( phi_v),       0],
                                 [np.sin(phi_v),    np.cos(phi_v),          0],
                                 [0,            0,              1]] # Schramm, Dieter Hiller, ManfredBardini, Roberto Pag 261 (TR2, TR4)
            "says rotational transformation to Kv is necessary"
            
            
            return wheel_angle

        def tire(wheel_angle, wheel_forces):
            
            'initial states of tire load are mandatory [Wheel torque], [Wheel_Fz]'
            
            # wheel states
            
            x0 = []
            x0.append(x0[3] / (P.r_dyn))  # left front wheel angular speed
            x0.append(x0[3] / (P.r_dyn)  # right front wheel angular speed
            x0.append(x0[3] / (P.r_dyn))  # left rear wheel angular speed
            x0.append(x0[3] / (P.r_dyn))  # right rear wheel angular speed
        
            x0.append(0)  # delta_y_f
            x0.append(0)  # delta_y_r
        
            return x0

            return ax,ay
        
        def  suspension(ax,ay,heave):
            
            # auxiliary initial states
            F0_z_f = p.m_s * g * p.b / ((p.a + p.b)) + p.m_uf * g
            F0_z_r = p.m_s * g * p.a / ((p.a + p.b)) + p.m_ur * g
            
            
            # all this state vector need to be atualized not extended
            # sprung mass states
            
            x0 = []  # init initial state vector
            x0.append(sx0)  # x-position in a global coordinate system
            x0.append(sy0)  # y-position in a global coordinate system
            x0.append(delta0)  # steering angle of front wheels
            x0.append(math.cos(beta0) * vel0)  # velocity in x-direction
            x0.append(Psi0)  # yaw angle
            x0.append(dotPsi0)  # yaw rate
            x0.append(0)  # roll angle
            x0.append(0)  # roll rate
            x0.append(0)  # pitch angle
            x0.append(0)  # pitch rate
            x0.append(math.sin(beta0) * vel0)  # velocity in y-direction
            x0.append(0)  # z-position (zero height corresponds to steady state solution)
            x0.append(0)  # velocity in z-direction
        
            # unsprung mass states (front)
            x0.append(0)  # roll angle front
            x0.append(0)  # roll rate front
            x0.append(math.sin(beta0) * vel0 + p.a * dotPsi0)  # velocity in y-direction front
            x0.append((F0_z_f) / (2 * p.K_zt))  # z-position front
            x0.append(0)  # velocity in z-direction front
        
            # unsprung mass states (rear)
            x0.append(0)  # roll angle rear
            x0.append(0)  # roll rate rear
            x0.append(math.sin(beta0) * vel0 - p.b * dotPsi0)  # velocity in y-direction rear
            x0.append((F0_z_r) / (2 * p.K_zt))  # z-position rear
            x0.append(0)  # velocity in z-direction rear

            return vertical_loads
        
        def  chassis(vertical_loads, ax, ay, t_step ):
            ""
            
            
            
            return x, x_dot, pitch, yaw, roll
        
        def road():
            return
        
            
        
        
            
    
# TODO: x is incorparate with y
