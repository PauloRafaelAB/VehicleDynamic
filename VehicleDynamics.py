# -*- coding: utf-8 -*-
"""
created on Thu Oct 13 10:28:58 2022

@author:  Paulo R.A. Bloemer, Maikol Funk Drechsler
Vehicle Dynamic Model

"""

from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
import yaml

class VehicleDynamics(object):
    # ^class name  #^ inherits from object

    """ This class initialize the values of a vehicular dynamic model. 
    Calculate longitudinal and lateral dynamics with desired values 
    of brake, steer and thorttle positons.
    """

    def __init__(self, state_0 = [0., 0., 0., 0., 0., 0.,0., 0., 0., 0., 0., 0.], initial_gear = 0, freq=100, param_path = ""):
    
        if param_path != "":
            self.param = ImportParam(param_path) # Import all the Parameters 
            
        #self.position = state_0
        self.time_step = 1/freq
        self.gravity = 9.81
        self.rpm = self.param.min_rpm
        self.gear = initial_gear                    # gear selector
        self.throttle = 0.0                         # thorttle position (0< throttle <1)
        self.brake = 0.0                            # Brake pedal position (0< brake <1)
        self.alpha_engine = 0.0                     # Angular acc engine
        self.torque_converter_ratio_inter = interp1d(self.param.speed_ratio_TC, self.param.torque_converter_ratio)
        self.drag = 0.5*self.param.row * self.param.Cd * self.param.Front_area  # constant for air resistance
        
        # Global Coordinates system  -- inertial system (O_E)
        o_e = [0., 0., 0.]
        # Coordinates of the vehicle CoG in the inertial system O_V
        o_v = [self.param.cg_x, self.param.cg_y, self.param.cg_height] # [xv, yv,zv] [m]
        
        # State
    
        self.x_a = StateVector(x=state_0[0],y=state_0[0],z=state_0[0], phi_v=state_0[0], theta_v=state_0[0], psi_v=state_0[0], vx=state_0[0], vy=state_0[0],vz=state_0[0], wx=state_0[0], wy=state_0[0], wz=state_0[0]) # State_0
        self.x_rr = AngularWheelPosition(pho_r=[0,0,0,0], pho_r_dot = [0,0,0,0])
        # self.x_rz = WheelsPositionZ(z_r=[0,0,0,0], z_r_dot=[0,0,0,0])
        self.x_rf = TireForces(fx1=0, fy1=0,fx2=0, fy2=0, fx3=0,fy3=0, fx4=0, fy4=0)
        self.displacement = Displacement(l_stat=[0,0,0,0], za=[0,0,0,0], zr =[0,0,0,0], za_dot=[0,0,0,0], zr_dot=[0,0,0,0])
        self.f_za = StrutForce(f_za = [0,0,0,0], f_za_dot = [0,0,0,0])
        self.f_zr = WheelHubForce(f_zr=[0,0,0,0],f_zr_dot=[0,0,0,0])
        
        self.polar_inertia_v = np.array([self.param.i_x_s, 0, 0],
                                        [0, self.param.i_y_s, 0],
                                        [0, 0, self.param.i_z])
        
        # Suspension to chassis conection (position Ai) Bardini eq. 11-20
        self.position_chassi_force[0] = np.array([self.param.lv],[self.param.sl],[-self.param.sz])                       
        self.position_chassi_force[1] = np.array([-self.param.lh],[self.param.sl],[-self.param.sz])
        self.position_chassi_force[2] = np.array([self.param.lv],[-self.param.sr],[-self.param.sz])
        self.position_chassi_force[3] = np.array([-self.param.lh],[-self.param.sr],[-self.param.sz])
        
        # Static displacement of the springs
        self.displacement.l_stat = np.array(self.param.m_sprung) / self.param.spring_stiff
        
        #TODO: make zr depend on zs
        self.displacement.zr = np.array(self.param.m/self.param.cr)
        # TODO: check m is adressed for each wheel 
        self.lr_stat = self.param.m/self.param.cr                       # Tire static deformation
        
        
    def debug_powertrain(self, gas_pedal, brake, steering):
        self.powertrain(gas_pedal, brake, self.param.rpm_table, self.param.torque_max)
        return self.powertrain_net_torque, self.powertrain_force
        
        
    def powertrain(self, d_brake_torque, rpm_table, torque_max_table):
        
        torque_interpolation = interp1d(rpm_table, torque_max_table)
        torque_available = torque_interpolation(self.rpm)
        d_engine_torque = self.throttle * torque_available   # Set throttle position
        
        # -----------Tranmission ratio----------
        # Comparing engine rpm and torque converter rpm to define the available torque. 
        for i in range(len(self.param.gear_ratio)): # checking number of gears
            if self.acc_x >= 0.0:  
                if self.x_a.vx > self.param.gear_selection[round(self.throttle*10)][self.x_a.vx]:
                     self.gear = self.param.gear_ratio[i+1]
                     
        # Using an offset of upshift
            if self.acc_x <0:
                if  (self.x_a.vx * 0.8) < self.param.gear_selection[round(self.throttle*10)][self.x_a.vx]:
                    self.gear = self.param.gear_ratio[i]
            else:
                self.gear = self.param.gear_ratio[0]    # neutral    
        
        #self.wheel_rpm = 30 * self.x_a.vx/( self.param.r_dyn * np.pi) #RADSEC2RPM = 30/np.pi
        self.tcrpm = self.gear * self.wheel_rpm * self.torque_converter_ratio_inter
        self.driveshaft_rpm = self.param.final_ratio * self.wheel_rpm
        speed_ratio = self.driveshaft_rpm/self.rpm 
        if speed_ratio > 0.98:
            speed_ratio = 1.0
        
        
        # TODO: Engine torque to wheel, taking inercia into account (separated by gear)
        # torque_convertion = self.torque_converter_ratio_inter(speed_ratio)
        
        self.traction_torque = (d_engine_torque * self.param.final_ratio * self.param.diff_ni * 
                                self.param.transmisson_ni) - ((self.param.engine_inertia + self.param.I_transmission
                                                               ) * self.gear ** 2 + self.param.I_diff * self.param.final_ratio ** 2 + 
                                                               self.param.Iw) * self.acc_x
        
        #--------------------Break Torque -------------------------
        # using brake pedal as input, the brake torque can be calculated by 
        self.brake_torque = self.brake * self.param.max_brake_torque
        
        #-------------------- Net Torque -------------------------
        self.powertrain_net_torque_torque = self.traction_torque - self.brake_torque
        
        # function traction divided between axles
        self.powertrain_net_torque[0] = self.powertrain_net_torque*0.5* self.para.b_bias  # CREATE AS PARAM
        self.powertrain_net_torque[1] = self.powertrain_net_torque*0.5* (1-self.para.b_bias)
        self.powertrain_net_torque[2] = self.powertrain_net_torque*0.5* self.para.b_bias
        self.powertrain_net_torque[3] = self.powertrain_net_torque*0.5* (1-self.para.b_bias)
        self.powetrain_force = self.powertrain_net_torque/self.r_dyn
        
        """
        # TODO: Check Angular accelerations
        self.alpha_wheel = self.acc_x / self.param.r_dyn #(it s gone be used the one in the car) 
        self.alpha_d_shaft = self.gear * self.torque_converter_ratio * self.alpha_wheel
        self.alpha_engine = self.alpha_d_shaft * self.torque_converter_ratio * self.param.diff 
        ratio of the final drive
        # make matricial 
        # self.wheel_wspeed = self.rpm * self.gear * self.torque_converter_ratio_inter  # Wheel speed angular
        
        self.clucht_torque = d_engine_torque - self.param.engine_inertia * self.alpha_engine * torque_convertion # Gillespi p.26
        
        #self.alpha_engine = self.alpha_wheel * self.param.gear * self.torque_converter_ratio * self.param.diff
        self.torque_d_shaft = (self.clucht_torque - self.param.i_clutch * self.alpha_egine) * self.gear ## Gillespi p.26
        self.engine2wheel_torque = (self.torque_d_shaft - self.param.i_d_shaft * self.alpha_d_shaft) * self.gear
        """
        
        return self.powertrain_net_torque, self.powertrain_force
    
   
    def steering(self):
        delta_dot = (self.last_delta - self.delta)/self.time_step
        # delta is steering angle 
        if (self.delta <= self.param.steering_min and delta_dot <= 0) or (self.delta >= self.param.steering_max and delta_dot >= 0):
            delta_dot = 0
        elif delta_dot <= self.param.v_min:
            delta_dot = self.param.v_min
        elif delta_dot >= self.param.v_max:
            delta_dot = self.param.v_max
        self.last_delta = self.delta
        
        # Matrix E_T_R (wheel angles) is calculate at steering fuction      
        # Bardini pag 261 eq. 11-6 (TR1, TR3)
        self.wheel_angle_front =  [[np.cos(self.x_a.phi_v + self.delta),        -np.sin(self.x_a.phi_v + self.delta),      0],
                                   [np.sin(self.x_a.phi_v + self.delta),         np.cos(self.x_a.phi_v + self.delta),      0],
                                   [                    0,                             0,      1]] 
        # Eq.11-6    Schramm and Bardini Pag 261 (TR2, TR4)
        self.wheel_angle_rear =  [[np.cos(self.x_a.phi_v),     - np.sin(self.x_a.phi_v),       0],
                                  [np.sin(self.x_a.phi_v),       np.cos(self.x_a.phi_v),       0],
                                  [            0,                                     0,       1]] 
            
        "says rotational transformation to Kv is necessary (VTR_front_axel)>> def vehiclefixed2inertial_system"
            
        return self.wheel_angle_front, self.wheel_angle_rear, self.last_delta
    
    
    def newton_euler_wheel(self):
        ''' wheel velocities are used to calcule slip, than when can futher calculate ax,ay using the forces 
        using the magic formula
        '''
        # NEWTON’s and EULER’s Equations of the Wheels: Solve  11-25 Bardini pag 266
        for i in range(4):
            # 11-26 Bardini pag 266. Where fx(tire_road force) is determined by magic formula
            self.z2dot[i] =  (self.wheel_load_z[i] - self.f_za.f_za[i] - self.param.wheel_mass[i] * self.param.gravity) /self.param.wheel_mass[i]
            self.pho2dot[i] =  (self.powertrain_net_torque[i] - self.param.r_dyn[i] * self.fx[i]) /self.param.wheel_inertia[i]
            
            # Calculate the intregral of the wheel_acc to calculate slip_x
            self.wheel_w_vel[i] = self.wheel_w_vel[i] + self.pho2dot[i] * self.time_step
            # Calculate the wheel hub displament
            self.displacement.zr_dot = self.displacement.zr_dot + self.z2dot[i] * self.time_step
            self.displacement.zr = self.displacement.zr +self.displacement.z_dot * self.time_step
    
    
    def rotationalmatrix(self):
        " "
        # For the angular velocity of the chassis we have:   Bardini Pag. 261 Eq. 11.5
        rotationalmatrix = [[                          -np.sin(self.x_a.theta_v),                        0,             1],
                            [ np.cos(self.x_a.theta_v) * np.sin(self.x_a.psi_v),    np.cos(self.x_a.psi_v),             0],
                            [ np.cos(self.x_a.theta_v) * np.cos(self.x_a.psi_v),  - np.sin(self.x_a.psi_v),             0]]
        
        # phi_dot,theta_dot, psi_dot -- Bardini Pag. 261 Eq. 11.4
        self.chassis_w_velocity = [[ self.x_a.wx],
                                   [ self.x_a.wy],
                                   [ self.x_a.wz]]
                              
        # Coordinate representation of the absolute velocity of point v with respect to coordinate system “E”, described in coordinates of coordinate system “v” bardni. Pag 260
        self.angular_vel_2inercial_sys_in_vehicle_coord = np.dot(rotationalmatrix, self.chassis_w_velocity) #Bardini Pag. 261 Eq. 11.4    
        return self.angular_vel_2inercial_sys_in_vehicle_coord, self.rotationalmatrix
    
    
    def wheel_fix_coord_sys(self): #delta
        # Wheel fixed coordinate(KR) rotation relativ to Kv(vehicle system) Bardni pag. 260 eq. 11-9
       self.VTR_front_axel = [[                                                 np.cos(self.delta) * np.cos(self.x_a.theta_v),                                                -np.sin(self.delta) * np.cos(self.x_a.theta_v),                          -np.sin(self.x_a.theta_v)],
                              [ np.sin(self.x_a.psi_v) * np.sin(self.x_a.theta_v) * np.cos(self.delta) + np.cos(self.x_a.psi_v) * np.sin(self.delta),     -np.sin(self.x_a.psi_v) * np.sin(self.x_a.theta_v) * np.sin(self.delta) + np.cos(self.x_a.psi_v) * np.cos(self.delta),            np.sin(self.x_a.psi_v)* np.cos(self.x_a.theta_v)],
                              [ np.cos(self.x_a.psi_v) * np.sin(self.x_a.theta_v) * np.cos(self.delta) - np.sin(self.x_a.psi_v) * np.sin(self.delta),     -np.cos(self.x_a.psi_v) * np.sin(self.x_a.theta_v) * np.sin(self.delta) - np.sin(self.x_a.psi_v) * np.cos(self.delta),            np.cos(self.x_a.psi_v)* np.cos(self.x_a.theta_v)]]
        
       # Bardni. Pag. 260 eq. 11-10
       self.VTR_rear_axel = [[                  np.cos(self.x_a.theta_v),                                  0,                                   -np.sin(self.x_a.theta_v)],
                              [ np.sin(self.x_a.psi_v) * np.sin(self.x_a.theta_v),     np.cos(self.x_a.psi_v),            np.sin(self.x_a.psi_v)* np.cos(self.x_a.theta_v)],
                              [ np.cos(self.x_a.psi_v) * np.sin(self.x_a.theta_v),   - np.sin(self.x_a.psi_v),            np.cos(self.x_a.psi_v)* np.cos(self.x_a.theta_v)]]

    def access_z_road(x,y):
       #TODO implament topography
        z = 0.
        return z
    
    def road(self):
        for k in range(4):
            self.displacement.zs[k][2] = self.access_z_road(self.displacement.zs[k][0],self.displacement.zs[k][1])
            
        # type of file is opencrg
    
    def wheel(self, wheel_angle, time_stamp): #input: fz, mi, wheel_angle,vx,vy   output:fz,slix,slip_y
        '''initial states of tire load are mandatory [Wheel torque], [Wheel_Fz], slip_x, slip_angle]
            calculate the forces..
        '''
        # wheel states
        # initialize angular velocity of each wheel (relative to vehicle system)
        # TODO: How to acquire the each wheel vx,vy, pho_dot >>> initial state (where to update) #TODO: make each wheel vx[0]
        wheel_vx = [self.x_a.vx, self.x_a.vx, self.x_a.vx, self.x_a.vx] 
        wheel_vy = [self.x_a.vy, self.x_a.vy, self.x_a.vy, self.x_a.vy]
        # Distance to wheel center. Tire position relative to vehicle coordinate system (measured by total station)
        hub_distance = [self.V_r_A.fl, self.V_r_A.rl,self.V_r_A.fr,self.V_r_A.rr]
        
        # Calculating tire forces F_Ri: pag.267 eq 11_27
        for i in range(4):
            self.wheel_hub_position = hub_distance + np.matmul(self.transpose_vehicle_fixed2inertial_system,np.array([0],[0],[(
                -self.displacement.za - self.zr.zr + self.displacement.l_stat)]))         # eq 11-27
            self.wheel_hub_velocity =  self.velocity  + np.cross(self.wheel_w_vel,hub_distance)                          # eq 28
            
            #TODO: Check first argument by displaying (velocity in wheel fixed coord) Bardini eq 11-29
            self.wheel_vel_fix_coord_sys = np.matmul(np.matmul(self.vehicle_fixed2inertial_system, self.wheel_hub_position), self.wheel_hub_velocity)
            
            # Slip calculation - Wheel slip is calculated by wheel fixed system
            self.slip_x = (self.param.r_dyn[i] * self.wheel_w_vel[i] - wheel_vx[i] / max([self.param.r_dyn[i] * self.wheel_w_vel[i], wheel_vx[i]]))         # equation 11.30 Bardini
            # Lateral slip: define wheel_vy Equacao 11_28 >> converte to tire direction
            self.slip_y = - np.arctan( wheel_vy[i] / max([self.param.r_dyn * self.wheel_w_vel[i], wheel_vx[i]])) # equation 11.31 Bardini
        
            # TODO: Make F_stv diferent than 0; make F_st input on tire fuction
            # self.zs[i] = self.wheel_load[i]/self.param.cr[i]  >> self.displacemnt.zs
            
            # Bardini pag.268 eq 11-33
            self.wheel_load_z[i] = - max(self.param.cr[i]*(self.displacement.zr[i] - self.displacement.zs[i] + self.displacement.lr_stat[i]), 0)
            ' Replace with followig code to take antiroll bar in to account'
            # wheel_load_z[0] = - max(self.param.cr[0]*(self.displacement.zr[0]-self.displacement.zs[0]+lr_stat[0]) + F_stv , 0) # Bardini pag.268 eq 11-33
            # wheel_load_z[1] = - max(self.param.cr[1]*(self.displacement.zr[1]-self.displacement.zs[1]+lr_stat[1]) + F_sth, 0)
            # wheel_load_z[2] = - max(self.param.cr[2]*(self.displacement.zr[2]-self.displacement.zs[2]+lr_stat[2]) - F_stv, 0)
            # wheel_load_z[3] = - max(self.param.cr[3]*(self.displacement.zr[3]-self.displacement.zs[3]+lr_stat[3]) - F_sth, 0)
    
        return self.slip_x, self.slip_y
     
    def tire_model(self,time_stamp): # fz, slip
    
        for i in range(4):
            self.fx[i] = self.wheel_load_z[i] * self.param.d * np.sin(self.param.c * np.arctan(self.param.b*self.slip_x - self.param.e * (self.param.b*self.slip_x - np.arctan(self.param.b *self.slip_x))))
            self.fy[i] = self.wheel_load_z[i] * self.param.d * np.sin(self.param.c * np.arctan(self.param.b*self.slip_y - self.param.e * (self.param.b*self.slip_y - np.arctan(self.param.b *self.slip_y))))
            
            # Compile wheel forces xyz on one vector  and tranform  to Vechicle coordinate
            Compiled_wheel_forces = [[self.fx],
                                     [self.fy],
                                     [self.wheel_load_z]]
            
            if i % 2 == 0:
                #TODO: check matrix operation 3x3 3x4>> define wheel_forces_transfomed matrix
                self.Wheel_forces_transformed_force2vehicle_sys[i] =  self.VTR_front_axel * Compiled_wheel_forces # Bardini pag. 267 eq.32
            else: 
                self.Wheel_forces_transformed_force2vehicle_sys[i] =  self.VTR_rear_axel * Compiled_wheel_forces
            #self.sum_Compiled_wheel_forces[[sum(self.fx)], [sum(self.fy)][sum(self.wheel_load_z)]]    
           
            
    
    def suspension(self): # previsious forces, ax,ay 
        # za - displacement on chassis to suspension link
        # zr - displacement on wheel hub
        # Forces on the vehicle chassis at the pivot points Ai
        for i in range(4):    
        # Bardini pag. 265 eq. 11-21  
            self.spring_force[i] = (self.param.spring_stiff[i] * ((self.displacement.za[i]-self.displacement_zr[i]) + (
                self.displacement.l_stat[i]))) *  np.matmul(self.transpose_vehicle_fixed2inertial_system, np.array([[0],[0],[1]]))        
        # Bardini pag. 266 eq. 11-22
            self.dumper_force[i] = (self.param.dumper[i] * (self.displacement.za_dot[i] - self.displacement.zr_dot[i]
                                                            ) + self.param.dumper[i]) * np.matmul(self.transpose_vehicle_fixed2inertial_system, np.array([[0],[0],[1]]))
            self.f_za.f_za[i] = self.spring_force[i] + self.dumper_force[i]         
            # forces on the vehicle chassis (Ai) >> Bardini pag 236  >> horizontal forces pag 264 self.f_za.f_za
            self.strut2chassi_xyz[i] = [self.fx[i], self.fy[i], self.f_za.f_za[i]] # Fi , may be transpose 
            
        # V_F_fi - Forces on tires action on the chassis at the pivot points Ai of the four wheel >> Bardini pag 263
        self.strut2chassi = [[self.f_za.f_za[0]],
                             [self.f_za.f_za[1]],
                             [self.f_za.f_za[2]],
                             [self.f_za.f_za[3]]]
        
        #TODO transformation fx,fy (where? wheel)
        
        
        # TODO: where is used >> Transformation of positions 
        self.chassi_converted_force[i] = np.cross(self.position_chassi_force,self.strut2chassi_xyz[i]) # check calculus: f.za is z direction
        #TODO: check transpose_vehicle_fixed2inertial_system operation  
    
    def  chassis(self, vertical_loads, ax, ay, t_step):
            "Equations of motion Bardini, pag 272 ---- need initialize values"
            position = [[self.x_a.x],
                       [self.x_a.y],
                       [self.x_a.z]]
           
            velocity = [[self.x_a.vx],
                       [self.x_a.vy],
                       [self.x_a.vz]]
            
            self.angular_rates = [[self.x_a.wx],
                                  [self.x_a.wy],
                                  [self.x_a.wz]]

            self.angular_positions =  [[self.x_a.phi_v],
                                       [self.x_a.theta_v],
                                       [self.x_a.psi_v]]

            'sum of  wheel forces for calculating translation of the vehicle'
            
            self.sum_f_wheel[0] = np.sum(self.Wheel_forces_transformed_force2vehicle_sys[:, 0])
            self.sum_f_wheel[1] = np.sum(self.Wheel_forces_transformed_force2vehicle_sys[:, 1])
            self.sum_f_wheel[2] = np.sum(self.Wheel_forces_transformed_force2vehicle_sys[:, 2])
            
            # Equation 11-46 >> 11-12, Pag. 273
            # TODO: check gavity diretion
            self.acc[0] = (self.sum_f_wheel[0] - self.param.m * self.gravity + self.drag * self.xa.vx **2)/self.param.m + self.x_a.wz * self.xa.vy - self.x_a.wy * self.xa.vz
            self.acc[1] = (self.sum_f_wheel[1] - self.param.m * self.gravity + self.drag * self.xa.vy **2)/self.param.m + self.x_a.wx * self.xa.vz - self.x_a.wz * self.xa.vx
            self.acc[2] = (self.sum_f_wheel[1] - self.param.m * self.gravity)/self.param.m + self.x_a.wy * self.xa.vx - self.x_a.wx * self.xa.vy
            
            # vehicle velocity calculation
            self.x_a.vx = self.x_a.vx + self.acc[0]* self.time_step 
            self.x_a.vy = self.x_a.vy + self.acc[1]* self.time_step 
            self.x_a.vz = self.x_a.vz + self.acc[2]* self.time_step
            

            
            #TODO:Check rolling resistance            
            self.rolling_resist = (self.param.fr * self.param.m * self.param.gravity * np.cos(0.) - 0.5 * self.param.row * self.param.Cl * self.param.area * self.speed ** 2)                              # Rolling Resistance with air lift
                
            '''Equation 11-46, vehicle velocity xyz calculation: for angular velocities and postion the last step is used
                Values calculate for positions, atittude and its derivates needs to be saved
            '''
            # aero_dyn_center = [[self.param.cg_x],[self.param.cg_y][self.param.cg_height]] # Shoul be center of aero pressure
            for i in range(4):
                
                self.crossproduct_r_f[i][0] = self.position_chassi_force[i][1] * self.strut2chassi_xyz[i][2] -self.self.position_chassi_force[i][2] * self.strut2chassi_xyz[i][1] 
                self.crossproduct_r_f[i][1] = self.position_chassi_force[i][2] * self.strut2chassi_xyz[i][0] -self.self.position_chassi_force[i][0] * self.strut2chassi_xyz[i][2]
                self.crossproduct_r_f[i][2] = self.position_chassi_force[i][0] * self.strut2chassi_xyz[i][1] -self.self.position_chassi_force[i][1] * self.strut2chassi_xyz[i][0]
        
            # TODO Check sum()
            self.sum_crossproduct_r_f = sum(self.crossproduct_r_f,axis=0)        
            self.acc_angular_v = (self.sum_crossproduct_r_f - np.cross(self.vector_w,(np.dot(self.polar_inertia_v,self.vector_w)))/self.polar_inertia_v)  # check inverse operation of multiplication

            # Angular velocity of the chassis
            self.wx = self.x_a.wx + self.acc_angular_v[0] * self.time_step
            self.wy = self.x_a.wy + self.acc_angular_v[1] * self.time_step
            self.wz = self.x_a.wz + self.acc_angular_v[3] * self.time_step
              
            # Angular position
            self.phi_v =    self.x_a.wx * self.time_step + self.phi_v
            self.theta_v =  self.x_a.wy * self.time_step + self.theta_v
            self.psi_v =    self.x_a.wz * self.time_step + self.psi_v
            
            # bardini pag 260 -- use vector of torque x euler angle rate
            
            # vehicle position calculation
            movement_vehicle =  (self.x_a.vx * self.time_step, self.x_a.vy * self.time_step, self.x_a.vz * self.time_step) 
                        
            #TODO tranformation form vehicle sys to Ke
            # Transformation matrix vehicle coord sys(Kv) into inertia sys(Ke) - E_T_V -- Bardini pag 260 eq. 11.3
            vehicle_fixed2inertial_system = [[np.cos(self.x_a.theta_v) * np.cos(self.x_a.psi_v), np.sin(self.x_a.phi_v) * np.sin(self.x_a.theta_v) * np.cos(self.x_a.psi_v) - np.cos(self.x_a.phi_v) * np.sin(self.x_a.psi_v),     np.cos(self.x_a.phi_v) * np.sin(self.x_a.theta_v) * np.cos(self.x_a.psi_v) + np.sin(self.x_a.phi_v) * np.sin(self.x_a.psi_v)],
                                                  [np.cos(self.x_a.theta_v) * np.sin(self.x_a.psi_v), np.sin(self.x_a.phi_v) * np.sin(self.x_a.theta_v) * np.sin(self.x_a.psi_v) + np.cos(self.x_a.phi_v) * np.sin(self.x_a.psi_v),     np.cos(self.x_a.phi_v) * np.sin(self.x_a.theta_v) * np.sin(self.x_a.psi_v) - np.sin(self.x_a.phi_v) * np.cos(self.x_a.psi_v)],
                                                  [-np.sin(self.x_a.theta_v),                         np.sin(self.x_a.phi_v) * np.cos(self.x_a.theta_v),                                                      np.cos(self.x_a.phi_v) * np.cos(self.x_a.theta_v)]]     #  Bardini pag 260
            # transformation from Ke to Kv. 
            self.transpose_vehicle_fixed2inertial_system = np.transpose(vehicle_fixed2inertial_system)
            #TODO check mat mul ordem
            [self.x_a.x,self.x_a.y,self.x_a.z] = [self.x_a.x,self.x_a.y,self.x_a.z] + np.matmul(movement_vehicle,vehicle_fixed2inertial_system)
              
            # TODO new displacements need taking euler angles into account
    
    
    def tick(self, gas_pedal, brake, steering):
        self.powertrain(gas_pedal, brake, self.param.rpm_table, self.param.torque_max) # 
        self.steering(self,steering)
        #self.wheel_fix_coord_sys(self)
        self.newton_euler_wheel(self)
        self.wheel(self)
        self.tire_model(self)
        self.suspension(self)
        self.chassis(self)  
        
################################################################################################################################        

class StateVector(object):
    def __init__(self, x=0, y=0, z=0, phi_v=0, theta_v=0, psi_v=0, vx=0, vy=0, vz=0,wx=0, wy=0, wz=0, acc = [0,0,0]):
        
        # Position and Euler angles
        self.x = x
        self.y = y 
        self.z = z 
        self.phi_v = phi_v
        self.theta_v = theta_v
        self.psi_v = psi_v
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.wx = wx 
        self.wy= wy 
        self.wz = wz
        self.acc = acc
        
class StrutForce(object):
    def __init__(self, f_za=[0,0,0,0], f_za_dot=[0,0,0,0], dumper_force = [0,0,0,0]): #TODO calculate forces on the strut
        self.f_za = f_za
        self.f_za_dot = f_za_dot
        self.dumper_force = dumper_force 
        
        
class WheelHubForce(object):
    def __init__(self, f_zr=[0,0,0,0], f_zr_dot=[0,0,0,0]):
        self.f_zr = f_zr
        self.f_zr_dot =f_zr_dot
        
class AngularWheelPosition(object):
    def __init__(self, pho_r=[0,0,0,0], pho_r_dot = [0,0,0,0],pho_r_2dot = [0,0,0,0]):
    
        # pag 272 eq.52
        # Angular position/speed of each wheel
        self.pho_r = pho_r
        self.pho_r_dot = pho_r_dot
        self.pho_r_2dot = pho_r_2dot
        
class TireForces(object):
    def __init__ (self, fx1=0, fy1=0,fx2=0, fy2=0, fx3=0,fy3=0, fx4=0, fy4=0):        
        # Dynamic forces on the tires
        # pag 272 eq.54
        self.fx1 = fx1
        self.fy1 = fy1
        self.fx2 = fx2
        self.fy2 = fy2
        self.fx3 = fx3
        self.fy3 = fy3
        self.fx4 = fx4
        self.fy4 = fy4
    
class Displacement(object):
    def __init__ (self, l_stat=[0,0,0,0], za=[0,0,0,0], zr=[0,0,0,0], za_dot=[0,0,0,0], zr_dot=[0,0,0,0],zr_2dot =[0,0,0,0]):
        self.l_stat = l_stat        
        self.za = za
        self.za_dot = za_dot
        self.zr_2dot = zr_2dot
        self.zr_dot = zr_dot
        self.zr = zr
        # Vertical displacements/velocity of the wheel *(x_rz)
        # pag 272 eq.53
        
class ImportParam(object):

    def __init__(self, path = 'config.yaml'):
        
        with open(path, 'r') as file:
          param = yaml.safe_load(file)
    
        
        #=====================================
        # Engine Coeficients
        #=====================================
        
        self.rpm_table = np.array(param['vehicle_model']['parameters']['rpm_table'])
        self.torque_max = np.array(param['vehicle_model']['parameters']['torque_max'])
        self.gear_ratio = np.array(param['vehicle_model']['parameters']['gear_ratio'])
        self.gear_selection = np.array(param['vehicle_model']['parameters']['gear_selection'])
        self.min_rpm = param['vehicle_model']['parameters']['min_rpm'] 
        self.diff = param['vehicle_model']['parameters']['diff']                       # Differential ratio
        self.diff_ni = param['vehicle_model']['parameters']['diff_ni']
        self.transmition_ni = param['vehicle_model']['parameters']['transmition_ni']
        
      
        #=====================================
        # Tire Data
        #=====================================
        self.cx = param['vehicle_model']['parameters']['c_tire_x']                                     # Tire Stiffiness [N/m]
        self.cy = param['vehicle_model']['parameters']['c_tire_y']                                     # Tire Stiffiness [N/m]
        self.F_max = param['vehicle_model']['parameters']['f_max']                                     # Tire max load
        self.bx = np.array(param['vehicle_model']['parameters']['bx'])                                           # tire coeficient Bardini pag 268
        self.by = np.array(param['vehicle_model']['parameters']['by'])
        self.cr = np.array(param['vehicle_model']['parameters']['cr'])                                 # Tire vertical Stiffnes
        self.K_lt = param['vehicle_model']['parameters']['k_lt']                                       # lateral compliance rate of tire, wheel, and suspension, per tire [m/N]  KLT
        self.i_wheel = np.array(param['vehicle_model']['parameters']['i_wheel'])
        self.r_stat = param['vehicle_model']['parameters']['r_stat']                                   # Tire Radius Static [m]
        self.r_dyn  = np.array(param['vehicle_model']['parameters']['r_dyn'])    
        # Rolling Resistance Parameters
        self.mi_x = param['vehicle_model']['parameters']['road_friction_x']   ##                       # Road Friction Coefficient
        self.mi_y = param['vehicle_model']['parameters']['road_friction_y']
        self.d = ['vehicle_model']['parameters']['magic_formula_d']                                    # Magic formula coefficient
        self.e = ['vehicle_model']['parameters']['magic_formula_e']
        self.c = ['vehicle_model']['parameters']['magic_formula_c']
        self.b = ['vehicle_model']['parameters']['magic_formula_b']
        #=====================================
        # Resistance Parameters
        #=====================================
        
        self.Cd = param['vehicle_model']['parameters']['cd']                                                 # Air Drag Coefficient
        self.Front_area = param['vehicle_model']['parameters']['front_area']  ###                            # Front Area of Car
        self.Air_density = param['vehicle_model']['parameters']['air_density']                               # Air Dencity  
        self.brake_caliper = param['vehicle_model']['parameters']['brake_caliper']                           # Break Calib Coefficient
        
        # "https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/PYTHON/vehiclemodels/vehicle_parameters.py "
    
        # steering constraints
        self.steering_min = param['vehicle_model']['parameters']['steering_min']  # minimum steering angle [rad]
        self.steering_max = param['vehicle_model']['parameters']['steering_max']  # maximum steering angle [rad]
        self.steering_v_min = param['vehicle_model']['parameters']['steering_v_min']  # minimum steering velocity [rad/s]
        self.steering_v_max = param['vehicle_model']['parameters']['steering_v_max']  # maximum steering velocity [rad/s]
        #=====================================
        # Masses
        self.m_s = np.array(param['vehicle_model']['parameters']['m_s'])                            # sprung mass [kg]  
        self.unsprung_mass = np.array(param['vehicle_model']['parameters']['unsprung_mass'])        # unsprung mass vector [kg]  
        self.m = param['vehicle_model']['parameters']['mass']                                       # Vehicle Mass [kg] 
        #=====================================
        "Intertial Resistance Parameters need to be measured"
        #=====================================
        #self.iw = param['vehicle_model']['parameters']['iw']                                    # Wheel inertia [Kgm^2]
        #self.ia = param['vehicle_model']['parameters']['ia']                                    # Axel inertia [Kgm^2]
        #self.id = param['vehicle_model']['parameters']['id']                                    # drive shaft inertia [Kgm^2]
        #self.Gd = param['vehicle_model']['parameters']['gd']                                    # Differential Gear Ratio
        #self.ig = param['vehicle_model']['parameters']['ig']
        self.engine_inertia = param['vehicle_model']['parameters']['engine_inertia']                              # Engine inertia [Kgm^2]
        self.i_uf = param['vehicle_model']['parameters']['i_uf']        # moment of inertia for unsprung mass about x-axis (front) [kg m^2]  IXUF
        self.i_ur = param['vehicle_model']['parameters']['i_ur']        # moment of inertia for unsprung mass about x-axis (rear) [kg m^2]  IXUR
        self.i_y_w = param['vehicle_model']['parameters']['i_y_w']      # wheel inertia, from internet forum for 235/65 R 17 [kg m^2]
        self.i_x_s = param['vehicle_model']['parameters']['i_x_s']      # moment of inertia for sprung mass in roll [kg m^2]  iXS
        self.i_y_s =  param['vehicle_model']['parameters']['i_y_s']     # moment of inertia for sprung mass in pitch [kg m^2]  iYS
        self.i_z = param['vehicle_model']['parameters']['i_z']          # moment of inertia for sprung mass in yaw [kg m^2]  iZZ
        self.i_xz_s = param['vehicle_model']['parameters']['i_xz_s']    # moment of inertia cross product [kg m^2]  iXZ
        self.i_d_shaft = param['vehicle_model']['parameters']['i_d_shaft']                    # calculate
        self.i_clutch = param['vehicle_model']['parameters']['i_clutch']
  
        # suspension parameters
        self.spring_stiff = np.array(param['vehicle_model']['parameters']['spring_stiff'])      # suspension spring rate (front i = 1 or 3)  (rear = 1 = 2,4) [N/m]  KSF 
        self.f_dumper = np.array(param['vehicle_model']['parameters']['f_dumper'])              # suspension damping rate (front i = 1 or 3)  (rear = 1 = 2,4)[N s/m]  KSDF
        self.anti_roll_stiffness = param['vehicle_model']['parameters']['anti_roll_stiffness']  # Anti roll bar stiffness [Nm/rad]
        self.K_ras = param['vehicle_model']['parameters']['k_ras']                              # lateral spring rate at compliant compliant pin joint between M_s and M_u [N/m]  KRAS
        self.K_tsf = param['vehicle_model']['parameters']['k_tsf']                              # auxiliary torsion roll stiffness per axle (normally negative) (front) [N m/rad]  KTSF
        self.K_tsr = param['vehicle_model']['parameters']['k_tsr']                              # auxiliary torsion roll stiffness per axle (normally negative) (rear) [N m/rad]  KTSR
        self.K_rad = param['vehicle_model']['parameters']['k_rad']                              # damping rate at compliant compliant pin joint between M_s and M_u [N s/m]  KRADP
        self.K_zt = param['vehicle_model']['parameters']['k_zt']                                # vertical spring rate of tire [N/m]  TSPRINGR
        
        #=====================================
        # geometric parameters
        #=====================================
        
        self.cg_height = param['vehicle_model']['parameters']['cg_height']                   # center of gravity height of total mass [m]
        self.cg_x = param['vehicle_model']['parameters']['cg_x']                             #  cg_x [m]
        self.cg_y = param['vehicle_model']['parameters']['cg_y']                             # [m]
        self.h_cg = param['vehicle_model']['parameters']['h_cg']
        self.h_raf = param['vehicle_model']['parameters']['h_raf']                           # height of roll axis above ground (front) [m]  HRAF
        self.h_rar = param['vehicle_model']['parameters']['h_rar']                           # height of roll axis above ground (rear) [m]  HRAR
        self.h_s = param['vehicle_model']['parameters']['h_s']                               # M_s center of gravity above ground [m]  HS
        self.wheel_base = param['vehicle_model']['parameters']['wheel_base']                 # [m]
        # axes distances
        self.lv = param['vehicle_model']['parameters']['lv']                                  # x-distance from Vehicle CoG to the front hub [m]
        self.lh = param['vehicle_model']['parameters']['lh']                                  # x-distance from Vehicle Cog to the rear hub [m]
        # Half track
        self.sl = param['vehicle_model']['parameters']['sl']                                  # Half track width front [m]
        self.sr = param['vehicle_model']['parameters']['sr']                                  # Half track width rear  [m]
        # vehicle body dimensions
        self.l = param['vehicle_model']['parameters']['length']                               # vehicle length [m]
        self.width = param['vehicle_model']['parameters']['width']                            # vehicle width [m]
        
        #=====================================
        # Powertrain parameters
        #=====================================
        self.speed_ratio_TC = np.array(param['vehicle_model']['parameters']['speed_ratio_TC'])
        self.torque_converter_ratio = np.array(param['vehicle_model']['parameters']['torque_converter_ratio'])