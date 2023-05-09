
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

    def __init__(self, initial_speed = 100., state_0 = [0., 0., 0., 0., 0., 0.], initial_gear = 0, freq=100, param_path = ""):
    
        if param_path != "":
            self.param = ImportParam(param_path) # Import all the Parameters 
            
        #self.position = state_0
        self.time_step = 1/freq
        self.last_delta = 0
        self.delta = 0
        self.gravity = 9.81
        self.rpm = self.param.min_rpm
        self.gear = initial_gear                    # gear selector
        self.throttle = 0.0                         # thorttle position (0< throttle <1)
        self.brake = 0.0                            # Brake pedal position (0< brake <1)
        self.alpha_engine = 0.0                     # Angular acc engine
        self.wheel_w_vel = np.array([0.,0.,0.,0.])
        self.torque_converter_ratio_inter = interp1d(self.param.speed_ratio_TC, self.param.torque_converter_ratio)
        self.drag = 0.5*self.param.row * self.param.Cd * self.param.Front_area  # constant for air resistance
        
        # TODO:Global Coordinates system  -- inertial system (O_E)
     
        # TODO:Coordinates of the vehicle CoG in the inertial system O_V
        o_v = [self.param.cg_x, self.param.cg_y, self.param.cg_height] # [xv, yv,zv] [m]
        
        # State
        self.x_a = StateVector(x=state_0[0],y=state_0[1],z=state_0[2], phi_v=state_0[3], theta_v=state_0[4], psi_v=state_0[5], vx =initial_speed, vy=0.,vz=0., wx=0., wy=0., wz=0.) # State_0
        self.x_rr = AngularWheelPosition(pho_r=[0.,0.,0.,0.], pho_r_dot = [0.,0.,0.,0.],pho_r_2dot =[0.,0.,0.,0.])
        # self.x_rz = WheelsPositionZ(z_r=[0.,0.,0.,0.], z_r_dot=[0.,0.,0.,0.])
        self.x_rf = TireForces(fx =np.array([0.,0.,0.,0.]), fy =np.array([0.,0.,0.,0.]),wheel_forces_transformed_force2vehicle_sys = np.zeros((4,3),dtype=float))
        self.displacement = Displacement(l_stat=[0.,0.,0.,0.], za= np.array([0.,0.,0.,0.]), zr =[0.,0.,0.,0.], za_dot=[0.,0.,0.,0.], zr_dot=[0.,0.,0.,0.],zr_2dot=[0.,0.,0.,0.])
        self.f_za = StrutForce(f_za = np.array([0.,0.,0.,0.]), f_za_dot = np.array([0.,0.,0.,0.]),spring_force = np.zeros((4,1),dtype=float),dumper_force = np.zeros((4,1),dtype=float))
        #################### TODO calculate initial forcers
        self.f_zr = WheelHubForce(f_zr=[10.,10.,10.,10.],f_zr_dot=[0.,0.,0.,0.], wheel_load_z =[200.,300.,200.,300.])
        #TODO wheel hub inital
        self.wheel_hub_position = np.array([[[0.],[0.],[0.]],[[0.],[0.],[0.]],[[0.],[0.],[0.]],[[0.],[0.],[0.]]])
        self.wheel_hub_velocity = np.array([[[0.],[0.],[0.]],[[0.],[0.],[0.]],[[0.],[0.],[0.]],[[0.],[0.],[0.]]])
        self.polar_inertia_v = np.array([[self.param.i_x_s, 0., 0.],
                                        [0., self.param.i_y_s, 0.],
                                        [0., 0., self.param.i_z]])
        
        # TODO# Suspension to chassis conection (position Ai) Bardini eq. 11-20
        self.position_chassi_force = np.zeros((4,3,1),dtype=float)
        
        self.position_chassi_force[0] = np.array([[self.param.lv],[self.param.sl],[-self.param.sz]])                       
        self.position_chassi_force[1] = np.array([[-self.param.lh],[self.param.sl],[-self.param.sz]])
        self.position_chassi_force[2] = np.array([[self.param.lv],[-self.param.sr],[-self.param.sz]])
        self.position_chassi_force[3] = np.array([[-self.param.lh],[-self.param.sr],[-self.param.sz]])

        # Forces on the chassis
        self.strut2chassi_xyz = np.zeros((4,3),dtype=float)
        
        #Foces on the wheel
        self.compiled_wheel_forces = np.zeros((4,3),dtype= float)
        
        # Static displacement of the springs
        self.displacement.l_stat = np.array(self.param.m_s) / self.param.spring_stiff
        print('l stat',self.displacement.l_stat)
        #TODO: make zr depend on zs
        self.displacement.zr = np.array(self.param.m/self.param.cr)
        # print('self.displacement.zr',self.displacement.zr)
        # TODO: check m is adressed for each wheel 
        self.lr_stat = self.param.m/self.param.cr                       # Tire static deformation
        print('self.lr_stat',self.lr_stat)

        # Creating vector for chassis method 
        self.sum_f_wheel = np.zeros(3, dtype=float) # Sum of wheel forces
        self.acc = np.zeros((3),dtype=float) # acceleration
        self.crossproduct_r_f = np.zeros((4,3), dtype=float)
        
        #TODO tranformation form vehicle sys to Ke
        # Transformation matrix vehicle coord sys(Kv) into inertia sys(Ke) - E_T_V -- Bardini pag 260 eq. 11.3
        self.vehicle_fixed2inertial_system = np.array([[np.cos(self.x_a.theta_v) * np.cos(self.x_a.psi_v), np.sin(self.x_a.phi_v) * np.sin(self.x_a.theta_v) * np.cos(self.x_a.psi_v) - np.cos(self.x_a.phi_v) * np.sin(self.x_a.psi_v),     np.cos(self.x_a.phi_v) * np.sin(self.x_a.theta_v) * np.cos(self.x_a.psi_v) + np.sin(self.x_a.phi_v) * np.sin(self.x_a.psi_v)],
                                              [np.cos(self.x_a.theta_v) * np.sin(self.x_a.psi_v), np.sin(self.x_a.phi_v) * np.sin(self.x_a.theta_v) * np.sin(self.x_a.psi_v) + np.cos(self.x_a.phi_v) * np.sin(self.x_a.psi_v),     np.cos(self.x_a.phi_v) * np.sin(self.x_a.theta_v) * np.sin(self.x_a.psi_v) - np.sin(self.x_a.phi_v) * np.cos(self.x_a.psi_v)],
                                              [-np.sin(self.x_a.theta_v),                         np.sin(self.x_a.phi_v) * np.cos(self.x_a.theta_v),                                                      np.cos(self.x_a.phi_v) * np.cos(self.x_a.theta_v)]])     #  Bardini pag 260
        # transformation from Ke to Kv. 
        self.transpose_vehicle_fixed2inertial_system = np.transpose(self.vehicle_fixed2inertial_system)
        self.wheel_vel_fix_coord_sys = np.zeros((4,3),dtype=float)
        
        
        
    def debug_powertrain(self, gas_pedal, brake, steering):
        self.powertrain(gas_pedal, brake, self.param.rpm_table, self.param.torque_max)
        return self.powertrain_net_torque, self.powertrain_force
        
        
    def powertrain(self, throttle, brake, rpm_table, torque_max_table):
        if self.rpm > 1200.:
            self.rpm = 1200.
        torque_interpolation = interp1d(rpm_table, torque_max_table)    # how much torque is available thoughout the rpm range 
        torque_available = torque_interpolation(self.rpm)               # Max torque available at rpm
        engine_torque = throttle * torque_available                     # find the torque delivered by te engine
        # print('eng torque', engine_torque)
        
        #-----------Tranmission ratio----------
        # Comparing engine rpm and torque converter rpm to define the available torque. 
        for i in range(len(self.param.gear_ratio)): # checking number of gears
            if self.x_a.acc[0] >= 0.0:
                #TODO check if int() is working 
                #if self.x_a.vx > self.param.gear_selection[int(self.throttle*10)][self.x_a.vx]:  
                if self.x_a.vx > self.param.gear_selection[int(self.throttle*10)][int(self.x_a.vx)]:
                    if i >6:
                        i=6
                        self.gear = self.param.gear_ratio[i+1]
                     
        # Using an offset of upshift
            if self.x_a.acc[0] <0:
                if  (self.x_a.vx * 0.8) < self.param.gear_selection[round(self.throttle*10)][self.x_a.vx]:
                    self.gear = self.param.gear_ratio[i]
            else:
                self.gear = self.param.gear_ratio[0]    # neutral    
        
        # print(self.x_a.vx)
        #self.wheel_rpm = 30 * self.x_a.vx/( self.param.r_dyn * np.pi) #RADSEC2RPM = 30/np.pi
        # replaced by self.wheel_w_vel
        
        # self.tcrpm = self.gear * self.wheel_w_vel[0] * self.torque_converter_ratio_inter
        # self.driveshaft_rpm = self.param.final_ratio * self.wheel_w_vel[0]
        # speed_ratio = self.driveshaft_rpm/self.rpm 
        # if speed_ratio > 0.98:
        #     speed_ratio = 1.0
        # torque_convertion = self.torque_converter_ratio_inter(speed_ratio)
        
        # TODO: Engine torque to wheel, taking inercia into account (separated by gear)
        self.final_ratio = self.gear * self.param.diff # check final gear calculation
        
        #TODO: check inertias and formulation
        self.traction_torque = (engine_torque * self.final_ratio * self.param.diff_ni * 
                                self.param.transmition_ni) - ((self.param.engine_inertia + self.param.ia + self.param.ig
                                                               ) * self.gear ** 2 + self.param.id * self.final_ratio ** 2 + 
                                                               self.param.i_wheel) * self.x_a.acc[0]
        # print(self.traction_torque)                                                       
        # print("engine_torque",engine_torque,'final_r',self.final_ratio,
        #       'diff_ni',self.param.diff_ni,'trans_ni',self.param.transmition_ni,
        #       'eng_nerti',self.param.engine_inertia) 
        # print(self.param.ia,self.param.ig ,
        #       self.gear,self.param.id, self.final_ratio, self.param.i_wheel)
                                                               
        #--------------------Break Torque -------------------------
        # using brake pedal as input, the brake torque can be calculated by 
        self.brake_torque = self.brake * self.param.max_brake_torque
        #print(self.brake)
        #-------------------- Net Torque -------------------------
        self.powertrain_net = self.traction_torque - self.brake_torque
        # print("ac",self.x_a.acc[0],type(self.x_a.acc[0])) 
        # print("tract",self.traction_torque,self.traction_torque.shape,type(self.traction_torque))
        
        # function traction divided between axles
        self.powertrain_net_torque = np.empty(4, dtype = float)
        self.powertrain_net_torque[0] = self.powertrain_net*0.5* self.param.b_bias  
        self.powertrain_net_torque[1] = self.powertrain_net*0.5* (1-self.param.b_bias)
        self.powertrain_net_torque[2] = self.powertrain_net*0.5* self.param.b_bias
        self.powertrain_net_torque[3] = self.powertrain_net*0.5* (1-self.param.b_bias)
        self.powetrain_force = self.powertrain_net_torque/self.param.r_dyn
        # print(self.powetrain_force, 'N')
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
        # return self.powertrain_net_torque, self.powertrain_force
    
   
    def steering(self):
        delta_dot = float((self.last_delta - self.delta)/self.time_step)
        # delta is steering angle 
        if (self.delta <= self.param.steering_min and delta_dot <= 0) or (self.delta >= self.param.steering_max and delta_dot >= 0):
            delta_dot = 0
        elif delta_dot <= self.param.steering_v_min:
            delta_dot = self.param.steering_v_min
        elif delta_dot >= self.param.steering_v_max:
            delta_dot = self.param.steering_v_max
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
            
        return self.wheel_angle_front, self.wheel_angle_rear
    
    def newton_euler_wheel(self):
        ''' wheel velocities are used to calcule slip, than when can futher calculate ax,ay using the forces 
        using the magic formula
        '''
        # NEWTON’s and EULER’s Equations of the Wheels: Solve  11-25 Bardini pag 266
        for i in range(4):
            # 11-26 Bardini pag 266. Where fx(tire_road force) is determined by magic formula
            self.displacement.zr_2dot[i] =  (self.f_zr.wheel_load_z[i] - self.f_za.f_za[i] - self.param.wheel_mass[i] * self.gravity) /self.param.wheel_mass[i]
            # print('self.f_zr.wheel_load_z[i]',self.f_zr.wheel_load_z[i])
            print('fza i',self.f_za.f_za[i])
            break
            # print('wheel_mass[i]',self.param.wheel_mass[i])
            self.x_rr.pho_r_2dot[i] =  (self.powertrain_net_torque[i] - self.param.r_dyn[i] * self.x_rf.fx[i]) /self.param.iw[i]
            # print('acc ang',self.x_rr.pho_r_2dot[i])
            # print('ang vel',self.wheel_w_vel[i])
            # Calculate the intregral of the wheel_acc to calculate slip_x
            self.wheel_w_vel[i] = self.wheel_w_vel[i] + self.x_rr.pho_r_2dot[i] * self.time_step
            # Calculate the wheel hub displament
            self.displacement.zr_dot = self.displacement.zr_dot + self.displacement.zr_2dot[i] * self.time_step
            self.displacement.zr = self.displacement.zr + self.displacement.zr_dot * self.time_step
    
    
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
       self.VTR_front_axel = np.array([[                                                 np.cos(self.delta) * np.cos(self.x_a.theta_v),                                                -np.sin(self.delta) * np.cos(self.x_a.theta_v),                          -np.sin(self.x_a.theta_v)],
                                       [ np.sin(self.x_a.psi_v) * np.sin(self.x_a.theta_v) * np.cos(self.delta) + np.cos(self.x_a.psi_v) * np.sin(self.delta),     -np.sin(self.x_a.psi_v) * np.sin(self.x_a.theta_v) * np.sin(self.delta) + np.cos(self.x_a.psi_v) * np.cos(self.delta),            np.sin(self.x_a.psi_v)* np.cos(self.x_a.theta_v)],
                                       [ np.cos(self.x_a.psi_v) * np.sin(self.x_a.theta_v) * np.cos(self.delta) - np.sin(self.x_a.psi_v) * np.sin(self.delta),     -np.cos(self.x_a.psi_v) * np.sin(self.x_a.theta_v) * np.sin(self.delta) - np.sin(self.x_a.psi_v) * np.cos(self.delta),            np.cos(self.x_a.psi_v)* np.cos(self.x_a.theta_v)]])
        
       # Bardni. Pag. 260 eq. 11-10
       self.VTR_rear_axel = np.array([[                  np.cos(self.x_a.theta_v),                                  0,                                   -np.sin(self.x_a.theta_v)],
                                      [ np.sin(self.x_a.psi_v) * np.sin(self.x_a.theta_v),     np.cos(self.x_a.psi_v),            np.sin(self.x_a.psi_v)* np.cos(self.x_a.theta_v)],
                                      [ np.cos(self.x_a.psi_v) * np.sin(self.x_a.theta_v),   - np.sin(self.x_a.psi_v),            np.cos(self.x_a.psi_v)* np.cos(self.x_a.theta_v)]])
       
  
       
    def access_z_road(x,y):
       #TODO implament topography
        z = 0.
        return z
    
    def road(self):
        for k in range(4):
        #     self.displacement.zs[k][2] = self.access_z_road(self.displacement.zs[k][0],self.displacement.zs[k][1])
            self.displacement.zs[k]=0.
        
        # print('self.displacement.zs',self.displacement.zs)    
        # type of file is opencrg
        
    
    def wheel(self, wheel_angle, time): #input: fz, mi, wheel_angle,vx,vy   output:fz,slix,slip_y
        '''initial states of tire load are mandatory [Wheel torque], [Wheel_Fz], slip_x, slip_angle]
            calculate the forces..
        '''
        # wheel states
        # initialize angular velocity of each wheel (relative to vehicle system)
        # TODO: How to acquire the each wheel vx,vy, pho_dot >>> initial state (where to update) #TODO: make each wheel vx[0]
        wheel_vx = np.array([self.x_a.vx, self.x_a.vx, self.x_a.vx, self.x_a.vx])
        wheel_vy = np.array([self.x_a.vy, self.x_a.vy, self.x_a.vy, self.x_a.vy])
        # Distance to wheel center. Tire position relative to vehicle coordinate system (measured by total station)
        hub_distance = np.array([self.param.hub_fl, self.param.hub_rl,self.param.hub_fr,self.param.hub_rr])
        
        # Calculating tire forces F_Ri: pag.267 eq 11_27
         
        
        for i in range(4):
            b =-self.displacement.za[i] + self.displacement.zr[i] - self.displacement.l_stat[i]
            v = np.array([[0],[0],[b]])
            tranq = np.matmul(self.transpose_vehicle_fixed2inertial_system,v)
            #print(' self.position_chassi_force[i]',self.position_chassi_force[i],self.position_chassi_force[i].shape,type(self.position_chassi_force))
            #print('wheel_w_vel',self.wheel_w_vel,type(self.wheel_w_vel))
            #print('       ')
            #print('wheel_hub_position[i]',self.wheel_hub_position[i],self.wheel_hub_position[i].shape,type(self.wheel_hub_position[i]))
            #print('transpose_vehicle_fixed2inertial',self.transpose_vehicle_fixed2inertial_system,  self.transpose_vehicle_fixed2inertial_system.shape)
            #print('       ')
            # print('tranq=',tranq,tranq.shape,type(tranq)) 
            
            ### self.wheel_hub_position[i] = self.position_chassi_force[i] + np.matmul(self.transpose_vehicle_fixed2inertial_system,np.array([[0],[0],[(b)]]))         # eq 11-27
            self.wheel_hub_position[i]= self.position_chassi_force[i] + tranq        # eq 11-27
            
            # print('wheel_w_vel',self.wheel_w_vel,self.wheel_w_vel.shape)
            # print('position_chassi_force[i]')
            # print(self.position_chassi_force[i],self.position_chassi_force[i].shape)
            c = np.zeros(3)
            d = self.wheel_w_vel[i]
            # print('d',d)
            c= np.array([d,0,0])
            # print('c',c,c.shape)
            #print('self.wheel_hub_velocity[i]', self.wheel_hub_velocity[i],self.wheel_hub_velocity[i].shape)
            e = np.squeeze(self.position_chassi_force[i])
            # print('e',e)
            f = np.reshape((np.cross(c,e)),(3,1))
            #print('f',f,f.shape,type(f))
            #print('wheel_vx[i]',wheel_vx[i],wheel_vx[i].shape)
            #print('vx',self.x_a.vx,type(self.x_a.vx))
            #print('vy',self.x_a.vy,type(self.x_a.vy))
            #print('zr i ',self.displacement.zr[i],type(self.displacement.zr[i]))
            
            #TODO: FIX
            self.wheel_hub_velocity[i] =  np.array([[self.x_a.vx],[self.x_a.vy], [self.displacement.zr[i]]]) + f # eq 28
            
            #self.wheel_hub_velocity[i] =  np.array([[self.x_a.vx],[self.x_a.vy],[self.x_a.vz]]
            #                                    ) + np.cross(self.wheel_w_vel,self.position_chassi_force[i]) # eq 28
            
            #TODO: Check first argument by displaying (velocity in wheel fixed coord) Bardini eq 11-29
            # print('wheel_vel_fix_coord_sys' )
            # print(self.wheel_vel_fix_coord_sys.shape,self.wheel_vel_fix_coord_sys)
            # print('vehicle_fixed2inertial_system')
            # print(self.vehicle_fixed2inertial_system,self.vehicle_fixed2inertial_system.shape)
            # print('self.wheel_hub_position')
            # print(self.wheel_hub_position,self.wheel_hub_position.shape)
            
            #TODO : Chech wheel_hub_velocity
            self.wheel_vel_fix_coord_sys = np.matmul(self.vehicle_fixed2inertial_system, self.wheel_hub_velocity[i])
            
            # Slip calculation - Wheel slip is calculated by wheel fixed system
            self.slip_x = ((self.param.r_dyn[i] * self.wheel_w_vel[i] - wheel_vx[i] )/ max([self.param.r_dyn[i] * self.wheel_w_vel[i]+1e-16, 1e-16+wheel_vx[i]]))         # equation 11.30 Bardini
            
            # Lateral slip: define wheel_vy Equacao 11_28 >> converte to tire direction
            self.slip_y = - np.arctan( wheel_vy[i] / max([self.param.r_dyn[i] * self.wheel_w_vel[i]+1e-16, 1e-16 + wheel_vx[i]])) # equation 11.31 Bardini
        
            # TODO: Make F_stv diferent than 0; make F_st input on tire fuction
            # self.zs[i] = self.wheel_load[i]/self.param.cr[i]  >> self.displacement.zs
            
            # Bardini pag.268 eq 11-33
            self.f_zr.wheel_load_z[i] = - np.array(max(self.param.cr[i]*(self.displacement.zr[i] - self.displacement.zs[i] + self.lr_stat[i]), 0))
            naota =np.array(self.param.cr[i]*(self.displacement.zr[i] - self.displacement.zs[i] + self.lr_stat[i]))
            
            
            print('self.displacement.zr[i]',self.displacement.zr[i])
            ' Replace with followig code to take antiroll bar in to account'
            # wheel_load_z[0] = - max(self.param.cr[0]*(self.displacement.zr[0]-self.displacement.zs[0]+lr_stat[0]) + F_stv , 0) # Bardini pag.268 eq 11-33
            # wheel_load_z[1] = - max(self.param.cr[1]*(self.displacement.zr[1]-self.displacement.zs[1]+lr_stat[1]) + F_sth, 0)
            # wheel_load_z[2] = - max(self.param.cr[2]*(self.displacement.zr[2]-self.displacement.zs[2]+lr_stat[2]) - F_stv, 0)
            # wheel_load_z[3] = - max(self.param.cr[3]*(self.displacement.zr[3]-self.displacement.zs[3]+lr_stat[3]) - F_sth, 0)
            # print('self.x_rf.wheel_forces_transformed_force2vehicle_sys[i]')
            # print(self.x_rf.wheel_forces_transformed_force2vehicle_sys[i],self.x_rf.wheel_forces_transformed_force2vehicle_sys[i].shape)
        # print(' wheel_vx[0]', wheel_vx[0])
        return self.slip_x, self.slip_y
     
        
      
            # Compile wheel forces xyz on one vector  and tranform  to Vechicle coordinate
        #print('compiled_wheel f')
            
            # print('self.wheel_load_z[i]')
            # print(self.wheel_load_z[i],type(self.wheel_load_z[i]),self.wheel_load_z[i].shape)
            # print('compiled_wheel f[i]')
            # print(compiled_wheel_forces[i],type(compiled_wheel_forces[i]),compiled_wheel_forces[i].shape)
            # print('vtr_front')
            # print(self.VTR_front_axel,type(self.VTR_front_axel),self.VTR_front_axel.shape)
            
            
            # print('reshape_compiled_wheel_forces',reshape_compiled_wheel_forces,reshape_compiled_wheel_forces.shape)
            
    def tire_model(self): # fz, slip
        
        for i in range(4):
            self.x_rf.fx[i] = self.f_zr.wheel_load_z[i] * self.param.d * np.sin(self.param.c * np.arctan(self.param.b*self.slip_x - self.param.e * (self.param.b*self.slip_x - np.arctan(self.param.b *self.slip_x))))
            self.x_rf.fy[i] = self.f_zr.wheel_load_z[i] * self.param.d * np.sin(self.param.c * np.arctan(self.param.b*self.slip_y - self.param.e * (self.param.b*self.slip_y - np.arctan(self.param.b *self.slip_y))))
            # print('self.x_rf.fx[',i,']',self.x_rf.fx[i])
            # print('self.wheel_load_z[i]',self.f_zr.wheel_load_z[i])
            
        
            self.compiled_wheel_forces[i] = np.array([self.x_rf.fx[i], self.x_rf.fy[i], self.f_zr.wheel_load_z[i]])
            # print('self.compiled_wheel_forces',self.compiled_wheel_forces,type(self.compiled_wheel_forces),self.compiled_wheel_forces.shape)
            reshape_compiled_wheel_forces = np.reshape(self.compiled_wheel_forces[i],(3))
            self.compiled_wheel_forces[i] = np.array([self.x_rf.fx[i], self.x_rf.fy[i], self.f_zr.wheel_load_z[i]]).flatten()

            if i % 2 == 0:
                #TODO: check matrix operation 3x3 3x4>> define wheel_forces_transfomed matrix
                self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] =  np.matmul(self.VTR_front_axel,reshape_compiled_wheel_forces) # Bardini pag. 267 eq.32
            
            else: 
                self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] =  np.matmul(self.VTR_rear_axel,reshape_compiled_wheel_forces)
            #self.sum_Compiled_wheel_forces[[sum(self.x_rf.fx)], [sum(self.x_rf.fy)][sum(self.f_zr.wheel_load_z)]]
            # print('wheel_forces_transformed_force2vehicle_sys[i]',self.x_rf.wheel_forces_transformed_force2vehicle_sys[i])
            # print('VTR_front_axel',self.VTR_front_axel)
            # print('reshape_compiled_wheel_forces',reshape_compiled_wheel_forces)
    
    def suspension(self): # previsious forces, ax,ay 
        # za - displacement on chassis to suspension link
        # zr - displacement on wheel hub
        # Forces on the vehicle chassis at the pivot points Ai
        for i in range(4):    
        # Bardini pag. 265 eq. 11-21  
            # print('f_za.spring_force[i]')
            # print(self.f_za.spring_force,self.f_za.spring_force[i].shape, type(self.f_za.spring_force[i]))
            # print('spring_stiff[i]')
            # print(self.param.spring_stiff[i])
            # print('za[i]')
            # print(self.displacement.za,self.displacement.za.shape,type(self.displacement.za))
            
            self.f_za.spring_force[i] = (self.param.spring_stiff[i] * ((self.displacement.za[i]-self.displacement.zr[i]) + (
                self.displacement.l_stat[i]))) *  np.matmul(self.transpose_vehicle_fixed2inertial_system, np.array([0.,0.,1]))[0] 
        # Bardini pag. 266 eq. 11-22
            self.f_za.dumper_force[i] = (self.param.dumper[i] * (self.displacement.za_dot[i] - self.displacement.zr_dot[i]
                                                            ) + self.param.dumper[i]) * np.matmul(self.transpose_vehicle_fixed2inertial_system, np.array([[0],[0],[1]]))[0]
            self.f_za.f_za[i] = self.f_za.spring_force[i] + self.f_za.dumper_force[i]         
            # forces on the vehicle chassis (Ai) >> Bardini pag 236  >> horizontal forces pag 264 self.f_za.f_za
            self.strut2chassi_xyz[i] = [self.x_rf.fx[i], self.x_rf.fy[i], self.f_za.f_za[i]] # Fi , may be transpose 
            
        # V_F_fi - Forces on tires action on the chassis at the pivot points Ai of the four wheel >> Bardini pag 263
        self.strut2chassi = [[self.f_za.f_za[0]],
                             [self.f_za.f_za[1]],
                             [self.f_za.f_za[2]],
                             [self.f_za.f_za[3]]]
        
        #TODO transformation fx,fy (where? wheel)
        
        
        # TODO: where is used >> Transformation of positions 
        # print('self.chassi_converted_force[i]',self.chassi_converted_force[i],self.chassi_converted_force[i],shape)
        # print('self.position_chassi_force',self.position_chassi_force,self.position_chassi_force.shape)
        # print('self.strut2chassi_xyz[i]',self.strut2chassi_xyz[i],self.strut2chassi_xyz[i].shape)
        
        # chassi_converted_force[i] = np.cross(self.position_chassi_force,self.strut2chassi_xyz[i]) # check calculus: f.za is z direction
        #TODO: check transpose_vehicle_fixed2inertial_system operation  
    
    def  chassis(self): # vertical_loads, ax, ay, t_step
            "Equations of motion Bardini, pag 272 ---- need initialize values"
            position = np.array([[self.x_a.x],
                                 [self.x_a.y],
                                 [self.x_a.z]])
           
            self.velocity = np.array([[self.x_a.vx],
                                      [self.x_a.vy],
                                      [self.x_a.vz]])
            
            self.angular_rates = np.array([self.x_a.wx,
                                           self.x_a.wy,
                                           self.x_a.wz])
            self.angular_positions = np.array([[self.x_a.phi_v],
                                               [self.x_a.theta_v],
                                               [self.x_a.psi_v]])

            'sum of  wheel forces for calculating translation of the vehicle'
            
            self.sum_f_wheel = np.sum(self.x_rf.wheel_forces_transformed_force2vehicle_sys,axis=0)
            # self.sum_f_wheel[1] = np.sum(self.x_rf.wheel_forces_transformed_force2vehicle_sys,axis=0)[1]
            # self.sum_f_wheel[2] = np.sum(self.x_rf.wheel_forces_transformed_force2vehicle_sys,axis=0)[2]
            # print('wheel_forces_tr',self.x_rf.wheel_forces_transformed_force2vehicle_sys,self.x_rf.wheel_forces_transformed_force2vehicle_sys.shape)
            # print('============')
            
            # print('self.sum_f_wheel',self.sum_f_wheel,self.sum_f_wheel.shape,type(self.sum_f_wheel))
            # print('self.sum_f_wheel_0',self.sum_f_wheel[0],self.sum_f_wheel[0].shape,type(self.sum_f_wheel[0]))
            
            # Equation 11-46 >> 11-12, Pag. 273
            # TODO: check gavity diretion
            
            self.acc[0] = (self.sum_f_wheel[0]  + self.drag * self.x_a.vx **2
                           )/self.param.m + self.x_a.wz * self.x_a.vy - self.x_a.wy * self.x_a.vz
            #print('acc ', self.acc[0])
            self.acc[1] = (self.sum_f_wheel[1] - self.param.m * self.gravity + self.drag * self.x_a.vy **2
                           )/self.param.m + self.x_a.wx * self.x_a.vz - self.x_a.wz * self.x_a.vx
            self.acc[2] = (self.sum_f_wheel[2] - self.param.m * self.gravity
                           )/self.param.m + self.x_a.wy * self.x_a.vx - self.x_a.wx * self.x_a.vy
            
            # vehicle velocity calculation
            self.x_a.vx = self.x_a.vx + self.acc[0]* self.time_step 
            self.x_a.vy = self.x_a.vy + self.acc[1]* self.time_step 
            self.x_a.vz = self.x_a.vz + self.acc[2]* self.time_step
            # print('acel x=' , self.acc[0])
            # print('self.x_a.vx',self.x_a.vx)

            
            #TODO:Check rolling resistance            
            # self.rolling_resist = (self.param.fr * self.param.m * self.param.gravity * np.cos(0.) - 0.5 * self.param.row * self.param.Cl * self.param.area * self.speed ** 2)                              # Rolling Resistance with air lift
                
            '''Equation 11-46, vehicle velocity xyz calculation: for angular velocities and postion the last step is used
                Values calculate for positions, atittude and its derivates needs to be saved
            '''
            # aero_dyn_center = [[self.param.cg_x],[self.param.cg_y][self.param.cg_height]] # Shoul be center of aero pressure
            for i in range(4):
                
                self.crossproduct_r_f[i][0] = self.position_chassi_force[i][1] * self.strut2chassi_xyz[i][2] -self.position_chassi_force[i][2] * self.strut2chassi_xyz[i][1] 
                self.crossproduct_r_f[i][1] = self.position_chassi_force[i][2] * self.strut2chassi_xyz[i][0] -self.position_chassi_force[i][0] * self.strut2chassi_xyz[i][2]
                self.crossproduct_r_f[i][2] = self.position_chassi_force[i][0] * self.strut2chassi_xyz[i][1] -self.position_chassi_force[i][1] * self.strut2chassi_xyz[i][0]
        
            # TODO Check sum()
            # print('self.angular_rates',self.angular_rates,self.angular_rates.shape)
            # print('self.polar_inertia_v',self.polar_inertia_v,self.polar_inertia_v.shape)
            # print(' ')
            
            # eq 11 - 47
            self.sum_crossproduct_r_f = np.sum(self.crossproduct_r_f,axis=0)        
            self.acc_angular_v = self.sum_crossproduct_r_f - np.cross(self.angular_rates,(np.matmul(self.polar_inertia_v,self.angular_rates)))  
            #TODO: add multiplication of tranpose polar inertia to eq above>>   * self.polar_inertia_v.T)
            
            
            # check inverse operation of multiplication

            # Angular velocity of the chassis
            self.wx = self.x_a.wx + self.acc_angular_v[0] * self.time_step
            self.wy = self.x_a.wy + self.acc_angular_v[1] * self.time_step
            self.wz = self.x_a.wz + self.acc_angular_v[2] * self.time_step
              
            # Angular position
            self.x_a.phi_v =    self.x_a.wx * self.time_step + self.x_a.phi_v
            self.x_a.theta_v =  self.x_a.wy * self.time_step + self.x_a.theta_v
            self.x_a.psi_v =    self.x_a.wz * self.time_step + self.x_a.psi_v
            
            # bardini pag 260 -- use vector of torque x euler angle rate
            
            # vehicle position calculation
            movement_vehicle =  (self.x_a.vx * self.time_step, self.x_a.vy * self.time_step, self.x_a.vz * self.time_step) 
                        
           
            #TODO check mat mul ordem
            [self.x_a.x,self.x_a.y,self.x_a.z] = [self.x_a.x,self.x_a.y,self.x_a.z] + np.matmul(movement_vehicle,self.vehicle_fixed2inertial_system)
              
            # TODO new displacements need taking euler angles into account
            
            return 
    
    def tick(self, gas_pedal, brake, steering,time):
        self.powertrain(gas_pedal, brake, self.param.rpm_table, self.param.torque_max) # 
        self.steering()
        self.wheel_fix_coord_sys()
        self.road()
        self.newton_euler_wheel()
        self.wheel(steering,time)
        self.tire_model()
        self.suspension()
        self.chassis() 
        
        return [self.x_a.x,self.x_a.y,self.x_a.z,self.x_a.phi_v,self.x_a.theta_v,self.x_a.psi_v,self.x_a.vx,self.x_a.vy,self.x_a.vz,self.x_a.wx,self.x_a.wy,self.x_a.wz,self.x_a.acc[0],self.x_a.acc[1],self.x_a.acc[2]]
        
################################################################################################################################        

class StateVector(object):
    def __init__(self, x=0., y=0., z=0., phi_v=0., theta_v=0., psi_v=0., vx=0., vy=0., vz=0.,wx=0., wy=0., wz=0., acc = [0.,0.,0.]):
        
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
    def __init__(self, f_za=np.array([0,0,0,0]), f_za_dot=np.array([0,0,0,0]), spring_force = np.array([0,0,0,0]),dumper_force = np.array([0.,0.,0.,0.])): #TODO calculate forces on the strut
        self.f_za = f_za
        self.f_za_dot = f_za_dot
        self.dumper_force = dumper_force 
        self.spring_force = spring_force 
        
class WheelHubForce(object):
    def __init__(self, f_zr=[0,0,0,0], f_zr_dot=[0,0,0,0],wheel_load_z =[200.,300.,200.,300.] ):
        self.f_zr = f_zr
        self.f_zr_dot =f_zr_dot
        self.wheel_load_z = wheel_load_z
        
        
class AngularWheelPosition(object):
    def __init__(self, pho_r=[0,0,0,0], pho_r_dot = [0,0,0,0],pho_r_2dot = [0,0,0,0]):
    
        # pag 272 eq.52
        # Angular position/speed of each wheel
        self.pho_r = pho_r
        self.pho_r_dot = pho_r_dot
        self.pho_r_2dot = pho_r_2dot
        
class TireForces(object):
    def __init__ (self, fx =np.array([0.,0.,0.,0.]),fy =np.array([0.,0.,0.,0.]),wheel_forces_transformed_force2vehicle_sys = np.zeros((4,3),dtype=float)):        
        # Dynamic forces on the tires
        # pag 272 eq.54
        self.fx = fx
        self.fy = fy
        self.wheel_forces_transformed_force2vehicle_sys = wheel_forces_transformed_force2vehicle_sys 
    
class Displacement(object):
    def __init__ (self, l_stat= np.array([0.,0.,0.,0.]), za=np.array([0.,0.,0.,0.]), zr=[0,0,0,0], za_dot=[0.,0.,0.,0.], zr_dot=[0,0,0,0],zr_2dot =[0,0,0,0],zs =[0,0,0,0]):
        self.l_stat = l_stat
        self.za = za
        self.za_dot = za_dot
        self.zr_2dot = zr_2dot
        self.zr_dot = zr_dot
        self.zr = zr
        self.zs = zs
        # Vertical displacements/velocity of the wheel *(x_rz)
        # pag 272 eq.53
        
class ImportParam(object):

    def __init__(self, path = 'config.yaml'):
        
        with open(path, 'r') as file:
          param = yaml.safe_load(file)
    
        
        #=====================================
        # Engine Coeficients
        #=====================================
        self.min_rpm = param['vehicle_model']['parameters']['min_rpm'] 
        self.rpm_table = np.array(param['vehicle_model']['parameters']['rpm_table'])
        self.torque_max = np.array(param['vehicle_model']['parameters']['torque_max'])
        self.gear_ratio = np.array(param['vehicle_model']['parameters']['gear_ratio'])
        self.gear_selection = np.array(param['vehicle_model']['parameters']['gear_selection'])
        self.diff = param['vehicle_model']['parameters']['diff']                       # Differential ratio
        self.diff_ni = param['vehicle_model']['parameters']['diff_ni']
        self.transmition_ni = param['vehicle_model']['parameters']['transmition_ni']
        self.b_bias = param['vehicle_model']['parameters']['b_bias']
        
        #=====================================
        # Powertrain parameters
        #=====================================
        self.speed_ratio_TC = np.array(param['vehicle_model']['parameters']['speed_ratio_TC'])
        self.torque_converter_ratio = np.array(param['vehicle_model']['parameters']['torque_converter_ratio'])
      
        #=====================================
        # Tire Data
        #=====================================
        # self.cx = param['vehicle_model']['parameters']['c_tire_x']                                    # Tire Stiffiness [N/m]
        #self.cy = param['vehicle_model']['parameters']['c_tire_y']                                     # Tire Stiffiness [N/m]
        #self.F_max = param['vehicle_model']['parameters']['f_max']                                     # Tire max load
        #self.bx = np.array(param['vehicle_model']['parameters']['bx'])                                           # tire coeficient Bardini pag 268
        #self.by = np.array(param['vehicle_model']['parameters']['by'])
        self.cr = np.array(param['vehicle_model']['parameters']['cr'])                                  # Tire vertical Stiffnes
        self.i_wheel = param['vehicle_model']['parameters']['i_wheel']                                  # sum of all 4 wheel inertia
        self.wheel_mass = np.array(param['vehicle_model']['parameters']['wheel_mass'])
        self.r_stat = param['vehicle_model']['parameters']['r_stat']                                    # Tire Radius Static [m]
        self.r_dyn  = np.array(param['vehicle_model']['parameters']['r_dyn'])    
        # Rolling Resistance Parameters
        
        self.d = param['vehicle_model']['parameters']['magic_formula_d']                                    # Magic formula coefficient
        self.e = param['vehicle_model']['parameters']['magic_formula_e']
        self.c = param['vehicle_model']['parameters']['magic_formula_c']
        self.b = param['vehicle_model']['parameters']['magic_formula_b']
        #=====================================
        # Resistance Parameters
        #=====================================
        self.Cd = param['vehicle_model']['parameters']['cd']                                                 # Air Drag Coefficient
        self.Front_area = param['vehicle_model']['parameters']['front_area']  ###                            # Front Area of Car
        self.Air_density = param['vehicle_model']['parameters']['air_density']                               # Air Dencity 
        self.row = param['vehicle_model']['parameters']['row']
        
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
        self.iw = param['vehicle_model']['parameters']['iw']                                    # Wheel inertia [Kgm^2]
        self.ia = param['vehicle_model']['parameters']['ia']                                    # Axel inertia [Kgm^2]
        self.id = param['vehicle_model']['parameters']['id']                                    # drive shaft inertia [Kgm^2]
        self.ig = param['vehicle_model']['parameters']['ig']            # gear box inertia
        self.engine_inertia = param['vehicle_model']['parameters']['engine_inertia']                              # Engine inertia [Kgm^2]
        self.max_brake_torque = param['vehicle_model']['parameters']['max_brake_torque']  
        #self.i_uf = param['vehicle_model']['parameters']['i_uf']        # moment of inertia for unsprung mass about x-axis (front) [kg m^2]  IXUF
        #self.i_ur = param['vehicle_model']['parameters']['i_ur']        # moment of inertia for unsprung mass about x-axis (rear) [kg m^2]  IXUR
        #self.i_y_w = param['vehicle_model']['parameters']['i_y_w']      # wheel inertia, from internet forum for 235/65 R 17 [kg m^2]
        self.i_x_s = param['vehicle_model']['parameters']['i_phi_s']      # moment of inertia for sprung mass in roll [kg m^2]  iXS
        self.i_y_s =  param['vehicle_model']['parameters']['i_y_s']     # moment of inertia for sprung mass in pitch [kg m^2]  iYS
        self.i_z = param['vehicle_model']['parameters']['i_z']          # moment of inertia for sprung mass in yaw [kg m^2]  iZZ
        #self.i_xz_s = param['vehicle_model']['parameters']['i_xz_s']    # moment of inertia cross product [kg m^2]  iXZ
        self.i_d_shaft = param['vehicle_model']['parameters']['i_d_shaft']                    # calculate
        self.i_clutch = param['vehicle_model']['parameters']['i_clutch']
  
        # suspension parameters
        self.spring_stiff = np.array(param['vehicle_model']['parameters']['spring_stiff'])      # suspension spring rate (front i = 1 or 3)  (rear = 1 = 2,4) [N/m]  KSF 
        self.dumper = np.array(param['vehicle_model']['parameters']['dumper'])              # suspension damping rate (front i = 1 or 3)  (rear = 1 = 2,4)[N s/m]  KSDF
        self.anti_roll_stiffness = param['vehicle_model']['parameters']['anti_roll_stiffness']  # Anti roll bar stiffness [Nm/rad]
    
        #=====================================
        # geometric parameters
        #=====================================
        
        self.cg_height = param['vehicle_model']['parameters']['cg_height']                   # center of gravity height of total mass [m]
        self.cg_x = param['vehicle_model']['parameters']['cg_x']                             #  cg_x [m]
        self.cg_y = param['vehicle_model']['parameters']['cg_y']                             # [m]
        self.wheel_base = param['vehicle_model']['parameters']['wheel_base']                 # [m]
        # axes distances
        self.lv = param['vehicle_model']['parameters']['lv']                                  # x-distance from Vehicle CoG to the front hub [m]
        self.lh = param['vehicle_model']['parameters']['lh']                                  # x-distance from Vehicle Cog to the rear hub [m]
        # Half track
        self.sl = param['vehicle_model']['parameters']['sl']                                  # y-distance from Vehicle Cog to the rear hub [m]
        self.sr = param['vehicle_model']['parameters']['sr']                                  # y-distance from Vehicle Cog to the rear hub [m]
        self.sz = param['vehicle_model']['parameters']['sz']                                  # z-distance from Vehicle Cog to the rear hub [m]
        self.track_width = param['vehicle_model']['parameters']['track_width']                # vehicle width [m]
        self.hub_fl= param['vehicle_model']['parameters']['hub_fl'] 
        self.hub_rl= param['vehicle_model']['parameters']['hub_rl'] 
        self.hub_fr= param['vehicle_model']['parameters']['hub_fr'] 
        self.hub_rr= param['vehicle_model']['parameters']['hub_rr'] 
        