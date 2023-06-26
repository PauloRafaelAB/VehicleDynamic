
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

class VehicleDynamics(object):
    # ^class name  #^ inherits from object

    """ This class initialize the values of a vehicular dynamic model. 
    Calculate longitudinal and lateral dynamics with desired values 
    of brake, steer and thorttle positons.
    """

    def __init__(self, initial_speed = 0., state_0 = [0., 0., 0., 0., 0., 0.], initial_gear = 1, freq=100, param_path = ""):
    
        if param_path != "":
            self.param = ImportParam(param_path) # Import all the Parameters 
            
        self.time_step = 1/freq
        
        # Steering_angle [t-1]
        self.last_delta = 0
        # Steering_angle [t]
        self.delta = 0
        self.gravity = 9.81
        self.rpm = self.param.min_rpm
        self.gear = initial_gear                    # gear selector
        self.throttle = 0.0                         # thorttle position (0< throttle <1)
        self.brake = 0.0                            # Brake pedal position (0< brake <1)
        #self.alpha_engine = 0.0                     # Angular acc engine
        self.wheel_w_vel = np.zeros(4)
        self.torque_converter_ratio_inter = interp1d(self.param.speed_ratio_TC, self.param.torque_converter_ratio)
        self.drag = 0.5*self.param.row * self.param.Cd * self.param.Front_area  # constant for air resistance
                       
        # State initiate with the position, orientation and speed provided by the arguments, acc = 0; 
        self.x_a = StateVector(x=state_0[0],y=state_0[1],z=state_0[2], psi=state_0[3], theta=state_0[4], phi=state_0[5], vx =initial_speed, vy=0.,vz=0., wx=0., wy=0., wz=0.) # State_0
        
        #Wheel initiate stoped 
        self.x_rr = AngularWheelPosition(pho_r=np.zeros(4), pho_r_dot = np.zeros(4),pho_r_2dot =np.zeros(4))
        self.x_rf = TireForces(fx =np.zeros(4), fy = np.zeros(4),wheel_forces_transformed_force2vehicle_sys = np.zeros((4,3),dtype=float))
        self.displacement = Displacement(l_stat=(self.param.m*self.param.wd*self.gravity)/self.param.eq_stiff, za= np.zeros(4), za_dot=np.zeros(4), zr_dot=np.zeros(4),zr_2dot=np.zeros(4))
        self.f_za = StrutForce(f_za = self.param.m*self.gravity*self.param.wd, f_za_dot = np.zeros(4),spring_force = (self.param.m_s*self.gravity).reshape(-1,1), dumper_force = np.zeros((4,1),dtype=float))
        self.f_zr = WheelHubForce(f_zr_dot=np.array([0.,0.,0.,0.]), wheel_load_z = self.param.m*self.gravity*self.param.wd)
        self.wheel_hub_velocity = np.zeros((4,3))
        self.final_ratio = 1
        self.polar_inertia_v = np.array([[self.param.i_x_s, 0., 0.],
                                        [0., self.param.i_y_s, 0.],
                                        [0., 0., self.param.i_z]])
        
        #self.hub_distance = np.array([self.param.hub_fl, self.param.hub_rl,self.param.hub_fr,self.param.hub_rr])
        self.position_chassi_force = np.array([[self.param.lv,self.param.sl,-self.param.sz], [-self.param.lh,self.param.sl,-self.param.sz], [self.param.lv,-self.param.sr,-self.param.sz],[-self.param.lh,-self.param.sr,-self.param.sz]])
                
        self.polar_inertia_v = np.array([[self.param.i_x_s, 0., 0.],
                                        [0., self.param.i_y_s, 0.],
                                        [0., 0., self.param.i_z]])
        # Forces on the chassis
        self.strut2chassi_xyz = np.zeros((4,3),dtype=float)
        
        #Forces on the wheel
        self.compiled_wheel_forces = np.zeros((4,3),dtype= float)
        
        # Static displacement of the springs and tire
        # self.displacement.l_stat = self.param.m*self.param.wd *self.gravity / self.param.eq_stiff
       
        # Creating vector for chassis method 
        self.sum_f_wheel = np.zeros(3, dtype=float) # Sum of wheel forces
        #self.acc = np.zeros((3),dtype=float) # acceleration
        self.crossproduct_r_f = np.zeros((4,3), dtype=float)
        
        # Transformation matrix vehicle coord sys(Kv) into inertia sys(Ke) - E_T_V -- Bardini pag 260 eq. 11.3
        self.vehicle_fixed2inertial_system = np.array([[np.cos(self.x_a.theta) * np.cos(self.x_a.phi),          np.sin(self.x_a.psi) * np.sin(self.x_a.theta) * np.cos(self.x_a.phi) - np.cos(self.x_a.psi) * np.sin(self.x_a.phi),         np.cos(self.x_a.psi) * np.sin(self.x_a.theta) * np.cos(self.x_a.phi) + np.sin(self.x_a.psi) * np.sin(self.x_a.phi)],
                                              [np.cos(self.x_a.theta) * np.sin(self.x_a.phi),                   np.sin(self.x_a.psi) * np.sin(self.x_a.theta) * np.sin(self.x_a.phi) + np.cos(self.x_a.psi) * np.cos(self.x_a.phi),         np.cos(self.x_a.psi) * np.sin(self.x_a.theta) * np.sin(self.x_a.phi) - np.sin(self.x_a.psi) * np.cos(self.x_a.phi)],
                                              [-np.sin(self.x_a.theta),                                           np.sin(self.x_a.psi) * np.cos(self.x_a.theta),                                                                                             np.cos(self.x_a.psi) * np.cos(self.x_a.theta)]])     #  Bardini pag 260
        # transformation from Ke to Kv. 
        self.transpose_vehicle_fixed2inertial_system = np.transpose(self.vehicle_fixed2inertial_system)
        self.wheel_vel_fix_coord_sys = np.zeros((4,3),dtype=float)
        
        # wheel hub inital eq-27
        #self.wheel_hub_position = np.zeros((4,3))
        # for i in range(4): 
        #     self.wheel_hub_position[i] = self.position_chassi_force[i] + np.matmul(self.transpose_vehicle_fixed2inertial_system,np.array([0,0,self.displacement.l_stat[i]]))

      
    def powertrain(self, throttle, brake, rpm_table, torque_max_table):
        ###################################CALCULAR RPMMMMMMMMMMM
        # Updating RPM
        self.rpm = self.final_ratio * np.mean(self.wheel_w_vel)   # Wheel vx to engine RPM
        print("RPM    ",self.rpm)
        if self.rpm < 1200.:
            self.rpm = 1200.
        
        # Torque provided by the engine
        torque_interpolation = interp1d(rpm_table, torque_max_table)    # how much torque is available thoughout the rpm range 
        torque_available = torque_interpolation(self.rpm)               # Max torque available at rpm
        engine_torque = throttle * torque_available                     # find the torque delivered by te engine
        
        # WOULD_IT_NOT_LEAD_TO_JUMPING_GEARS_REQUIRES_IMPROVEMENT___________________________________________________________________      
        #-----------Tranmission ratio----------
            
        
        if self.x_a.vx > self.param.gear_selection[int(self.throttle*10)][self.gear]:
            self.gear = self.gear + 1
            if self.gear >= self.param.gear_ratio.size:
                self.gear = self.param.gear_ratio.size-1
        
        # Using an offset of upshift
        elif self.x_a.vx < 0.8*self.param.gear_selection[int(self.throttle*10)][self.gear]:
            self.gear = self.gear - 1
            if self.gear < 1:
                self.gear = 1
        
        '''for i in range(len(self.param.gear_ratio)): # checking number of gears
         
            if self.x_a.acc[0] >= 0.0:
                if self.x_a.vx > self.param.gear_selection[int(self.throttle*10)][i]:
                    if i >6:
                        i=6
                        self.gear = self.param.gear_ratio[i+1]
            # Using an offset of upshift
            if self.x_a.acc[0] <0:
                if  (self.x_a.vx * 0.8) < self.param.gear_selection[int(self.throttle*10)][i]:
                    self.gear = self.param.gear_ratio[i]
            
            ## Se o carro nao estiver com acc=0, accelerando nem desacelerando ele fica no neutro (Nunca vai acontecer)
            else:
                self.gear = self.param.gear_ratio[0]    # neutral   ''' 
        # _____________________________________________________________________________________________________________________________
        
        # self.wheel_rpm = 30 * self.x_a.vx/( self.param.r_dyn * np.pi) #RADSEC2RPM = 30/np.pi
        # replaced by self.wheel_w_vel
        
        # self.tcrpm = self.gear * self.wheel_w_vel[0] * self.torque_converter_ratio_inter
        # self.driveshaft_rpm = self.param.final_ratio * self.wheel_w_vel[0]
        # speed_ratio = self.driveshaft_rpm/self.rpm
        # if speed_ratio > 0.98:
        #     speed_ratio = 1.0
        # torque_convertion = self.torque_converter_ratio_inter(speed_ratio)
        
        # TODO: Engine torque to wheel, taking inercia into account (separated by gear)
        self.final_ratio = self.param.gear_ratio[self.gear] * self.param.diff # check final gear calculation
        
        print("Final ratio", self.final_ratio)
        print ("Engine Torque", engine_torque)
        #TODO: check inertias and formulation
        
        #### ENGINE TORQUE IS INCORRECT 
        traction_torque = (engine_torque * self.final_ratio * self.param.diff_ni * self.param.transmition_ni) 
                                                                #- ((self.param.engine_inertia + self.param.ia + self.param.ig) * self.gear ** 2 
                                                                  # + self.param.id * self.final_ratio ** 2 + self.param.i_wheel) * self.x_a.acc[0]
                                                               
                                                                      ### REMOVED THE INERTIA
                            
        #--------------------Break Torque -------------------------
        # using brake pedal as input, the brake torque can be calculated by 
        brake_torque = self.brake * self.param.max_brake_torque
        
        #-------------------- Net Torque -------------------------
        self.powertrain_net = traction_torque - brake_torque
        
        # function traction divided between axles
        self.powertrain_net_torque = np.empty(4, dtype = float)
        self.powertrain_net_torque[0] = self.powertrain_net*0.5* self.param.b_bias  
        self.powertrain_net_torque[1] = self.powertrain_net*0.5* (1-self.param.b_bias)
        self.powertrain_net_torque[2] = self.powertrain_net*0.5* self.param.b_bias
        self.powertrain_net_torque[3] = self.powertrain_net*0.5* (1-self.param.b_bias)
        
        
        #self.powetrain_force = self.powertrain_net_torque/self.param.r_dyn # NAO ESTÀ SENDO USADO EM LUGAR NENHUM 
        
        print ("Powertrain Torque", self.powertrain_net_torque)
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

   
    def steering(self, steering):
        
        
        self.delta = steering*self.param.steering_ratio 
        delta_dot = float((self.last_delta - self.delta)/self.time_step)
        # delta is steering angle of the wheel 
        #______________________________________________________________________________________________________________________________
        
        if (self.delta <= self.param.steering_min and delta_dot <= 0) or (self.delta >= self.param.steering_max and delta_dot >= 0):
            delta_dot = 0
        elif delta_dot <= self.param.steering_v_min:
            delta_dot = self.param.steering_v_min
        elif delta_dot >= self.param.steering_v_max:
            delta_dot = self.param.steering_v_max
        
        self.last_delta = self.delta
        
        #Where are you including the steering bar ratio? 
        # Matrix E_T_R (wheel angles) is calculate at steering fuction      
        # Bardini pag 261 eq. 11-6 (TR1, TR3)
        self.wheel_angle_front =  [[np.cos(self.x_a.psi + self.delta),        -np.sin(self.x_a.psi + self.delta),      0],
                                   [np.sin(self.x_a.psi + self.delta),         np.cos(self.x_a.psi + self.delta),      0],
                                   [                    0,                             0,      1]]
        
        # Eq.11-6    Schramm and Bardini Pag 261 (TR2, TR4)
        self.wheel_angle_rear =  [[np.cos(self.x_a.psi),     - np.sin(self.x_a.psi),       0],
                                  [np.sin(self.x_a.psi),       np.cos(self.x_a.psi),       0],
                                  [            0,                                     0,       1]] 
        
        # Wheel fixed coordinate(KR) rotation relativ to Kv(vehicle system) Bardni pag. 260 eq. 11-9
        self.VTR_front_axel = np.array([[                                                 np.cos(self.delta) * np.cos(self.x_a.theta),                                                -np.sin(self.delta) * np.cos(self.x_a.theta),                          -np.sin(self.x_a.theta)],
                                       [ np.sin(self.x_a.phi) * np.sin(self.x_a.theta) * np.cos(self.delta) + np.cos(self.x_a.phi) * np.sin(self.delta),     -np.sin(self.x_a.phi) * np.sin(self.x_a.theta) * np.sin(self.delta) + np.cos(self.x_a.phi) * np.cos(self.delta),            np.sin(self.x_a.phi)* np.cos(self.x_a.theta)],
                                       [ np.cos(self.x_a.phi) * np.sin(self.x_a.theta) * np.cos(self.delta) - np.sin(self.x_a.phi) * np.sin(self.delta),     -np.cos(self.x_a.phi) * np.sin(self.x_a.theta) * np.sin(self.delta) - np.sin(self.x_a.phi) * np.cos(self.delta),            np.cos(self.x_a.phi)* np.cos(self.x_a.theta)]])
        
        # Bardni. Pag. 260 eq. 11-10
        self.VTR_rear_axel = np.array([[                  np.cos(self.x_a.theta),                                  0,                                   -np.sin(self.x_a.theta)],
                                      [ np.sin(self.x_a.phi) * np.sin(self.x_a.theta),     np.cos(self.x_a.phi),            np.sin(self.x_a.phi)* np.cos(self.x_a.theta)],
                                      [ np.cos(self.x_a.phi) * np.sin(self.x_a.theta),   - np.sin(self.x_a.phi),            np.cos(self.x_a.phi)* np.cos(self.x_a.theta)]])
       
        "says rotational transformation to Kv is necessary (VTR_front_axel)>> def vehiclefixed2inertial_system"

        #return self.wheel_angle_front, self.wheel_angle_rear
        
        
    def wheel_slip(self): #input: fz, mi, wheel_angle,vx,vy   output:fz,slix,slip_y
        '''initial states of tire load are mandatory [Wheel torque], [Wheel_Fz], slip_x, slip_angle]
            calculate the forces..
        '''
        # wheel states
        # initialize angular velocity of each wheel (relative to vehicle system)
        for i in range(4):
            
            # Slip calculation - Wheel slip is calculated by wheel fixed system
            self.slip_x = (((self.param.r_dyn[i] * self.wheel_w_vel[i] - self.x_a.vx )/ max([self.param.r_dyn[i] * self.wheel_w_vel[i]+1e-16, 1e-16+self.x_a.vx])))         # equation 11.30 Bardini
            ##### REMOVED THE ABS()
            print('self.wheel_w_vel[i]',self.wheel_w_vel[i])
            print(self.x_a.vx,'vx')
            
            # Lateral slip: define wheel_vy Equacao 11_28 >> converte to tire direction
            self.slip_y = - np.arctan( self.x_a.vy / max([self.param.r_dyn[i] * self.wheel_w_vel[i]+1e-16, 1e-16 + self.x_a.vx])) # equation 11.31 Bardini
        
            print ("Slip X ", self.slip_x, "  Slip Y ", self.slip_y)
            # _______________________________________________________________________________________________
            # TODO: Check first argument by displaying (velocity in wheel fixed coord) Bardini eq 11-29
            
            # Bardini pag.268 eq 11-33
            ' Replace with followig code to take antiroll bar in to account'
            # TODO: Make F_stv diferent than 0; make F_st input on tire fuction
            # wheel_load_z[0] = - max(self.param.cr[0]*(self.displacement.zr[0]-self.displacement.zs[0]+lr_stat[0]) + F_stv , 0) # Bardini pag.268 eq 11-33
    
    
    def tire_model(self): # fz, slip
    
    
    ######## X IS BIGGER THAN Z ?????????????
    
    ## Input slip, fz - fx_car, fy_car
        print('wheel_load_z',self.f_zr.wheel_load_z)
        self.x_rf.fx = self.f_zr.wheel_load_z * self.param.d * np.sin(self.param.c * np.arctan(self.param.b*self.slip_x - self.param.e * (self.param.b*self.slip_x - np.arctan(self.param.b *self.slip_x))))
        self.x_rf.fy = self.f_zr.wheel_load_z * self.param.d * np.sin(self.param.c * np.arctan(self.param.b*self.slip_y - self.param.e * (self.param.b*self.slip_y - np.arctan(self.param.b *self.slip_y))))
        
        for i in range(4):
            #whell load é negativo, mas fx deve ser positivo
            #self.x_rf.fx[i] = self.f_zr.wheel_load_z[i] * self.param.d * np.sin(self.param.c * np.arctan(self.param.b*self.slip_x - self.param.e * (self.param.b*self.slip_x - np.arctan(self.param.b *self.slip_x))))
            #self.x_rf.fy[i] = self.f_zr.wheel_load_z[i] * self.param.d * np.sin(self.param.c * np.arctan(self.param.b*self.slip_y - self.param.e * (self.param.b*self.slip_y - np.arctan(self.param.b *self.slip_y))))
      
        
            self.compiled_wheel_forces[i] = np.array([self.x_rf.fx[i], self.x_rf.fy[i], self.f_zr.wheel_load_z[i]])
            # print('self.compiled_wheel_forces',self.compiled_wheel_forces,type(self.compiled_wheel_forces),self.compiled_wheel_forces.shape)
            '''reshape_compiled_wheel_forces = np.reshape(self.compiled_wheel_forces[i],(3))

            if i % 2 == 0:
                #TODO: check matrix operation 3x3 3x4>> define wheel_forces_transfomed matrix
                self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] =  np.matmul(self.VTR_front_axel,reshape_compiled_wheel_forces) # Bardini pag. 267 eq.32
            
            else: 
                self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] =  np.matmul(self.VTR_rear_axel,reshape_compiled_wheel_forces)
            
            # forces on the vehicle chassis (Ai) >> Bardini pag 236  >> horizontal forces pag 264 self.f_za.f_za
            self.strut2chassi_xyz[i] = [self.x_rf.fx[i], self.x_rf.fy[i], self.f_za.f_za[i]] # Fi , may be transpose '''
        
        print("Compiled wheel forces ", self.compiled_wheel_forces)
        
        
        
    
    def Wheel_angular(self):
        ''' wheel velocities are used to calcule slip, than when can futher calculate ax,ay using the forces 
        using the magic formula
        '''
        # EULER’s Equations of the Wheels: Solve  11-25 Bardini pag 266
        ## Torque, fx_car, fy_car, Inertias wheel,  output wheel Wv, dot_wv
        
        self.x_rr.pho_r_2dot  =  (self.powertrain_net_torque  - self.param.r_dyn  * self.x_rf.fx ) /self.param.iw
        print(self.x_rr.pho_r_2dot,"pho_r_2dot")
        print("FX_________",self.x_rf.fx)
        
        # Calculate the intregral of the wheel_acc to calculate slip_x
        self.wheel_w_vel  = self.wheel_w_vel  + self.x_rr.pho_r_2dot * self.time_step # rad/s
        #___________________________________________________________________________________________________________
        

    def rotationalmatrix(self): ## ONDE ESTÁ ATUALIZANDO OS ANGULOS?-------------------------------------------------------------
        " "
        # For the angular velocity of the chassis we have:   Bardini Pag. 261 Eq. 11.5
        rotationalmatrix = [[                          -np.sin(self.x_a.theta),                        0,             1],
                            [ np.cos(self.x_a.theta) * np.sin(self.x_a.phi),    np.cos(self.x_a.phi),             0],
                            [ np.cos(self.x_a.theta) * np.cos(self.x_a.phi),  - np.sin(self.x_a.phi),             0]]
        
        # phi_dot,theta_dot, psi_dot -- Bardini Pag. 261 Eq. 11.4
        self.chassis_w_velocity = [[ self.x_a.wx],
                                   [ self.x_a.wy],
                                   [ self.x_a.wz]]
                              
        # Coordinate representation of the absolute velocity of point v with respect to coordinate system “E”, described in coordinates of coordinate system “v” bardni. Pag 260
        self.angular_vel_2inercial_sys_in_vehicle_coord = np.dot(rotationalmatrix, self.chassis_w_velocity) #Bardini Pag. 261 Eq. 11.4    
        return self.angular_vel_2inercial_sys_in_vehicle_coord, self.rotationalmatrix


    def access_z_road(x,y):
       #TODO implement topography
        z = 0.
        return z
    
    def road(self):
        for k in range(4):
        #     self.displacement.zs[k][2] = self.access_z_road(self.displacement.zs[k][0],self.displacement.zs[k][1])
            self.displacement.zs[k]=0.
        
     
    def suspension(self): # previsious forces, ax,ay 
        # Forces on the vehicle chassis at the pivot points Ai
        # Bardini pag. 265 eq. 11-21  
        # TODO adicionar zs dot
        self.displacement.za = np.array([0,0,0,0]) # CONSIDERANDO QUE NAO HÁ TRANSFERENCIA DE CARGA
        self.f_zr.wheel_load_z = (self.param.eq_stiff*(-self.displacement.za + self.displacement.zs + self.displacement.l_stat
                                                        ) + self.param.dumper * (self.displacement.za_dot-self.displacement.za_dot)) * np.matmul(self.transpose_vehicle_fixed2inertial_system, np.array([[0],[0],[1]]))[2]
        
        ### NO MEU PONTO DE VISTA AQUI VOCE CALCULARIA COMO AS FORCAS QUE AGEM NO CG ATUAM NOS PO
        print ("displacement.za",self.displacement.za)
        print ("whell load z", self.f_zr.wheel_load_z)
        # V_F_fi - Forces on tires action on the chassis at the pivot points Ai of the four wheel >> Bardini pag 263
        
        #TODO: check transpose_vehicle_fixed2inertial_system operation  
    
    def  chassis(self): # vertical_loads, ax, ay, t_step
            "Equations of motion Bardini, pag 272 ---- need initialize values"
            
            self.angular_rates = np.array([self.x_a.wx,
                                           self.x_a.wy,
                                           self.x_a.wz])
            
            'sum of  wheel forces for calculating translation of the vehicle'
            
            self.sum_f_wheel = np.sum(self.x_rf.wheel_forces_transformed_force2vehicle_sys,axis=0)
            
            # self.sum_f_wheel[1] = np.sum(self.x_rf.wheel_forces_transformed_force2vehicle_sys,axis=0)[1]
            # self.sum_f_wheel[2] = np.sum(self.x_rf.wheel_forces_transformed_force2vehicle_sys,axis=0)[2]
            # Equation 11-46 >> 11-12, Pag. 273
            # TODO: check gavity diretion
            
            self.x_a.acc[0] = (self.sum_f_wheel[0]  - self.drag * self.x_a.vx **2
                           )/self.param.m + self.x_a.wz * self.x_a.vy - self.x_a.wy * self.x_a.vz
            self.x_a.acc[1] = (self.sum_f_wheel[1] -self.x_a.vy **2
                           )/self.param.m + self.x_a.wx * self.x_a.vz - self.x_a.wz * self.x_a.vx
            self.x_a.acc[2] = (self.sum_f_wheel[2] - self.param.m * self.gravity
                           )/self.param.m + self.x_a.wy * self.x_a.vx - self.x_a.wx * self.x_a.vy
            
            # vehicle velocity calculation
            self.x_a.vx = self.x_a.vx + self.x_a.acc[0]* self.time_step 
            self.x_a.vy = self.x_a.vy + self.x_a.acc[1]* self.time_step 
            self.x_a.vz = self.x_a.vz + self.x_a.acc[2]* self.time_step
            
            
            
            #TODO:Check rolling resistance            
            # self.rolling_resist = (self.param.fr * self.param.m * self.param.gravity * np.cos(0.) - 0.5 * self.param.row * self.param.Cl * self.param.area * self.speed ** 2)                              # Rolling Resistance with air lift
                
            '''Equation 11-46, vehicle velocity xyz calculation: for angular velocities and postion the last step is used
                Values calculate for positions, atittude and its derivates needs to be saved
            '''
            for i in range(4):
                
                self.crossproduct_r_f[i][0] = self.position_chassi_force[i][1] * self.strut2chassi_xyz[i][2] -self.position_chassi_force[i][2] * self.strut2chassi_xyz[i][1] 
                self.crossproduct_r_f[i][1] = self.position_chassi_force[i][2] * self.strut2chassi_xyz[i][0] -self.position_chassi_force[i][0] * self.strut2chassi_xyz[i][2]
                self.crossproduct_r_f[i][2] = self.position_chassi_force[i][0] * self.strut2chassi_xyz[i][1] -self.position_chassi_force[i][1] * self.strut2chassi_xyz[i][0]

            
            # TODO Check eq 11 - 47
            self.sum_crossproduct_r_f = np.sum(self.crossproduct_r_f,axis=0)
            # print('sum_crossproduct',self.sum_crossproduct_r_f)
            #TODO make the return of acc angular be type (3,0)
            
            self.x_a.acc_angular_v = np.matmul((self.sum_crossproduct_r_f - np.cross(self.angular_rates,(np.matmul(self.polar_inertia_v,self.angular_rates)))),inv(self.polar_inertia_v))
            
            
            print('sum_crossproduct',self.sum_crossproduct_r_f)
            # print('angular_rates', self.angular_rates)
            # print('polar inertia',self.polar_inertia_v)
            
            # print('self.x_a.acc_angular_v',self.x_a.acc_angular_v,type(self.x_a.acc_angular_v))
            #TODO: add multiplication of tranpose polar inertia to eq above>>   * self.polar_inertia_v.T)
            
            # Angular velocity of the chassis
            self.x_a.wx = self.x_a.wx + self.x_a.acc_angular_v[0] * self.time_step
            self.x_a.wy = self.x_a.wy + self.x_a.acc_angular_v[1] * self.time_step
        
            self.x_a.wz = self.x_a.wz + self.x_a.acc_angular_v[2] * self.time_step
        
            # Angular position
            self.x_a.psi =    self.x_a.wx * self.time_step + self.x_a.psi
            self.x_a.theta =  self.x_a.wy * self.time_step + self.x_a.theta
            self.x_a.phi =    self.x_a.wz * self.time_step + self.x_a.phi
            
            #TODO: updated transformation to vehicle system
            
            # self.vehicle_fixed2inertial_system = np.array([[np.cos(self.x_a.theta) * np.cos(self.x_a.phi), np.sin(self.x_a.psi) * np.sin(self.x_a.theta) * np.cos(self.x_a.phi) - np.cos(self.x_a.psi) * np.sin(self.x_a.phi),     np.cos(self.x_a.psi) * np.sin(self.x_a.theta) * np.cos(self.x_a.phi) + np.sin(self.x_a.psi) * np.sin(self.x_a.phi)],
            #                                       [np.cos(self.x_a.theta) * np.sin(self.x_a.phi), np.sin(self.x_a.psi) * np.sin(self.x_a.theta) * np.sin(self.x_a.phi) + np.cos(self.x_a.psi) * np.sin(self.x_a.phi),     np.cos(self.x_a.psi) * np.sin(self.x_a.theta) * np.sin(self.x_a.phi) - np.sin(self.x_a.psi) * np.cos(self.x_a.phi)],
            #                                       [-np.sin(self.x_a.theta),                         np.sin(self.x_a.psi) * np.cos(self.x_a.theta),                                                      np.cos(self.x_a.psi) * np.cos(self.x_a.theta)]])
            
            # bardini pag 260 -- use vector of torque x euler angle rate
            
            # vehicle position calculation
            movement_vehicle =  (self.x_a.vx * self.time_step, self.x_a.vy * self.time_step, self.x_a.vz * self.time_step) 
            
            #TODO check mat mul ordem
            [self.x_a.x,self.x_a.y,self.x_a.z] = [self.x_a.x,self.x_a.y,self.x_a.z] + np.matmul(movement_vehicle, self.vehicle_fixed2inertial_system) 
            print("x",self.x_a.x,"y",self.x_a.y,"z",self.x_a.z)
            
            # TODO new displacements need taking euler angles into account

            self.displacement.za[0]= self.x_a.z - self.param.lv* np.sin(self.x_a.theta)+ self.param.sl*np.sin(self.x_a.phi)
            self.displacement.za[1]= self.x_a.z + self.param.lh* np.sin(self.x_a.theta)+ self.param.sl*np.sin(self.x_a.phi)
            self.displacement.za[2]= self.x_a.z - self.param.lv* np.sin(self.x_a.theta)- self.param.sr*np.sin(self.x_a.phi)
            self.displacement.za[3]= self.x_a.z + self.param.lh* np.sin(self.x_a.theta)- self.param.sr*np.sin(self.x_a.phi)
            
            return 
    
    def tick(self, gas_pedal, brake, steering, time):
        self.powertrain(gas_pedal, brake, self.param.rpm_table, self.param.torque_max) # 
        self.steering(steering)
        self.wheel_slip()
        self.tire_model()
        self.Wheel_angular()
        self.road()
        self.suspension()
        self.chassis() 
        print('                      ')

        return [self.x_a.x,self.x_a.y,self.x_a.z,self.x_a.psi,self.x_a.theta,self.x_a.phi,self.x_a.vx,self.x_a.vy,self.x_a.vz,self.x_a.wx,self.x_a.wy,self.x_a.wz,self.x_a.acc[0],self.x_a.acc[1],self.x_a.acc[2],self.x_rf.fx[2]]  #
        
################################################################################################################################        

class StateVector(object):
    def __init__(self, x=0., y=0., z=0., psi=0., theta=0., phi=0., vx=0., vy=0., vz=0.,wx=0., wy=0., wz=0., acc = [0.,0.,0.]):
        
        # Position and Euler angles
        self.x = x
        self.y = y 
        self.z = z 
        self.psi = psi
        self.theta = theta
        self.phi = phi
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.wx = wx 
        self.wy= wy 
        self.wz = wz
        self.acc = acc
        
class StrutForce(object):
    def __init__(self, f_za=np.zeros(4), f_za_dot=np.zeros(4), spring_force = np.zeros(4),dumper_force = np.zeros(4)): #TODO calculate forces on the strut
        self.f_za = f_za
        self.f_za_dot = f_za_dot
        self.dumper_force = dumper_force 
        self.spring_force = spring_force 
        
class WheelHubForce(object):
    def __init__(self, f_zr_dot=np.zeros(4),wheel_load_z =np.array([2000.,3000.,2000.,3000.])):
        self.f_zr_dot =f_zr_dot
        self.wheel_load_z = wheel_load_z
        
        
class AngularWheelPosition(object):
    def __init__(self, pho_r=np.zeros(4), pho_r_dot = np.zeros(4),pho_r_2dot = np.zeros(4)):
    
        # pag 272 eq.52
        # Angular position/speed of each wheel
        self.pho_r = pho_r
        self.pho_r_dot = pho_r_dot
        self.pho_r_2dot = pho_r_2dot
        
class TireForces(object):
    def __init__ (self, fx =np.zeros(4),fy =np.zeros(4),wheel_forces_transformed_force2vehicle_sys = np.zeros((4,3),dtype=float)):        
        # Dynamic forces on the tires
        # pag 272 eq.54
        self.fx = fx
        self.fy = fy
        self.wheel_forces_transformed_force2vehicle_sys = wheel_forces_transformed_force2vehicle_sys 
    
class Displacement(object):
    def __init__ (self, l_stat= np.zeros(4), za=np.zeros(4), za_dot=np.zeros(4), zr_dot=np.zeros(4),zr_2dot = np.zeros(4),zs =np.zeros(4)):
        self.l_stat = l_stat
        self.za = za
        self.za_dot = za_dot
        self.zr_2dot = zr_2dot
        self.zr_dot = zr_dot
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
        #self.cr = np.array(param['vehicle_model']['parameters']['cr'])                                  # Tire vertical Stiffnes
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
        self.eq_stiff = np.array(param['vehicle_model']['parameters']['eq_stiff'])      # suspension spring rate (front i = 1 or 3)  (rear = 1 = 2,4) [N/m]  KSF 
        self.dumper = np.array(param['vehicle_model']['parameters']['dumper'])              # suspension damping rate (front i = 1 or 3)  (rear = 1 = 2,4)[N s/m]  KSDF
        self.anti_roll_stiffness = param['vehicle_model']['parameters']['anti_roll_stiffness']  # Anti roll bar stiffness [Nm/rad]
    
        #=====================================
        # geometric parameters
        #===================================== 
        # axes distances
        self.lv = param['vehicle_model']['parameters']['lv']                                  # x-distance from Vehicle CoG to the front hub [m]
        self.lh = param['vehicle_model']['parameters']['lh']                                  # x-distance from Vehicle Cog to the rear hub [m]
        # Half track
        self.sl = param['vehicle_model']['parameters']['sl']                                  # y-distance from Vehicle Cog to the rear hub [m]
        self.sr = param['vehicle_model']['parameters']['sr']                                  # y-distance from Vehicle Cog to the rear hub [m]
        self.sz = param['vehicle_model']['parameters']['sz']                                  # z-distance from Vehicle Cog to the rear hub [m]
        self.hub_fl= param['vehicle_model']['parameters']['hub_fl'] 
        self.hub_rl= param['vehicle_model']['parameters']['hub_rl'] 
        self.hub_fr= param['vehicle_model']['parameters']['hub_fr'] 
        self.hub_rr= param['vehicle_model']['parameters']['hub_rr']
        self.steering_ratio = param['vehicle_model']['parameters']['steering_ratio']

        #=====================================
        # Masses
        
        self.unsprung_mass = np.array(param['vehicle_model']['parameters']['unsprung_mass'])        # unsprung mass vector [kg]  
        self.m = np.array(param['vehicle_model']['parameters']['mass']) 

        wheel_base = self.lh + self.lv
        track_width = self.sl+self.sr
        self.m_s = (self.m/(track_width*wheel_base))*np.array([self.sr*self.lh, self.sr*self.lv,self.sl*self.lh,self.sl*self.lv])
        self.wd1 = (self.lh/(self.lh+self.lv)) * self.sr/(self.sl +self.sr)
        self.wd2 = (self.lv/(self.lh+self.lv)) * self.sr/(self.sl +self.sr)
        self.wd3 = (self.lh/(self.lh+self.lv)) * self.sr/(self.sl +self.sl)
        self.wd4 = (self.lv/(self.lh+self.lv)) * self.sr/(self.sl +self.sl)
        self.wd = np.array([self.wd1,self.wd2,self.wd3,self.wd4])