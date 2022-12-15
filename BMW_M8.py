# -*- coding: utf-8 -*-
"""
Created on Thu Jun 30 10:55:21 2022

@author: Paulo Rafael A.Bloemer
"""

class Parameters_bmw_m8(): 
    
    def _init_(self):
        
        self.sample_time = 0.01
        
        #=====================================
        # Engine Coeficients
        #=====================================
        #TODO: make list
        self.torque_max = [200, 550, 550, 500, 450, 380]            # [Nm]
        self.rpm_table = [1000, 1800, 5500, 6000, 7000, 8000]
        self.gear_ratio = [5, 3.2, 2.14, 1.72, 1.31, 1, 0.82, 0.64]
        self.gear_selection = [[0,	5.555555556,	8.333333333,	11.11111111,	13.88888889, 16.66666667, 20.83333333, 27.7777777]
                               [2.777777778,	5.555555556,	8.333333333,	11.11111111,	13.88888889,	16.66666667,	20.83333333,	27.77777778]
                               [5.555555556,	9.722222222,	13.88888889,	18.05555556,	22.22222222,	27.77777778,	30.55555556,	41.66666667]
                               [8.333333333,	13.88888889,	19.44444444,	25,         	30.55555556,	38.88888889,	43.05555556,	55.55555556]
                               [11.11111111,	18.05555556,	25,          	31.94444444,	38.88888889,	50,      	    55.55555556,	69.44444444]
                               [11.11111111,	22.22222222,	33.33333333,	41.66666667,	50,	            58.33333333,  	66.66666667,	75]
                               [11.38888889,	22.5,	        33.61111111,	41.94444444,	50.27777778,	58.61111111,	66.94444444,	75.27777778]
                               [11.66666667,	22.77777778,	33.88888889,	42.22222222,	50.55555556,	58.88888889,	67.22222222,	75.55555556]
                               [11.94444444,	23.05555556,	34.16666667,	42.5,       	50.83333333,	59.16666667,	67.5,	        75.83333333]
                               [12.22222222,	23.33333333,	34.44444444,	42.77777778,	51.11111111,	59.44444444,	67.77777778,	76.11111111]
                               [12.5,	        23.61111111,	34.72222222,	43.05555556,	51.38888889,	59.72222222,	68.05555556,	76.38888889]]

        
        self.diff = 3.15                        # Differential ratio
        self.diff_ni =0.98
        self.transmition_ni = 0.95
        self.I_d_shaft = 1                      # calculate
        self.I_clutch = 1
      
      
        #=====================================
        # Tire Data
        #=====================================
        self.c = 406884. ##                                     # Tire Stiffiness [N/m]
        self.F_max = 5440. ###                                  # Tire load
        
        #=====================================
        # Rolling Resistance Parameters
        #=====================================
        
        self.Road_friction = 0.015   ##                              # Road Friction Coefficient
        self.m = 1980      ##                                        # Vehicle Mass
        self.g = 9.81      ###                                       # Acceleration Due to Gravity
        
        #=====================================
        # Air Resistance Parameters
        #=====================================
        
        self.Cd = 0.35                                       #Air Drag Coefficient
        self.Front_area = 2.057###                                  #Front Area of Car
        self.Air_density = 1.2                                     # Air Dencity 
        
        #=====================================
        "Intertial Resistance Parameters need to be measured"
        #=====================================
        
        self.Iw = 0.953                                    # Wheel Inertia [Kgm^2]
        self.Ia = 0.135                                    # Axel Inertia [Kgm^2]
        self.Id = 0.0                                      # drive shaft Inertia [Kgm^2]
                                      
        self.Gd = 1                                        # Differential Gear Ratio
        self.Ig = 0.01                                     # Gear Box Inertia [Kgm^2]
        self.b_1 = 7                                       # Acceleration / Deaceleration Calib Coefficient 
        self.brake_caliper = 1.5                           # Break Calib Coefficient
        self.I_engine = 0.227                              # Engine Inertia [Kgm^2]
        self.r_stat = 0.5334                               # Tire Radius Static [m]
        self.r_dyn  = 0.5330                               # Tire Radius Dynamic [m]
       
        
        self.cg_height = 300.                              # center of gravity height of total mass [m]
        self.cg_x = 12500                                  #  cg_x [m]
        self.cg_y = 0.                                     # [m]
        self.track_width = 0.                              # [m]
        self.wheel_base = 2.6897                           # [m]
        
        # axes distances
        self.lv = 1.1853                                   # x-distance from Vehicle CoG to the front hub [m]
        self.lh = .0                                       # x-distance from Vehicle Cog to the rear hub [m]
        
        # Half track
        self.sl = .0                                       # Half track width front [m]
        self.sr = .0                                       # Half track width rear  [m]
        
        #self.weight_rear = self.m * self.CG_x / self.wb  
        #self.weight_front = self.m - self.weightrear 
        
        self.final_ratio = self.gear_ratio * self.diff    # final ratio             
        
        
        "https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/PYTHON/vehiclemodels/vehicle_parameters.py "

        
        # vehicle body dimensions
        self.l = 5  # vehicle length [m]
        self.w = 2  # vehicle width [m]
    
        # # steering constraints
        # self.steering.min = -0.910  # minimum steering angle [rad]
        # self.steering.max = 0.910  # maximum steering angle [rad]
        # self.steering.v_min = -0.4  # minimum steering velocity [rad/s]
        # self.steering.v_max = 0.4  # maximum steering velocity [rad/s]
    
        # # longitudinal constraints
        # self.longitudinal.v_min = -13.9  # minimum velocity [m/s]
        # self.longitudinal.v_max = 45.8  # minimum velocity [m/s]
        # self.longitudinal.v_switch = 4.755  # switching velocity [m/s]
        # self.longitudinal.a_max = 11.5  # maximum absolute acceleration [m/s^2]
    
        # masses
        self.m = 0  # vehicle mass [kg]  MASS
        self.m_s = 0  # sprung mass [kg]  SMASS
        self.m_uf = 0  # unsprung mass front [kg]  UMASSF
        self.m_ur = 0  # unsprung mass rear [kg]  UMASSR
    
        
    
        # moments of inertia of sprung mass
        self.I_Phi_s = 0 # moment of inertia for sprung mass in roll [kg m^2]  IXS
        self.I_y_s =  0 # moment of inertia for sprung mass in pitch [kg m^2]  IYS
        self.I_z = 0  # moment of inertia for sprung mass in yaw [kg m^2]  IZZ
        self.I_xz_s = 0   # moment of inertia cross product [kg m^2]  IXZ
    
        # suspension parameters
        self.K_sf = 0  # suspension spring rate (front) [N/m]  KSF
        self.K_sdf =  0 # suspension damping rate (front) [N s/m]  KSDF
        self.K_sr = 0  # suspension spring rate (rear) [N/m]  KSR
        self.K_sdr = 0 # suspension damping rate (rear) [N s/m]  KSDR
    
        # geometric parameters
        self.T_f = 0  # track width front [m]  TRWF
        self.T_r = 0 # track width rear [m]  TRWB
        self.K_ras =0   # lateral spring rate at compliant compliant pin joint between M_s and M_u [N/m]  KRAS
    
        self.K_tsf =   0     # auxiliary torsion roll stiffness per axle (normally negative) (front) [N m/rad]  KTSF
        self.K_tsr =  0        # auxiliary torsion roll stiffness per axle (normally negative) (rear) [N m/rad]  KTSR
        self.K_rad =  0       # damping rate at compliant compliant pin joint between M_s and M_u [N s/m]  KRADP
        self.K_zt =   0        # vertical spring rate of tire [N/m]  TSPRINGR
    
        self.h_cg = 0
        self.h_raf = 0  # height of roll axis above ground (front) [m]  HRAF
        self.h_rar = 0  # height of roll axis above ground (rear) [m]  HRAR
    
        self.h_s = 0  # M_s center of gravity above ground [m]  HS
    
        self.I_uf = 0        # moment of inertia for unsprung mass about x-axis (front) [kg m^2]  IXUF
        self.I_ur = 0   # moment of inertia for unsprung mass about x-axis (rear) [kg m^2]  IXUR
        self.I_y_w = 1.7  # wheel inertia, from internet forum for 235/65 R 17 [kg m^2]
    
        self.K_lt = 0  # lateral compliance rate of tire, wheel, and suspension, per tire [m/N]  KLT
        self.R_w = 0.344  # effective wheel/tire radius  chosen as tire rolling radius RR  taken from ADAMS documentation [m]
    
    
        # suspension parameters
        self.D_f = 0  # [rad/m]  DF
        self.D_r = 0  # [rad/m]  DR
        self.E_f = 0  # [needs conversion if nonzero]  EF
        self.E_r = 0  # [needs conversion if nonzero]  ER
    



