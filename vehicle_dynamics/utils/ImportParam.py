import yaml
import numpy as np

class ImportParam(object):

    def __init__(self, path = 'config.yaml'):

        with open(path, 'r') as file:
            param = yaml.safe_load(file)

        # =====================================
        # Engine Coeficients
        # =====================================
        self.min_rpm = param['vehicle_model']['parameters']['min_rpm'] 
        self.rpm_table = np.array(param['vehicle_model']['parameters']['rpm_table'])
        self.torque_max = np.array(param['vehicle_model']['parameters']['torque_max'])
        self.gear_ratio = np.array(param['vehicle_model']['parameters']['gear_ratio'])
        self.gear_selection = np.array(param['vehicle_model']['parameters']['gear_selection'])
        self.diff = param['vehicle_model']['parameters']['diff']                       # Differential ratio
        self.diff_ni = param['vehicle_model']['parameters']['diff_ni']
        self.transmition_ni = param['vehicle_model']['parameters']['transmition_ni']
        self.b_bias = np.array(param['vehicle_model']['parameters']['b_bias'])

        # =====================================
        # Powertrain parameters
        # =====================================
        self.speed_ratio_TC = np.array(param['vehicle_model']['parameters']['speed_ratio_TC'])
        self.torque_converter_ratio = np.array(param['vehicle_model']['parameters']['torque_converter_ratio'])

        # =====================================
        # Tire Data
        # =====================================
        # self.cx = param['vehicle_model']['parameters']['c_tire_x']                                    # Tire Stiffiness [N/m]
        # self.cy = param['vehicle_model']['parameters']['c_tire_y']                                     # Tire Stiffiness [N/m]
        # self.F_max = param['vehicle_model']['parameters']['f_max']                                     # Tire max load
        # self.bx = np.array(param['vehicle_model']['parameters']['bx'])                                           # tire coeficient Bardini pag 268
        #self.by = np.array(param['vehicle_model']['parameters']['by'])
        # self.cr = np.array(param['vehicle_model']['parameters']['cr'])                                  # Tire vertical Stiffnes
        self.i_wheel = param['vehicle_model']['parameters']['i_wheel']                                  # sum of all 4 wheel inertia
        self.wheel_mass = np.array(param['vehicle_model']['parameters']['wheel_mass'])
        self.r_stat = param['vehicle_model']['parameters']['r_stat']                                    # Tire Radius Static [m]
        self.r_dyn = np.array(param['vehicle_model']['parameters']['r_dyn'])    
        # Rolling Resistance Parameters

        self.d = param['vehicle_model']['parameters']['magic_formula_d']                                    # Magic formula coefficient
        self.e = param['vehicle_model']['parameters']['magic_formula_e']
        self.c = param['vehicle_model']['parameters']['magic_formula_c']
        self.b = param['vehicle_model']['parameters']['magic_formula_b']
        # =====================================
        # Resistance Parameters
        # =====================================
        self.Cd = param['vehicle_model']['parameters']['cd']                                                 # Air Drag Coefficient
        self.Front_area = param['vehicle_model']['parameters']['front_area']  # Front Area of Car
        self.Air_density = param['vehicle_model']['parameters']['air_density']                               # Air Dencity 
        self.row = param['vehicle_model']['parameters']['row']

        # "https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/PYTHON/vehiclemodels/vehicle_parameters.py "

        # steering constraints
        self.steering_min = param['vehicle_model']['parameters']['steering_min']  # minimum steering angle [rad]
        self.steering_max = param['vehicle_model']['parameters']['steering_max']  # maximum steering angle [rad]
        self.steering_v_min = param['vehicle_model']['parameters']['steering_v_min']  # minimum steering velocity [rad/s]
        self.steering_v_max = param['vehicle_model']['parameters']['steering_v_max']  # maximum steering velocity [rad/s]

        # =====================================
        # Masses
        self.m_s = np.array(param['vehicle_model']['parameters']['m_s'])                            # sprung mass [kg]  
        self.unsprung_mass = np.array(param['vehicle_model']['parameters']['unsprung_mass'])        # unsprung mass vector [kg]  
        self.m = param['vehicle_model']['parameters']['mass']                                       # Vehicle Mass [kg] 
        # =====================================
        "Intertial Resistance Parameters need to be measured"
        # =====================================
        self.iw = param['vehicle_model']['parameters']['iw']                                    # Wheel inertia [Kgm^2]
        self.ia = param['vehicle_model']['parameters']['ia']                                    # Axel inertia [Kgm^2]
        self.id = param['vehicle_model']['parameters']['id']                                    # drive shaft inertia [Kgm^2]
        self.ig = param['vehicle_model']['parameters']['ig']            # gear box inertia
        self.engine_inertia = param['vehicle_model']['parameters']['engine_inertia']                              # Engine inertia [Kgm^2]
        self.max_brake_torque = param['vehicle_model']['parameters']['max_brake_torque']  
        # self.i_uf = param['vehicle_model']['parameters']['i_uf']        # moment of inertia for unsprung mass about x-axis (front) [kg m^2]  IXUF
        # self.i_ur = param['vehicle_model']['parameters']['i_ur']        # moment of inertia for unsprung mass about x-axis (rear) [kg m^2]  IXUR
        # self.i_y_w = param['vehicle_model']['parameters']['i_y_w']      # wheel inertia, from internet forum for 235/65 R 17 [kg m^2]
        self.i_x_s = param['vehicle_model']['parameters']['i_phi_s']      # moment of inertia for sprung mass in roll [kg m^2]  iXS
        self.i_y_s = param['vehicle_model']['parameters']['i_y_s']     # moment of inertia for sprung mass in pitch [kg m^2]  iYS
        self.i_z = param['vehicle_model']['parameters']['i_z']          # moment of inertia for sprung mass in yaw [kg m^2]  iZZ
        # self.i_xz_s = param['vehicle_model']['parameters']['i_xz_s']    # moment of inertia cross product [kg m^2]  iXZ
        self.i_d_shaft = param['vehicle_model']['parameters']['i_d_shaft']                    # calculate
        self.i_clutch = param['vehicle_model']['parameters']['i_clutch']

        # suspension parameters
        self.eq_stiff = np.array(param['vehicle_model']['parameters']['eq_stiff'])      # suspension spring rate (front i = 1 or 3)  (rear = 1 = 2,4) [N/m]  KSF 
        self.dumper = np.array(param['vehicle_model']['parameters']['dumper'])              # suspension damping rate (front i = 1 or 3)  (rear = 1 = 2,4)[N s/m]  KSDF
        self.anti_roll_stiffness = param['vehicle_model']['parameters']['anti_roll_stiffness']  # Anti roll bar stiffness [Nm/rad]

        # =====================================
        # geometric parameters
        # ===================================== 
        # axes distances
        self.lv = param['vehicle_model']['parameters']['lv']                                  # x-distance from Vehicle CoG to the front hub [m]
        self.lh = param['vehicle_model']['parameters']['lh']                                  # x-distance from Vehicle Cog to the rear hub [m]
        # Half track
        self.sl = param['vehicle_model']['parameters']['sl']                                  # y-distance from Vehicle Cog to the rear hub [m]
        self.sr = param['vehicle_model']['parameters']['sr']                                  # y-distance from Vehicle Cog to the rear hub [m]
        self.sz = param['vehicle_model']['parameters']['sz']                                  # z-distance from Vehicle Cog to the rear hub [m]
        self.hub_fl = param['vehicle_model']['parameters']['hub_fl'] 
        self.hub_rl = param['vehicle_model']['parameters']['hub_rl'] 
        self.hub_fr = param['vehicle_model']['parameters']['hub_fr'] 
        self.hub_rr = param['vehicle_model']['parameters']['hub_rr']
        self.steering_ratio = param['vehicle_model']['parameters']['steering_ratio']

        # =====================================
        # Masses

        self.unsprung_mass = np.array(param['vehicle_model']['parameters']['unsprung_mass'])        # unsprung mass vector [kg]  
        self.m = np.array(param['vehicle_model']['parameters']['mass']) 

        wheel_base = self.lh + self.lv
        track_width = self.sl + self.sr
        self.m_s = (self.m / (track_width * wheel_base)) * np.array([self.sr * self.lh, self.sr * self.lv, self.sl * self.lh, self.sl * self.lv])
        self.wd1 = (self.lh / (self.lh + self.lv)) * self.sr / (self.sl + self.sr)
        self.wd2 = (self.lv / (self.lh + self.lv)) * self.sr / (self.sl + self.sr)
        self.wd3 = (self.lh / (self.lh + self.lv)) * self.sr / (self.sl + self.sl)
        self.wd4 = (self.lv / (self.lh + self.lv)) * self.sr / (self.sl + self.sl)
        self.wd = np.array([self.wd1, self.wd2, self.wd3, self.wd4])
