import yaml
import numpy as np


class ImportParam(object):

    def __init__(self, path='config.yaml'):

        with open(path, 'r') as file:
            param = yaml.safe_load(file)

        # =====================================
        # Engine Coeficients
        # =====================================
        self.min_engine_w = param['vehicle_model']['parameters']['min_engine_w']
        self.max_engine_w = param['vehicle_model']['parameters']['max_engine_w']
        self.engine_w_table = np.array(
            param['vehicle_model']['parameters']['engine_w_table'])
        self.torque_max = np.array(
            param['vehicle_model']['parameters']['torque_max'])
        self.gear_ratio = np.array(
            param['vehicle_model']['parameters']['gear_ratio'])
        self.gear_selection = np.array(
            param['vehicle_model']['parameters']['gear_selection'])
        # Differential ratio
        self.diff = param['vehicle_model']['parameters']['diff']
        self.diff_ni = param['vehicle_model']['parameters']['diff_ni']
        self.transmition_ni = param['vehicle_model']['parameters']['transmition_ni']
        self.brake_bias = np.array(
            param['vehicle_model']['parameters']['b_bias'])
        self.torque_max_table = np.array(
            param['vehicle_model']['parameters']['torque_max'])
        self.engine_torque_drag = np.array(
            param['vehicle_model']['parameters']['engine_torque_drag'])
        # =====================================
        # Powertrain parameters
        # =====================================
        self.speed_ratio_TC = np.array(
            param['vehicle_model']['parameters']['speed_ratio_TC'])
        self.torque_converter_ratio = np.array(
            param['vehicle_model']['parameters']['torque_converter_ratio'])
        self.torque_converter_efficiency = np.array(
            param['vehicle_model']['parameters']['torque_converter_efficiency'])

        # =====================================
        # Tire Data
        # =====================================
        # self.cx = param['vehicle_model']['parameters']['c_tire_x']                                    # Tire Stiffiness [N/m]
        # self.cy = param['vehicle_model']['parameters']['c_tire_y']                                     # Tire Stiffiness [N/m]
        # self.F_max = param['vehicle_model']['parameters']['f_max']                                     # Tire max load
        # self.bx = np.array(param['vehicle_model']['parameters']['bx'])                                           # tire coeficient Bardini pag 268
        # self.by = np.array(param['vehicle_model']['parameters']['by'])
        # self.cr = np.array(param['vehicle_model']['parameters']['cr'])                                  # Tire vertical Stiffnes
        # sum of all 4 wheel inertia
        self.i_wheel = param['vehicle_model']['parameters']['i_wheel']
        self.wheel_mass = np.array(
            param['vehicle_model']['parameters']['wheel_mass'])
        # Tire Radius Static [m]
        self.r_stat = param['vehicle_model']['parameters']['r_stat']
        self.r_dyn = np.array(param['vehicle_model']['parameters']['r_dyn'])
        # Rolling Resistance Parameters

        # Magic formula coefficient
        self.d = param['vehicle_model']['parameters']['magic_formula_d']
        self.e = param['vehicle_model']['parameters']['magic_formula_e']
        self.c = param['vehicle_model']['parameters']['magic_formula_c']
        self.b = param['vehicle_model']['parameters']['magic_formula_b']
        # =====================================
        # Resistance Parameters
        # =====================================
        # Air Drag Coefficient
        self.Cd = param['vehicle_model']['parameters']['cd']
        # Front Area of Car
        self.Front_area = param['vehicle_model']['parameters']['front_area']
        # Air Dencity
        self.Air_density = param['vehicle_model']['parameters']['air_density']
        self.row = param['vehicle_model']['parameters']['row']

        # "https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/PYTHON/vehiclemodels/vehicle_parameters.py "

        # steering constraints
        # max steering wheel angle [rad]
        self.steering_lock = param['vehicle_model']['parameters']['steering_lock']
        # maximum steering angle [rad]
        #self.steering_max = param['vehicle_model']['parameters']['steering_max']
        # minimum steering velocity [rad/s]
        #self.steering_v_min = param['vehicle_model']['parameters']['steering_v_min']
        # maximum steering velocity [rad/s]
        self.steering_v_max = param['vehicle_model']['parameters']['steering_v_max']

        # =====================================
        # Masses
        # sprung mass [kg]
        self.m_s = np.array(param['vehicle_model']['parameters']['m_s'])
        # unsprung mass vector [kg]
        self.unsprung_mass = np.array(
            param['vehicle_model']['parameters']['unsprung_mass'])
        # Vehicle Mass [kg]
        self.m = param['vehicle_model']['parameters']['mass']
        # =====================================
        "Intertial Resistance Parameters need to be measured"
        # =====================================
        # Wheel inertia [Kgm^2]
        self.wheel_inertia = param['vehicle_model']['parameters']['iw']
        # Axel inertia [Kgm^2]
        self.axel_inertia = param['vehicle_model']['parameters']['ia']
        # drive shaft inertia [Kgm^2]
        self.shaft_inertia = param['vehicle_model']['parameters']['id']
        # gear box inertia
        self.gearbox_inertia = param['vehicle_model']['parameters']['ig']
        # Engine inertia [Kgm^2]
        self.engine_inertia = param['vehicle_model']['parameters']['engine_inertia']
        self.max_brake_torque = param['vehicle_model']['parameters']['max_brake_torque']
        # moment of inertia for sprung mass in roll [kg m^2]  iXS
        self.i_x_s = param['vehicle_model']['parameters']['i_x_s']
        # moment of inertia for sprung mass in pitch [kg m^2]  iYS
        self.i_y_s = param['vehicle_model']['parameters']['i_y_s']
        # moment of inertia for sprung mass in yaw [kg m^2]  iZZ
        self.i_z = param['vehicle_model']['parameters']['i_z']
        # calculate
        self.i_d_shaft = param['vehicle_model']['parameters']['i_d_shaft']
        self.i_clutch = param['vehicle_model']['parameters']['i_clutch']

        # suspension parameters
        # suspension spring rate (front i = 1 or 3)  (rear = 1 = 2,4) [N/m]  KSF
        self.eq_stiff = np.array(
            param['vehicle_model']['parameters']['eq_stiff'])
        # suspension damping rate (front i = 1 or 3)  (rear = 1 = 2,4)[N s/m]  KSDF
        self.dumper = np.array(param['vehicle_model']['parameters']['dumper'])
        # Anti roll bar stiffness [Nm/rad]
        self.k_roll = param['vehicle_model']['parameters']['k_roll']
        self.c_roll = param['vehicle_model']['parameters']['c_roll']
        self.k_pitch = param['vehicle_model']['parameters']['k_pitch']
        self.c_pitch = param['vehicle_model']['parameters']['c_pitch']
        # =====================================
        # geometric parameters
        # =====================================
        # axes distances
        # x-distance from Vehicle CoG to the front hub [m]
        self.lv = param['vehicle_model']['parameters']['lv']
        # x-distance from Vehicle Cog to the rear hub [m]
        self.lh = param['vehicle_model']['parameters']['lh']
        # Half track
        # y-distance from Vehicle Cog to the rear hub [m]
        self.sl = param['vehicle_model']['parameters']['sl']
        # y-distance from Vehicle Cog to the rear hub [m]
        self.sr = param['vehicle_model']['parameters']['sr']
        # z-distance from Vehicle Cog to the rear hub [m]
        self.sz = param['vehicle_model']['parameters']['sz']
        self.hub_fl = param['vehicle_model']['parameters']['hub_fl']
        self.hub_rl = param['vehicle_model']['parameters']['hub_rl']
        self.hub_fr = param['vehicle_model']['parameters']['hub_fr']
        self.hub_rr = param['vehicle_model']['parameters']['hub_rr']
        self.steering_ratio = param['vehicle_model']['parameters']['steering_ratio']

        # =====================================
        # Masses

        # unsprung mass vector [kg]
        self.unsprung_mass = np.array(
            param['vehicle_model']['parameters']['unsprung_mass'])
        self.m = np.array(param['vehicle_model']['parameters']['mass'])

        wheel_base = self.lh + self.lv
        track_width = self.sl + self.sr
        self.m_s = (self.m / (track_width * wheel_base)) * np.array(
            [self.sr * self.lh, self.sr * self.lv, self.sl * self.lh, self.sl * self.lv])
        self.wd1 = (self.lh / (self.lh + self.lv)) * \
            self.sr / (self.sl + self.sr)
        self.wd2 = (self.lv / (self.lh + self.lv)) * \
            self.sr / (self.sl + self.sr)
        self.wd3 = (self.lh / (self.lh + self.lv)) * \
            self.sr / (self.sl + self.sl)
        self.wd4 = (self.lv / (self.lh + self.lv)) * \
            self.sr / (self.sl + self.sl)
        self.wd = np.array([self.wd1, self.wd2, self.wd3, self.wd4])
