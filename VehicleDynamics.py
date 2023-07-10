# -*- coding: utf-8 -*-
"""
Vehicle Dynamic Model Main Class
@author:  Paulo R.A. Bloemer, Maikol Funk Drechsler, Yuri Poledna 
"""

from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.Displacement import Displacement
from vehicle_dynamics.structures.StrutForce import StrutForce
from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.AngularWheelPosition import AngularWheelPosition

from vehicle_dynamics.modules.chassis import chassis
from vehicle_dynamics.modules.powertrain import powertrain
from vehicle_dynamics.modules.road import road
from vehicle_dynamics.modules.rotational_matrix import rotational_matrix
from vehicle_dynamics.modules.steering import steering
from vehicle_dynamics.modules.suspension import suspension
from vehicle_dynamics.modules.tire_model import tire_model
from vehicle_dynamics.modules.wheel_angular import wheel_angular
from vehicle_dynamics.modules.wheel_slip import wheel_slip

from vehicle_dynamics.utils.Initialization import Initialization

from scipy.interpolate import interp1d
from numpy.linalg import inv
import numpy as np
import yaml
import math


class VehicleDynamics(object):
    """ This Class does the grunt of calculating VehicleDynamics!!
    Calculate longitudinal and lateral dynamics with desired values 
    of brake, steer and thorttle positons.
    """

    def __init__(self, initial_speed = 0., state_0 = [0., 0., 0., 0., 0., 0.], initial_gear = 1, freq=100, param_path = ""):
        self.parameters = Initialization()
        self.logger = LocalLogger("MainLogger").logger

    def tick(self, gas_pedal, brake, steering, time):
        self.parameters, self.logger = powertrain(self.parameters, self.logger, gas_pedal = gas_pedal, brake = brake)
        self.parameters, self.logger = steering(self.parameters, self.logger, steering = steering)
        self.parameters, self.logger = rotational_matrix(self.parameters, self.logger) 
        self.parameters, self.logger = wheel_slip(self.parameters, self.logger) 
        self.parameters, self.logger = tire_model(self.parameters, self.logger) 
        self.parameters, self.logger = wheel_angular(self.parameters, self.logger) 
        self.parameters, self.logger = road(self.parameters, self.logger) 
        self.parameters, self.logger = suspension(self.parameters, self.logger) 
        self.parameters, self.logger = chassis(self.parameters, self.logger)  

        # in essence you could do: 
        # self.parameters, self.logger = powertrain(steering(rotational_matrix(wheel_slip(tire_model(wheel_angular(road(suspension(chassis(self.parameters, self.logger)))))))steering=steering)gas_pedal=gas_pedal, brake=brake)

        return [self.parameters.x_a.x,
                self.parameters.x_a.y,
                self.parameters.x_a.z,
                self.parameters.x_a.roll,
                self.parameters.x_a.pitch,
                self.parameters.x_a.yaw,
                self.parameters.x_a.vx,
                self.parameters.x_a.vy,
                self.parameters.x_a.vz,
                self.parameters.x_a.wx,
                self.parameters.x_a.wy,
                self.parameters.x_a.wz,
                self.parameters.x_a.acc[0],
                self.parameters.x_a.acc[1],
                self.parameters.x_a.acc[2],
                self.parameters.gear,
                self.parameters.slip_x[0],
                self.parameters.slip_x[1],
                self.parameters.slip_x[2],
                self.parameters.slip_x[3],
                self.parameters.wheel_w_vel[0],
                self.parameters.wheel_w_vel[1]] 
