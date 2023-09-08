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
from vehicle_dynamics.structures.OutputStates import OutputStates

from vehicle_dynamics.modules.chassis_translation import chassis_translation
from vehicle_dynamics.modules.chassis_rotation import chassis_rotation
from vehicle_dynamics.modules.powertrain import Powertrain
from vehicle_dynamics.modules.road import road
from vehicle_dynamics.modules.rotational_matrix import rotational_matrix
from vehicle_dynamics.modules.steering import steering
from vehicle_dynamics.modules.suspension import suspension
from vehicle_dynamics.modules.tire_model import tire_model
from vehicle_dynamics.modules.wheel_angular import wheel_angular
from vehicle_dynamics.modules.wheel_slip import wheel_slip

from vehicle_dynamics.utils.Initialization import Initialization
from vehicle_dynamics.utils.LocalLogger import LocalLogger

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

    def __init__(self, state_0 = np.zeros(15), initial_gear = 1, freq=100, param_path = ""):
        self.logger = LocalLogger("MainLogger").logger
        self.logger.setLevel("INFO")
        self.parameters = Initialization(param_path, freq, state_0, initial_gear, self.logger)
        self.powertrain = Powertrain(self.parameters)

    def tick(self, throttle, brake, steering_angle):
        self.parameters, self.logger = self.powertrain.powertrain(self.parameters, self.logger, throttle, brake)
        self.parameters, self.logger = steering(self.parameters, self.logger, steering_angle)
        self.parameters, self.logger = wheel_slip(self.parameters, self.logger) 
        self.parameters, self.logger = tire_model(self.parameters, self.logger) 
        self.parameters, self.logger = wheel_angular(self.parameters, self.logger) 
        self.parameters, self.logger = road(self.parameters, self.logger) 
        self.parameters, self.logger = suspension(self.parameters, self.logger) 
        self.parameters, self.logger = chassis_translation(self.parameters, self.logger)  
        self.parameters, self.logger = chassis_rotation(self.parameters, self.logger)  

        return self.parameters
