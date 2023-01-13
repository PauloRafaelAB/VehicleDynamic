# -*- coding: utf-8 -*-
"""
Created on Thu Oct 13 10:28:58 2022

@author:  Paulo R.A. Bloemer, Maikol Funk Drechsler
Vehicle Dynamic Model

"""
from scipy.interpolate import interp1d
from scipy.integrate import odeint
import numpy as np
import yaml
import VehicleDynamics

class ParameterEstimation(VehicleDynamics):
    
    def __init__(self, estimation_file = "config.yaml", data_path = ""):
        
        #Open the first osi message to read the initial parameters 
        VehicleDynamics.__init__(self, initial_speed = 0., state_0 = [0., 0., 0., 0., 0., 0.], initial_gear = 0, freq=100, param_path ="")
        
        