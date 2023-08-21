from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.Displacement import Displacement
from vehicle_dynamics.structures.StrutForce import StrutForce
from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.AngularWheelPosition import AngularWheelPosition
from collections import namedtuple
from copy import copy

OutputState = namedtuple('OutputState', 'x_a gear slip_x wheel_w_vel')


class OutputStates(object):
    """docstring for OutputStates"""

    def __init__(self):
        super(OutputStates, self).__init__()
        self.x_a = []
        self.gear = []
        self.slip_x = []
        self.wheel_w_vel = []

    def set_states(self, parameters):
        self.x_a.append(copy(parameters.x_a))
        self.gear.append(copy(parameters.gear))
        self.slip_x.append(copy(parameters.slip_x))
        self.wheel_w_vel.append(copy(parameters.wheel_w_vel))

    def __getitem__(self, items):
        return OutputState(self.x_a[items], self.gear[items], self.slip_x[items], self.wheel_w_vel[items])
