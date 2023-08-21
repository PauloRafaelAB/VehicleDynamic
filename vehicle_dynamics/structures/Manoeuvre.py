from collections import namedtuple


ManoeuvreState = namedtuple('ManoeuvreState', 'throttle brake steering')


class Manoeuvre():
    """docstring for Manoeuvre"""

    def __init__(self, steering, throttle, brake, time):
        super(Manoeuvre, self).__init__()
        self.steering = steering
        self.throttle = throttle
        self.brake = brake
        self.time = time

    def __getitem__(self, items):
        """ 
        receives items to be returned
        returns ManoeuvreState(throttle, brake, steering)

        """
        return ManoeuvreState(self.throttle[items], self.brake[items], self.steering[items])
