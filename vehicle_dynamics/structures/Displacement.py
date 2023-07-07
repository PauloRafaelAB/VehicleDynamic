import numpy as np


class Displacement(object):
    def __init__(self, l_stat= np.zeros(4), za=np.zeros(4), za_dot=np.zeros(4), zr_dot=np.zeros(4), zr_2dot = np.zeros(4), zs =np.zeros(4)):
        self.l_stat = l_stat
        self.za = za
        self.za_dot = za_dot
        self.zr_2dot = zr_2dot
        self.zr_dot = zr_dot
        self.zs = zs
        # Vertical displacements/velocity of the wheel *(x_rz)
        # pag 272 eq.53
