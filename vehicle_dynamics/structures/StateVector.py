class StateVector(object):
    def __init__(self, x=0., y=0., z=0., roll=0., pitch=0., yaw=0., vx=0., vy=0., vz=0., wx=0., wy=0., wz=0., acc = [0., 0., 0.]):

        # Position and Euler angles
        self.x = x
        self.y = y 
        self.z = z 
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.wx = wx 
        self.wy = wy 
        self.wz = wz
        self.acc = acc
