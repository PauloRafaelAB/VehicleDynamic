class TireForces(object):
    def __init__(self, fx =np.zeros(4), fy =np.zeros(4), wheel_forces_transformed_force2vehicle_sys = np.zeros((3, 4), dtype=float)):        
        # Dynamic forces on the tires
        # pag 272 eq.54
        self.fx = fx
        self.fy = fy
        self.wheel_forces_transformed_force2vehicle_sys = wheel_forces_transformed_force2vehicle_sys 