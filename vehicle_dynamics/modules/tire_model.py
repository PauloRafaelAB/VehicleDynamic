def tire_model(self):  # fz, slip
    # Input slip, fz - fx_car, fy_car
    print('wheel_load_z', self.f_zr.wheel_load_z)
    self.x_rf.fx = self.f_zr.wheel_load_z * self.param.d * np.sin(self.param.c * np.arctan(self.param.b * self.slip_x - self.param.e * (self.param.b * self.slip_x - np.arctan(self.param.b * self.slip_x))))
    self.x_rf.fy = self.f_zr.wheel_load_z * self.param.d * np.sin(self.param.c * np.arctan(self.param.b * self.slip_y - self.param.e * (self.param.b * self.slip_y - np.arctan(self.param.b * self.slip_y))))

    for i in range(4):

        # 3x4
        self.compiled_wheel_forces[i] = np.array([self.x_rf.fx[i], self.x_rf.fy[i], self.f_zr.wheel_load_z[i]])
        # print('self.compiled_wheel_forces',self.compiled_wheel_forces,type(self.compiled_wheel_forces),self.compiled_wheel_forces.shape)

    """if i % 2 == 0:
        #TODO: check matrix operation 3x3 3x4>> define wheel_forces_transfomed matrix
        self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] =  np.matmul(self.VTR_front_axel,self.compiled_wheel_forces[i]) # Bardini pag. 267 eq.32
        
    else: 
        self.x_rf.wheel_forces_transformed_force2vehicle_sys[i] =  np.matmul(self.VTR_rear_axel,self.compiled_wheel_forces[i])
        
    """
    # 3x4 = 3x3 @ 3x4
    # 3x1 = 3x3 @ 3x1

    self.x_rf.wheel_forces_transformed_force2vehicle_sys[:, 0] = np.matmul(self.VTR_front_axel, self.compiled_wheel_forces.T[:, 0])
    self.x_rf.wheel_forces_transformed_force2vehicle_sys[:, 1] = np.matmul(self.VTR_rear_axel, self.compiled_wheel_forces.T[:, 1])
    self.x_rf.wheel_forces_transformed_force2vehicle_sys[:, 2] = np.matmul(self.VTR_front_axel, self.compiled_wheel_forces.T[:, 2])
    self.x_rf.wheel_forces_transformed_force2vehicle_sys[:, 3] = np.matmul(self.VTR_rear_axel, self.compiled_wheel_forces.T[:, 3])

    # forces on the vehicle chassis (Ai) >> Bardini pag 236  >> horizontal forces pag 264 self.f_za.f_za
    self.strut2chassi_xyz = self.compiled_wheel_forces

    print("VTR FRONT AXLE", self.VTR_front_axel)
    print("Compiled wheel forces ", self.compiled_wheel_forces)
    print("Compiled wheel force to vehicle", self.x_rf.wheel_forces_transformed_force2vehicle_sys)
