def rotational_matrix(self):  # ONDE ESTÁ ATUALIZANDO OS ANGULOS?-------------------------------------------------------------
    # For the angular velocity of the chassis we have:   Bardini Pag. 261 Eq. 11.5
    rotationalmatrix = [[-np.sin(self.x_a.picth), 0, 1],
                        [np.cos(self.x_a.picth) * np.sin(self.x_a.yaw), np.cos(self.x_a.yaw), 0],
                        [np.cos(self.x_a.picth) * np.cos(self.x_a.yaw), - np.sin(self.x_a.yaw), 0]]

    # yaw_dot,picth_dot, roll_dot -- Bardini Pag. 261 Eq. 11.4
    self.angular_rates = np.array([self.x_a.wx,
                                   self.x_a.wy,
                                   self.x_a.wz])  # It is already calculated in the rotationalmatrix CALCULATE Only once please

    # Coordinate representation of the absolute velocity of point v with respect to coordinate system “E”, described in coordinates of coordinate system “v” bardni. Pag 260
    self.angular_vel_2inercial_sys_in_vehicle_coord = np.dot(rotationalmatrix, self.angular_rates)  # Bardini Pag. 261 Eq. 11.4    
    # return self.angular_vel_2inercial_sys_in_vehicle_coord, self.rotationalmatrix
