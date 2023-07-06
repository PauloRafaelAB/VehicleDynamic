def steering(self, steering):
    self.delta = steering * self.param.steering_ratio 
    delta_dot = float((self.last_delta - self.delta) / self.time_step)
    # delta is steering angle of the wheel 
    # ______________________________________________________________________________________________________________________________

    if (self.delta <= self.param.steering_min and delta_dot <= 0) or (self.delta >= self.param.steering_max and delta_dot >= 0):
        delta_dot = 0
    elif delta_dot <= self.param.steering_v_min:
        delta_dot = self.param.steering_v_min
    elif delta_dot >= self.param.steering_v_max:
        delta_dot = self.param.steering_v_max

    self.last_delta = self.delta

    # Where are you including the steering bar ratio? 
    # Matrix E_T_R (wheel angles) is calculate at steering fuction      
    # Bardini pag 261 eq. 11-6 (TR1, TR3)
    self.wheel_angle_front = [[np.cos(self.x_a.roll + self.delta), -np.sin(self.x_a.roll + self.delta), 0],
                              [np.sin(self.x_a.roll + self.delta), np.cos(self.x_a.roll + self.delta), 0],
                              [0, 0, 1]]

    # Eq.11-6    Schramm and Bardini Pag 261 (TR2, TR4)
    self.wheel_angle_rear = [[np.cos(self.x_a.roll), - np.sin(self.x_a.roll), 0],
                             [np.sin(self.x_a.roll), np.cos(self.x_a.roll), 0],
                             [0, 0, 1]] 

    self.VTR_front_axel = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    self.VTR_rear_axel = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])

    # Wheel fixed coordinate(KR) rotation relativ to Kv(vehicle system) Bardni pag. 260 eq. 11-9
    # self.VTR_front_axel = np.array([[                                                 np.cos(self.delta) * np.cos(self.x_a.picth),                                                -np.sin(self.delta) * np.cos(self.x_a.picth),                          -np.sin(self.x_a.picth)],
    #                                [ np.sin(self.x_a.yaw) * np.sin(self.x_a.picth) * np.cos(self.delta) + np.cos(self.x_a.yaw) * np.sin(self.delta),     -np.sin(self.x_a.yaw) * np.sin(self.x_a.picth) * np.sin(self.delta) + np.cos(self.x_a.yaw) * np.cos(self.delta),            np.sin(self.x_a.yaw)* np.cos(self.x_a.picth)],
    #                                [ np.cos(self.x_a.yaw) * np.sin(self.x_a.picth) * np.cos(self.delta) - np.sin(self.x_a.yaw) * np.sin(self.delta),     -np.cos(self.x_a.yaw) * np.sin(self.x_a.picth) * np.sin(self.delta) - np.sin(self.x_a.yaw) * np.cos(self.delta),            np.cos(self.x_a.yaw)* np.cos(self.x_a.picth)]])

    # # Bardni. Pag. 260 eq. 11-10
    # self.VTR_rear_axel = np.array([[                  np.cos(self.x_a.picth),                                  0,                                   -np.sin(self.x_a.picth)],
    #                               [ np.sin(self.x_a.yaw) * np.sin(self.x_a.picth),     np.cos(self.x_a.yaw),            np.sin(self.x_a.yaw)* np.cos(self.x_a.picth)],
    #                               [ np.cos(self.x_a.yaw) * np.sin(self.x_a.picth),   - np.sin(self.x_a.yaw),            np.cos(self.x_a.yaw)* np.cos(self.x_a.picth)]])

    "says rotational transformation to Kv is necessary (VTR_front_axel)>> def vehiclefixed2inertial_system"

    # return self.wheel_angle_front, self.wheel_angle_rear
