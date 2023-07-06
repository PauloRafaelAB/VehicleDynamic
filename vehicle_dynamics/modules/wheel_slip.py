def wheel_slip(self):  # input: fz, mi, wheel_angle,vx,vy   output:fz,slix,slip_y
    '''initial states of tire load are mandatory [Wheel torque], [Wheel_Fz], slip_x, slip_angle]
        calculate the forces..
    '''
    # wheel states
    # initialize angular velocity of each wheel (relative to vehicle system)
    for i in range(4):

        # Slip calculation - Wheel slip is calculated by wheel fixed system

        # if self.param.r_dyn[i] * self.wheel_w_vel[i] == abs(self.x_a.vx):
        #     self.slip_x[i] = .0
        # elif abs(self.param.r_dyn[i] * self.wheel_w_vel[i]) > abs(self.x_a.vx):
        #     self.slip_x[i] = 1 - abs(self.x_a.vx/(self.param.r_dyn[i] * self.wheel_w_vel[i] + 1e-52))
        # else:
        #     self.slip_x[i] = -1 + abs(self.param.r_dyn[i] * self.wheel_w_vel[i]/(self.x_a.vx + 1e-52))

        self.slip_x[i] = (((self.param.r_dyn[i] * self.wheel_w_vel[i] - self.x_a.vx) / max([abs(self.param.r_dyn[i] * self.wheel_w_vel[i] + 1e-26), abs(1e-26 + self.x_a.vx)])))         # equation 11.30 Bardini
        # REMOVED THE ABS()
        # print('self.wheel_w_vel[i]',self.wheel_w_vel[i])
        # print(self.x_a.vx,'vx')

        # Lateral slip: define wheel_vy Equacao 11_28 >> converte to tire direction
        self.slip_y[i] = - np.arctan(self.x_a.vy / max([abs(self.param.r_dyn[i] * self.wheel_w_vel[i] + 1e-16), abs(1e-16 + self.x_a.vx)]))  # equation 11.31 Bardini

        # _______________________________________________________________________________________________
        # TODO: Check first argument by displaying (velocity in wheel fixed coord) Bardini eq 11-29

        # Bardini pag.268 eq 11-33
        ' Replace with followig code to take antiroll bar in to account'
        # TODO: Make F_stv diferent than 0; make F_st input on tire fuction
        # wheel_load_z[0] = - max(self.param.cr[0]*(self.displacement.zr[0]-self.displacement.zs[0]+lr_stat[0]) + F_stv , 0) # Bardini pag.268 eq 11-33
    print("SLIP X ", self.slip_x)
