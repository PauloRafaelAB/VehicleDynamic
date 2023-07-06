def suspension(self):  # previsious forces, ax,ay 
    # Forces on the vehicle chassis at the pivot points Ai
    # Bardini pag. 265 eq. 11-21  

    # TODO adicionar zs dot VERIFICAR SE É ZA_DOT-ZS_DOT ou se é do outro jeito 
    # self.displacement.za = np.array([0,0,0,0]) # CONSIDERANDO QUE NAO HÁ TRANSFERENCIA DE CARGA
    self.f_zr.wheel_load_z = (self.param.eq_stiff * (-self.displacement.za + self.displacement.zs + self.displacement.l_stat
                                                     ) + self.param.dumper * (self.displacement.za_dot)) * np.matmul(self.transpose_vehicle_fixed2inertial_system, np.array([[0], [0], [1]]))[2]

    # NO MEU PONTO DE VISTA AQUI VOCE CALCULARIA COMO AS FORCAS QUE AGEM NO CG ATUAM NOS PO
    print("displacement.za", self.displacement.za)
    print("whell load z", self.f_zr.wheel_load_z)
    # V_F_fi - Forces on tires action on the chassis at the pivot points Ai of the four wheel >> Bardini pag 263

    # TODO: check transpose_vehicle_fixed2inertial_system operation  
