import matplotlib.pyplot as plt


def plot_function(output_states, manoeuvre, sim_data):
    states = output_states[:].x_a
    throttle = manoeuvre[:].throttle
    x_position = [i.x for i in states]
    y_position = [i.y for i in states]
    pitch = [i.pitch for i in states]
    vx = [i.vx for i in states]
    vy = [i.vy for i in states]

    gear = output_states[:].gear
    #plot 1
    
    #plt.plot(manoeuvre.time, x_position, '4k', label='x position')
    #plt.plot(manoeuvre.time, vx, '--k', label='Vx')
    '''
    plt.plot(manoeuvre.time, throttle, ':', label='Throttle')
    plt.legend(loc=0)
    plt.ylabel('Thorttle ')
    plt.twinx()
    plt.plot(manoeuvre.time, pitch, '-', label='Pitch')
    #plt.step(manoeuvre.time, gear, 'k', label='gear')
    plt.xlabel('time (s)')
    plt.ylabel('Angle [deg]')
    plt.title('Logitudinal dynamics')
    plt.legend(loc=5)
    #plt.grid()
    
    '''
    #plot 2
    plt.plot(manoeuvre.time, output_states.slip_y,"--", label = ["Slip y FL","Slip y RL","Slip y FR","Slip y RR" ])
    #plt.plot(manoeuvre.time, output_states.slip_x, label = "Slip x")
    plt.legend(loc=0)
    plt.xlabel('time (s)')
    plt.ylabel('Slip ratio')
    #plt.twinx()
    #plt.plot(manoeuvre.time, output_states.powertrain_net_torque,"--",label = "Torque")
    #plt.legend(loc=3)
    plt.title("Slip Angles")
   
    '''
    #plot 3 - steering
    plt.figure()
    plt.plot(manoeuvre.time, output_states.delta, label = "delta")
    plt.legend(loc=0)
    plt.title("Delta")
    

    #plot 4 - Fy
    
    plt.figure()
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,0] for i in output_states[:].x_rf],"*", label = "Forces on the wheel FL")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,1] for i in output_states[:].x_rf],"*",label = "Forces on the wheel RL")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,2] for i in output_states[:].x_rf],label = "Forces on the wheel FR")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,3] for i in output_states[:].x_rf],label = "Forces on the wheel RR")
    plt.legend(loc=0)
    plt.plot(manoeuvre.time, manoeuvre.steering, '--m', label='steer')
    plt.legend(loc=3)
    plt.xlabel('time (s)')
    plt.title("Fy")
    

    #plot 5
    plt.figure()
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[0,0] for i in output_states[:].x_rf],":", label = "Fx FL")
    #plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[0,1] for i in output_states[:].x_rf],"--", label = "Forces on the wheel Fx RL")
    #plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[0,2] for i in output_states[:].x_rf],"--", label = "Forces on the wheel Fx FR")
    #plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[0,3] for i in output_states[:].x_rf],"--", label = "Forces on the wheel Fx RR")
    
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,0] for i in output_states[:].x_rf],"--", label = "Fy FL")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,1] for i in output_states[:].x_rf],"--",label = "Fy RL")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,2] for i in output_states[:].x_rf],"--",label = "Fy FR")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,3] for i in output_states[:].x_rf],"--",label = "Fy RR")
    plt.legend()
    plt.ylabel('Force [N]')
    plt.xlabel('Time [s]')
    plt.title("Wheel Forces [N] ")
    plt.twinx()
    plt.plot(manoeuvre.time, output_states.delta,"k",label = "delta")
    plt.legend()   
    #plt.step(manoeuvre.time, gear, 'k', label='gear')
    

    
    '''

    #plot 6 ROLL
    plt.figure()
    #plt.plot(manoeuvre.time, [i.roll for i in states],'--',label = "Roll")
    plt.plot(manoeuvre.time, [i.yaw for i in states],'--',label = "Yaw")
    #plt.plot(manoeuvre.time, [i.pitch for i in states],"--k",label = "pitch")
    var_name="Vhcl_Yaw"
    plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"--",label='Yaw CM')
    #plt.plot(manoeuvre.time, manoeuvre.throttle, '--c', label='throttle')
    #plt.plot(manoeuvre.time, [i.yaw for i in states],'',label = "Yaw")
    #plt.plot(manoeuvre.time, vy, '--y', label='vy')
    plt.plot(manoeuvre.time, output_states.delta, ":k",label = "Delta")
    plt.legend(loc=0)
    plt.xlabel('Time (s)')
    #plt.title('Roll at Step Steer')
    plt.ylabel('Angle [rad]')
         
    #var_name="acc_x"
    #plt.plot(manoeuvre.time, [getattr(i,var_name) for i in states],'--k',label = var_name)
    #plt.step(manoeuvre.time, gear, 'k', label='gear')
    
    #plot 6-2 pitch trottle brake
    plt.figure()
    #plt.plot(manoeuvre.time, [i.roll for i in states],label = " roll")
    #plt.plot(manoeuvre.time, [i.pitch*57 for i in states],'-',label = "Pitch")
    #plt.plot(manoeuvre.time, manoeuvre.throttle, '--', label='Throttle')
    #plt.plot(manoeuvre.time, manoeuvre.brake, ':', label='Brake Pedal')
    plt.plot(manoeuvre.time, [i.yaw for i in states],label = "yaw")
    #plt.plot(manoeuvre.time, vy, '--y', label='vy')
    plt.xlabel('time (s)')
    #plt.title('Pitch at Acceleration and Braking')
    plt.ylabel('Yaw [rad]')
    plt.legend(loc=1)
    
    '''
   #plot 6-3 pitch trottle brake compariso
    plt.figure()
    #plt.plot(manoeuvre.time, [i.roll for i in states],label = " roll")
    plt.plot(manoeuvre.time, [i.pitch for i in states],'-',label = "Pitch")
    var_name="Vhcl_Pitch"
    plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"--",label='Pitch CM')
    
    plt.plot(manoeuvre.time, manoeuvre.throttle, '--', label='Throttle')
    #plt.plot(manoeuvre.time, manoeuvre.brake, ':', label='Brake Pedal')
    #plt.plot(manoeuvre.time, [i.yaw for i in states],label = "yaw")
    #plt.plot(manoeuvre.time, vy, '--y', label='vy')
    plt.xlabel('time (s)')
    plt.title('Pitch at Acceleration and Braking')
    plt.ylabel('Pitch [rad]')
    plt.legend(loc=1)
    '''
    
    '''
    #plot 6-4 pitch trottle brake compariso
    plt.figure()
    #plt.plot(manoeuvre.time, [i.roll for i in states],label = " roll")
    plt.plot(manoeuvre.time, [i.wy for i in states],'-',label = "Pitch Rate")
    var_name="Vhcl_PitchVel"
    plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"k",label='Pitch CM')
    #plt.plot(manoeuvre.time, [i.yaw for i in states],label = "yaw")
    plt.plot(manoeuvre.time, vy, '--y', label='vy')
    plt.xlabel('time (s)')
    plt.title('Pitch rate at Step Steer')
    plt.ylabel('Pitch [rad]')
    plt.legend(loc=1)   
    '''
    

    #plot 7
    plt.figure()
    #var_name="acc_x"
    #plt.plot(manoeuvre.time, [getattr(i,var_name) for i in states],'y',label = var_name)
    #var_name="acc_y"
    #plt.plot(manoeuvre.time, [getattr(i,var_name) for i in states],'b',label = var_name)
    #plt.plot(manoeuvre.time, output_states.delta, 'g', label = "delta")
    #plt.legend(loc=1)
    #plt.twinx()
    #plt.plot(manoeuvre.time, manoeuvre.throttle, '--c', label='throttle')
    plt.xlabel('time (s)')
    plt.title('Yaw Rate at Lane Change')
    plt.ylabel('Yaw Rate [Rad/s]')
    var_name="wz"
    plt.plot(manoeuvre.time, [getattr(i,var_name) for i in states],":", label = 'Yaw Rate')
    #var_name="wx"
    #plt.plot(manoeuvre.time, [getattr(i,var_name) for i in states],":", label = 'Roll Rate')
    #var_name= 'Vhcl_RollVel'
    #plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"--",label='Roll Rate CM')
    var_name= 'Vhcl_YawVel'
    plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"--",label='Yaw Rate CM')
    plt.legend(loc=2)
    
    plt.figure()
    plt.plot(x_position, label = 'x Position')
    plt.plot(y_position, label = 'y Position')
    plt.plot([getattr(sim_data[i], "Vhcl_PoI_Pos_x") for j, i in enumerate(sim_data) if j % 10 == 0],
             [getattr(sim_data[i], "Vhcl_PoI_Pos_y") for j, i in enumerate(sim_data) if j % 10 == 0])
    plt.legend()
    
    '''
    #plot 9
    plt.figure()
    plt.title("Wheel Torque")
    plt.plot(manoeuvre.time, output_states[:].powertrain_net_torque,":", label = "Forces on the wheel 0fl")
    var_name="wheel_torque_FL"
    plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"--r",label=var_name)
    var_name="wheel_torque_FR"
    plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"--m",label=var_name)
    var_name="wheel_torque_RL"
    plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"*-b",label=var_name)
    var_name="wheel_torque_RR"
    plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"-k",label=var_name)    
    plt.twinx()
    plt.plot(manoeuvre.time, manoeuvre.brake, '--y', label='brake')
    plt.legend()
    

    #plot 10
    plt.figure()
    plt.title("Vehicle Velocity")
    plt.plot(manoeuvre.time, vx, '--k', label='Vx')
    var_name="Vhcl_PoI_Vel_1_x"
    plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"--",label="Vx CM")
    plt.legend(loc=2)
    plt.twinx()
    plt.plot(manoeuvre.time, vy, ':k', label='Vy')
    var_name="Vhcl_PoI_Vel_1_y"
    plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],":",label='Vy CM')
    plt.legend(loc=0)
    '''

    #plot 11
    #plt.figure()
    #plt.title("Engine rotation")
    #plt.plot(manoeuvre.time, output_states[:].engine_w,"*",label = "Engine rotation")
    #var_name="engine_rotv"
    #plt.plot([i for j, i in enumerate(manoeuvre.time) if j % 10 == 0], [getattr(sim_data[i], var_name) for j, i in enumerate(sim_data) if j % 10 == 0],"*-",label=var_name)
    #plt.legend()
    #plt.twinx()
    #plt.plot(manoeuvre.time, manoeuvre.throttle, '--c', label='throttle')
    #plt.legend()

    plt.show()
