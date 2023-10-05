import matplotlib.pyplot as plt


def plot_function(output_states, manoeuvre):
    states = output_states[:].x_a
    throttle = manoeuvre[:].throttle
    x_position = [i.x for i in states]
    pitch = [i.pitch for i in states]
    vx = [i.vx for i in states]
    vy = [i.vy for i in states]

    gear = output_states[:].gear
    plt.plot(manoeuvre.time, x_position, '4k', label='x position')
    plt.plot(manoeuvre.time, vx, '2y', label='vx')
    plt.legend(loc=0)
    plt.twinx()
    plt.plot(manoeuvre.time, throttle, 'b', label='throttle')
    plt.plot(manoeuvre.time, pitch, '4c', label='pitch')
    plt.step(manoeuvre.time, gear, 'k', label='gear')
    plt.xlabel('time (s)')
    plt.title('Logitudinal dynamic')
    plt.legend(loc=5)
    plt.grid()
    

    plt.figure()
    plt.plot(manoeuvre.time, output_states.slip_y,"*", label = "Slip y")
    plt.plot(manoeuvre.time, output_states.slip_x, label = "Slip x")
    plt.legend(loc=0)
    plt.twinx()
    plt.plot(manoeuvre.time, output_states.powertrain_net_torque,"--",label = "Torque")
    plt.legend(loc=3)
    plt.title("Fodeu")
    
    
    plt.figure()
    plt.plot(manoeuvre.time, output_states.last_delta, label = "Last delta")
    plt.legend(loc=0)
    plt.title("Delta")


    
    plt.figure()
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,0] for i in output_states[:].x_rf],"*", label = "Forces on the wheel 0r")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,1] for i in output_states[:].x_rf],"*",label = "Forces on the wheel 1f")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,2] for i in output_states[:].x_rf],label = "Forces on the wheel 2r")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[1,3] for i in output_states[:].x_rf],label = "Forces on the wheel 3f")
    plt.legend(loc=0)
    plt.plot(manoeuvre.time, manoeuvre.steering, '--m', label='steer')
    plt.legend(loc=3)
    plt.xlabel('time (s)')
    plt.title("Fy")
    plt.twinx()
    plt.step(manoeuvre.time, gear, 'k', label='gear')

    plt.figure()
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[0,0] for i in output_states[:].x_rf],"*", label = "Forces on the wheel 0r")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[0,1] for i in output_states[:].x_rf],"*",label = "Forces on the wheel 1f")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[0,2] for i in output_states[:].x_rf],label = "Forces on the wheel 2r")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[0,3] for i in output_states[:].x_rf],label = "Forces on the wheel 3f")
    plt.legend(loc=0)
    plt.plot(manoeuvre.time, manoeuvre.steering, '--m', label='steer')
    plt.legend(loc=3)
    plt.xlabel('time (s)')
    plt.title("Fx")
    plt.twinx()
    plt.step(manoeuvre.time, gear, 'k', label='gear')
      
    plt.figure()
    #plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[2,0] for i in output_states[:].x_rf],"*", label = "Forces on the wheel 0r")
    #plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[2,1] for i in output_states[:].x_rf],"*",label = "Forces on the wheel 1f")
    #plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[2,2] for i in output_states[:].x_rf],label = "Forces on the wheel 2r")
    plt.plot(manoeuvre.time, [i.wheel_forces_transformed_force2vehicle_sys[2,3] for i in output_states[:].x_rf],label = "Forces on the wheel 3f")
    plt.legend(loc=0)
    #plt.plot(manoeuvre.time, manoeuvre.steering, '--m', label='steer')
    plt.legend(loc=3)
    plt.xlabel('time (s)')
    plt.title("Fz")
    plt.twinx()
    plt.step(manoeuvre.time, gear, 'k', label='gear')
    
    
    plt.figure()
    plt.plot(manoeuvre.time, [i.roll for i in states],label = " roll")
    plt.plot(manoeuvre.time, [i.pitch for i in states],label = "pitch")
    plt.plot(manoeuvre.time, [i.yaw for i in states],label = "yaw")
    plt.plot(manoeuvre.time, vy, '--y', label='vy')
    plt.legend(loc=1)
    plt.twinx()
    plt.plot(manoeuvre.time, manoeuvre.steering, '--m', label='steer')
    plt.plot(manoeuvre.time, manoeuvre.throttle, '--c', label='throttle')
    plt.legend(loc=3)
    plt.xlabel('time (s)')
    plt.title('Euler Angles ')
    
    
    
    plt.figure()
    var_name="acc_x"
    plt.plot(manoeuvre.time, [getattr(i,var_name) for i in states],label = var_name)
    var_name="acc_y"
    plt.plot(manoeuvre.time, [getattr(i,var_name) for i in states],label = var_name)

    plt.legend(loc=0)
    plt.twinx()
    plt.plot(manoeuvre.time, manoeuvre.steering, '--m', label='steer')
    plt.plot(manoeuvre.time, manoeuvre.throttle, '--c', label='throttle')
    plt.legend(loc=3)
    plt.xlabel('time (s)')
    plt.title('Euler Angles')
    
    plt.figure()
    var_name="wx"
    plt.plot(manoeuvre.time, [getattr(i,var_name) for i in states],"*",label = var_name)
    var_name="wy"
    plt.plot(manoeuvre.time, [getattr(i,var_name) for i in states],label = var_name)
    plt.legend()
    plt.show()
