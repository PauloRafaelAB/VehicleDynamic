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
    plt.plot(manoeuvre.time, [i.roll for i in states],label = " roll")
    plt.plot(manoeuvre.time, [i.pitch for i in states],label = "pitch")
    plt.plot(manoeuvre.time, [i.yaw for i in states],label = "yaw")
    plt.plot(manoeuvre.time, vy, '--y', label='vy')
    plt.legend(loc=0)
    plt.twinx()
    plt.plot(manoeuvre.time, manoeuvre.steering, '--m', label='steer')
    plt.plot(manoeuvre.time, manoeuvre.throttle, '--c', label='throttle')
    plt.legend(loc=3)
    plt.xlabel('time (s)')
    plt.title('Euler Angles ')
    plt.show()
