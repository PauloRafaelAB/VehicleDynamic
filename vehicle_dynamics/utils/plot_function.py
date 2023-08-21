import matplotlib.pyplot as plt


def plot_function(output_states, manoeuvre):
    states = output_states[:].x_a
    throttle = manoeuvre[:].throttle
    x_position = [i.x for i in states]
    pitch = [i.pitch for i in states]
    vx = [i.vx for i in states]

    gear = output_states[:].gear
    plt.plot(manoeuvre.time, throttle, 'b', label='throttle')
    plt.plot(manoeuvre.time, x_position, '4k', label='x position')
    plt.plot(manoeuvre.time, pitch, '4c', label='pitch')
    plt.plot(manoeuvre.time, vx, '2y', label='vx')
    plt.step(manoeuvre.time, gear, 'k', label='gear')
    plt.xlabel('time (s)')
    plt.title('Logitudinal dynamic')
    plt.legend()
    plt.grid()
    plt.ylim([-2, 10])

    plt.show()
