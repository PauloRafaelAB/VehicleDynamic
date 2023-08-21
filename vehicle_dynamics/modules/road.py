from vehicle_dynamics.utils.Initialization import Initialization
import logging


def access_z_road(x, y):
    # TODO implement topography
    z = 0.
    return z


def road(parameters, logger):
    for k in range(4):
        #displacement.zs[k][2] = self.access_z_road(self.displacement.zs[k][0],self.displacement.zs[k][1])
        parameters.displacement.zs[k] = 0.

    return parameters, logger 


def main():
    SIM_ITER = 1000
    test_function = road
    function_name = function.__name__

    logger = LocalLogger(function_name).logger

    parameters = Initialization("../../bmw_m8.yaml")
    logger.info("loaded Parameters")

    path_to_simulation_data = "../../exampledata/2_acc_brake/SimulationData.pickle"

    data = import_data_CM(path_to_simulation_data)
    logger.info("loaded SimulationData")

    data = [test_function(parameters, logger)[0] for i in range(SIM_ITER)]

    plt.title(function_name)
    plt.plot(data)
    plt.show()


if __name__ == '__main__':
    main()
