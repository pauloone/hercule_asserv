"""
    This module contains the Unicycle class, that will simulate the unicycle
"""

import numpy
from scipy.integrate import solve_ivp


class Unicycle:
    """
    Simulate an unicycle robot
    """

    def __init__(self, interwheel_distance: float):
        """
        :param interwheel_distance: the distance between the two wheels
        """
        self.x = 0
        self.y = 0
        self.theta = 0

    @staticmethod
    def _cinematic_model(v: float, omega: float, theta: float) -> numpy.array:
        """
        Cinematic model of the
        :param v:
        :param omega:
        :return: a numpy array, representing the x_speed, y_speed and rotational_speed of the robot
        """

        return numpy.array(
            [v * numpy.cos(theta),
             v * numpy.sin(theta),
             omega]
        )

    def next_step(self, v: float, omega: float, step_size: float) -> None:
        """

        :param v: linear speed of the robot
        :param omega: rotational speed of the robot
        :return: calculate the next position of the robot based on input linear speed and rotational speed
        """

        sol = solve_ivp(
            lambda t, state: self._cinematic_model(v, omega, state[2]),
            (0, step_size),
            [self.x, self.y, self.theta])
        # the next point is the last step of the solver
        self.x = sol.y[0][-1]
        self.y = sol.y[1][-1]
        self.theta = sol.y[2][-1]