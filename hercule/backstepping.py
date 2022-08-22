"""
Implementation of the integrator backstepping for a robot
"""
import numpy
from .unicycle import Unicycle

class Backstepping:

    STATIC_FRICTION_SPEED = 0.00001 # static friction speed, necessary to rotate on ourself

    def __init__(self, kx1, k2x, ky1, k2y, unicycle: Unicycle, simulation_interval: float):
        """
        Initialise a backstepping
        :param kx1:
        :param k2x:
        """

        self.kx1 = kx1
        self.k2x = k2x
        self.ky1 = ky1
        self.k2y = k2y
        self.unicycle = unicycle
        self.simulation_interval = simulation_interval
        self.command = numpy.array([0, 0])
        self.d_command = numpy.array([0, 0])  # derivative of the command
        self.dd_command = numpy.array([0, 0])  # second derivative of the command
        self.forward_speed = 0  # the forward speed of the robot, (previous output of the backstepping command)

    def next_step(self, command: numpy.array):
        """
        :param command: an array giving the command for a specified step. first element is x command, second is y command.
        :return: a numpy two dimentional array. First element is the command forward speed, and the second is the rotational speed
        """
        d_command = (command - self.command) / self.simulation_interval
        self.command = command
        self.dd_command = (d_command - self.d_command) / self.simulation_interval
        self.d_command = d_command
        v1 = - self.kx1 * (self.unicycle.dx - self.d_command[0]) + self.dd_command[0] - self.k2x * (self.unicycle.dx - self.d_command[0] + self.kx1 * (self.unicycle.x - self.command[0]))
        v2 = - self.ky1 * (self.unicycle.dy - self.d_command[1]) + self.dd_command[1] - self.k2y * (self.unicycle.dy - self.d_command[1] + self.ky1 * (self.unicycle.y - self.command[1]))

        # dynamic decupling
        forward_speed_d = v1 * numpy.cos(self.unicycle.theta) + v2 * numpy.sin(self.unicycle.theta)
        self.forward_speed += forward_speed_d * self.simulation_interval

        # it's necessary to have a V not null for omega. we add a small 'static' fiction logic to do that
        forward_speed_with_static_friction = self.forward_speed
        if numpy.abs(forward_speed_with_static_friction) < self.STATIC_FRICTION_SPEED:
            forward_speed_with_static_friction = self.STATIC_FRICTION_SPEED*numpy.sign(forward_speed_with_static_friction)

        omega = (v2 * numpy.cos(self.unicycle.theta) - v1 * numpy.sin(self.unicycle.theta))/forward_speed_with_static_friction

        return numpy.array([self.forward_speed, omega])
