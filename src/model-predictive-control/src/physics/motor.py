import numpy as np
import rospy
from aerosandbox.numpy import *

from .static_parameters import RPM_TO_RADS, OZF_TO_N, IN_TO_M, a, b


class Motor:
    def __init__(self,
                 stall_current: float,
                 stall_torque: float,
                 free_current: float,
                 free_speed: float,

                 static_friction: float,
                 coulomb_friction: float,
                 viscous_friction: float,

                 stribeck_power: float,
                 stribeck_velocity: float):
        self.static_friction = static_friction
        self.coulomb_friction = coulomb_friction
        self.viscous_friction = viscous_friction
        self.stribeck_velocity = stribeck_velocity
        self.stribeck_power = stribeck_power

        self.nominal_voltage = 12
        self.armature_resistance = self.nominal_voltage / stall_current
        self.backemf_constant = (self.nominal_voltage - free_current * self.armature_resistance) / free_speed
        self.torque_constant = stall_torque / stall_current

    def backemf(self, speed):
        return self.backemf_constant * speed

    # TODO: make this simpler and more differentiable
    def friction(self, speed):
        return sign(speed) \
               * (self.coulomb_friction + (self.static_friction - self.coulomb_friction) *
                  exp(-pow(abs(speed / self.stribeck_velocity), self.stribeck_power))
                  + self.viscous_friction * abs(speed))

    def armature_current(self, voltage, speed):
        return (voltage - self.backemf(speed)) / self.armature_resistance

    def torque(self, voltage, speed):
        torque = self.armature_current(voltage, speed) * self.torque_constant
        friction = self.friction(speed)
        return torque #- friction # DO NOT UNCOMMENT THIS FUCKS SHIT UP


orbital_20_gearmotor = Motor(
    stall_current=11.5,
    stall_torque=175 * OZF_TO_N * IN_TO_M,
    free_current=.5,
    free_speed=340 * RPM_TO_RADS,
    static_friction=3e-3,
    coulomb_friction=2e-3,
    viscous_friction=1e-4,
    stribeck_power=.05,
    stribeck_velocity=25
)
