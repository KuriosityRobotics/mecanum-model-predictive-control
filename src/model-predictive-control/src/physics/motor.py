import numpy as np
import rospy
from aerosandbox.numpy import *

from .static_parameters import RPM_TO_RADS


class Motor:
    def __init__(self,
                 stall_current: float,
                 free_speed: float,
                 free_current: float,
                 module: float,
                 reduction: float,
                 nominal_voltage: float = 12,
                 inductance: float = 0):
        self.module = module
        self.reduction = reduction
        self.nominal_voltage = nominal_voltage
        self.inductance = inductance

        self.armature_resistance = self.nominal_voltage / stall_current
        self.motor_constant = (self.nominal_voltage - self.armature_resistance * free_current) / (free_speed * self.armature_resistance)

    def friction(self, speed):
        return .35 * sign(speed)

    def torque(self, voltage, speed):
        return (voltage * self.module * self.reduction * self.motor_constant / self.armature_resistance) - (
            speed * self.module * self.reduction**2 * self.motor_constant**2 / self.armature_resistance
        ) - self.friction(speed)

    def power(self, voltage, speed):
        return voltage * (voltage - self.reduction * self.motor_constant * speed) / self.armature_resistance


orbital_20_gearmotor = Motor(
    stall_current=11.7,
    free_speed=6600 * RPM_TO_RADS,
    free_current=.4,
    module=.48,
    reduction=19.2,
)
