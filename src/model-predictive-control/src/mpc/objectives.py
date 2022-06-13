from math import pi

import rospy
from aerosandbox.numpy import *
from casadi import atan2, mod, sign, casadi, vertcat

from physics.dynamics import rotation_matrix
from physics.static_parameters import r, Ll
from .index import index
from physics.motor import orbital_20_gearmotor


def objective_dynamic(state, p):
    # qr = state[index.qr]
    # qr_dot = state[index.qr_dot]
    # angle_error = qr[2] - atan2(qr_dot[1], qr_dot[0])
    # angle_error = anglewrap(angle_error)

    # return (p[index.p.weight.slack] * state[index.slack]) ** 2 + \
    #        (p[index.p.weight.strafe] * angle_error) ** 2 \
    #        + p[index.p.weight.use_begin] * objective_dynamic_n(state, p)

    # return vertcat(p[index.p.weight.slack] * state[index.slack],
    #                # p[index.p.weight.strafe] * angle_error, # DO NOT UNCOMMENT THIS FUCKS SHIT UP
    #                *casadi.horzsplit(p[index.p.weight.energy] * state[index.u]),
    #                *casadi.horzsplit(p[index.p.weight.use_begin] * objective_dynamic_n(state, p))
    #                )

    qr_dot = state[index.qr_dot].reshape((3, 1))

    R = (1 / r) * array([[1, -1, -Ll], [1, 1, Ll], [1, 1, -Ll], [1, -1, Ll]], dtype=float)

    undo_rotation = transpose(rotation_matrix(state[index.qr[2]]))
    wheel_speeds = R @ undo_rotation @ qr_dot
    return p[index.p.weight.energy] * casadi.sum1(orbital_20_gearmotor.power(state[index.u].reshape((-1,1)), wheel_speeds))


def objective_dynamic_n(state, p):
    weight = array([p[index.p.weight.position], p[index.p.weight.position], p[index.p.weight.angle]])

    error = find_error(state, p)
    # rospy.logerr(f"fullstate:  {state}, Desired:  {p[:3]}, state:  {state[index.qr ]}")
    # error[2] = anglewrap(error[2])

    # return transpose(weight * error)

    return p[index.p.weight.position] * (error[0]**2) + p[index.p.weight.position] * (error[1]**2) + p[index.p.weight.angle] * error[2]**2
    # return transpose(weight * error)

def find_error(state, p):
    error = p[:3] - state[index.qr]
    error[2] = anglewrap(error[2])
    return error

def anglewrap(theta):
    a = mod(theta + pi, 2 * pi)
    return a - sign(a) * pi
