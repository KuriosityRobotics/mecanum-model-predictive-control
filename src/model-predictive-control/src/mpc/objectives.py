from math import pi

import rospy
from aerosandbox.numpy import *
from casadi import atan2, mod, sign, casadi, vertcat

from .index import index


def objective_dynamic(state, p):
    qr = state[index.qr]
    qr_dot = state[index.qr_dot]
    angle_error = qr[2] - atan2(qr_dot[1], qr_dot[0])
    angle_error = anglewrap(angle_error)

    # return (p[index.p.weight.slack] * state[index.slack]) ** 2 + \
    #        (p[index.p.weight.strafe] * angle_error) ** 2 \
    #        + p[index.p.weight.use_begin] * objective_dynamic_n(state, p)

    return vertcat(p[index.p.weight.slack] * state[index.slack],
                 # p[index.p.weight.strafe] * angle_error, # DO NOT UNCOMMENT THIS FUCKS SHIT UP
                   *casadi.horzsplit(p[index.p.weight.energy] * state[index.u]),
                   *casadi.horzsplit(p[index.p.weight.use_begin] * objective_dynamic_n(state, p))
                   )

def objective_dynamic_n(state, p):
    weight = array([p[index.p.weight.position], p[index.p.weight.position], p[index.p.weight.angle]])

    error = p[:3] - state[index.qr]
    # rospy.logerr(f"fullstate:  {state}, Desired:  {p[:3]}, state:  {state[index.qr ]}")
    #error[2] = anglewrap(error[2])

    return weight * error


def anglewrap(theta):
    a = mod(theta + pi, 2 * pi)
    return a - sign(a) * pi
