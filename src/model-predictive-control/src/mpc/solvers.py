from math import inf

import casadi
import numpy as np
import forcespro.nlp
from aerosandbox.numpy import concatenate
from forcespro.nlp.solver import Solver
from forcespro.nlp.symbolicModel import SymbolicModel, CodeOptions

from physics.dynamics import robot_acceleration
from .index import labels, index, big_len
from .objectives import objective_dynamic, objective_dynamic_n



def load_solver():
    return Solver.from_directory("DynamicSolver")


time_lookup = 1
frequency = 10
nNodes = 6

nin = 4
nslack = 1
nstate = 6
nvar = nin + nslack + nstate

model = SymbolicModel(time_lookup * frequency)  # create empty model
model.nvar = nin + nstate + nslack
model.neq = nstate
model.npar = big_len(labels)

model.LSobjective = objective_dynamic
model.LSobjectiveN = objective_dynamic_n

model.continuous_dynamics = lambda x, u: casadi.vertcat(*casadi.vertsplit(x[index.x.qr_dot]), *casadi.vertsplit(robot_acceleration(
    x[index.x.qr],
    x[index.x.qr_dot],
    u
)))
# model.eq = lambda z: forcespro.nlp.integrate(robot_dynamics, z[5:], z[:4],
#                                            integrator=forcespro.nlp.integrators.RK4,
#                                           stepsize=.01)

model.E = np.hstack((np.zeros((nstate, nin + nslack)), np.eye(nstate)))

model.lb = [*(-12 * np.ones(nin)), *(-inf * np.ones(nslack)), *(-inf * np.ones(nstate))]
model.ub = [*(+12 * np.ones(nin)), *(+inf * np.ones(nslack)), *(+inf * np.ones(nstate))]

model.xinitidx = index.fromz



codeoptions = CodeOptions("DynamicSolver")

codeoptions.nlp.integrator.type = 'ERK4'
codeoptions.nlp.integrator.Ts = 1 / frequency
codeoptions.nlp.integrator.nodes = nNodes

codeoptions.maxit = 200
codeoptions.printlevel = 2
codeoptions.optlevel = 2  # 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed

codeoptions.overwrite = 1

# codeoptions.solvemethod = 'PDIP_NLP'
# codeoptions.sqp_nlp.rti = 1
# codeoptions.sqp_nlp.maxSQPit = 10
# codeoptions.nlp.hessian_approximation = 'gauss-newton'

codeoptions.noVariableElimination = 1


def generate_solver():
    return model.generate_solver(codeoptions)
