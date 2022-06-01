from aerosandbox.numpy import *

from .motor import orbital_20_gearmotor
from .static_parameters import *


def robot_acceleration(qr, qr_dot, motor_voltages):
    heading = qr[2]
    heading_vel = qr_dot[2]

    motor_voltages = motor_voltages[:4].reshape((4, 1))
    qr = qr.reshape((3, 1))
    qr_dot = qr_dot.reshape((3, 1))

    R = (1 / r) * array([[1, -1, -Ll], [1, 1, Ll], [1, 1, -Ll], [1, -1, Ll]], dtype=float)

    undo_rotation = transpose(rotation_matrix(heading))

    wheel_speeds = R @ undo_rotation @ qr_dot
    wheel_torques = orbital_20_gearmotor.torque(
        motor_voltages[:4].astype(float) if isinstance(motor_voltages, np.ndarray) else motor_voltages[:4],
        wheel_speeds)
    forces = robot_force(wheel_torques, heading)

    H = diag([m, m, Iz]) + (4 * Iw / r ** 2) * diag([1, 1, Ll ** 2])
    K = (4 * Iw / r ** 2) * heading_vel * array([[0, 1, 0], [-1, 0, 0], [0, 0, 0]])
    qr_ddot = linalg.inv(H) @ (forces - (K @ qr_dot))

    return qr_ddot.reshape((3,1))


def rotation_matrix(heading):
    return array([[cos(heading), -sin(heading), 0],
                  [sin(heading), cos(heading), 0],
                  [0, 0, 1]], dtype=(float if isinstance(heading, ndarray) else None))


def robot_force(torque, heading):
    return (1 / r) * array([
        sin(heading) * (torque[0] - torque[1] - torque[2] + torque[3]) + cos(heading) * (
                torque[0] + torque[1] + torque[2] + torque[3]),
        sin(heading) * (torque[0] + torque[1] + torque[2] + torque[3]) - cos(heading) * (
                torque[0] - torque[1] - torque[2] + torque[3]),
        -Ll * (torque[0] - torque[1] + torque[2] - torque[3])])


if __name__ == '__main__':
    state = array([0, 0., 0., 0., 0., 0.], dtype=float64).reshape((6, 1))
    for i in range(10):
        state = state + .1 * robot_acceleration(state, array([12., 12., 12., 12.]).reshape((4, 1)))

    print(state)
