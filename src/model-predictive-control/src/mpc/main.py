import math
import time

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry, Path
from numpy import ndarray
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float64MultiArray, Header, Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from mpc.objectives import find_error
from . import solvers

from .solvers import *


def pp_state(state, voltage):
    [x, y, theta, xdot, ydot, thetadot] = state
    [m1, m2, m3, m4] = voltage
    print(
        f'[{m1:6.2f}, {m2:6.2f}, {m3:6.2f}, {m4:6.2f}] V, <{x:6.2f} m, {y:6.2f} m, {math.degrees(theta):6.2f} deg>, <{xdot:6.2f} m/s, {ydot:6.2f} m/s, {math.degrees(thetadot):6.2f} deg/s>')


def __main():
    x = np.zeros((6, 1))

    def set_state(odometry: Odometry):
        x[0] = odometry.pose.pose.position.x
        x[1] = odometry.pose.pose.position.y
        x[2] = euler_from_quaternion(
            [odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z,
             odometry.pose.pose.orientation.w])[2]

        x[3] = odometry.twist.twist.linear.x
        x[4] = odometry.twist.twist.linear.y
        x[5] = odometry.twist.twist.angular.z

    rospy.Subscriber("odom", Odometry, callback=set_state)

    error = rospy.Publisher("error", Float64MultiArray, queue_size=10)
    objective_sum = rospy.Publisher("objective_sum", Float64, queue_size=10)

    m1 = rospy.Publisher("fl", Float64, queue_size=1)
    m2 = rospy.Publisher("fr", Float64, queue_size=1)
    m3 = rospy.Publisher("bl", Float64, queue_size=1)
    m4 = rospy.Publisher("br", Float64, queue_size=1)
    path = rospy.Publisher("path", Path, queue_size=1)

    solve_status = rospy.Publisher("diagnostics", DiagnosticArray, queue_size=10)

    np.set_printoptions(precision=3, suppress=True)
    solver = solvers.load_solver()

    # Set initial condition
    xinit = np.transpose(np.array([0, 5, 0, 0.1, 0.1, 0]))

    x0i = np.zeros((nvar, 1))
    x0i[:, 0] = [0, 0, 0, 0, 0, *xinit]
    x0 = np.transpose(np.tile(x0i, (1, model.N)))

    p = np.zeros(model.npar)
    p[index.p.objective.x] = 0
    p[index.p.objective.y] = 0
    p[index.p.objective.theta] = 0
    p[index.p.weight.position] = 20
    p[index.p.weight.angle] = 5
    p[index.p.weight.energy] = .00001
    p[index.p.weight.slack] = 0
    p[index.p.weight.strafe] = 0
    p[index.p.weight.use_begin] = 0

    def set_target_pose(target: PoseStamped):
        p[index.p.objective.x] = target.pose.position.x
        p[index.p.objective.y] = target.pose.position.y
        p[index.p.objective.theta] = euler_from_quaternion([target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w])[2]
    rospy.Subscriber("goal", PoseStamped, callback=set_target_pose)

    problem = {"x0": x0,
               "xinit": xinit,
               "all_parameters": np.transpose(np.tile(p, (1, model.N)))}

    do_mpc = False
    def mpc_toggle(_):
        nonlocal do_mpc
        do_mpc = not do_mpc
    rospy.Subscriber("mpc_toggle", Empty, queue_size=10, callback=mpc_toggle)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        problem["x0"][:, index.fromz] = np.transpose(x.reshape((-1,)))
        problem["all_parameters"] = np.transpose(np.tile(p, (1, model.N)))

        problem["xinit"] = np.transpose(x.reshape((-1,)))
        output, exitflag, info = solver.solve(problem)

        z = np.array([0, 0, 0, 0, 0, *x.reshape((-1,))])
        assert problem["x0"].shape == (model.N, model.nvar), f"Expected x0 to have shape {[model.N, model.nvar]}, but it had {problem['x0'].shape}."

        score = objective_dynamic_n(output["x01"], p)
        assert not ((np.isnan(score)) or (np.isinf(score)))
        # objective.publish(Floa
        # t64MultiArray(data=score))
        objective_sum.publish(score)

        err = find_error(output["x01"], p)
        err[2] = math.degrees(err[2])
        error.publish(data=err)

        poses = [PoseStamped(Header(None, None, "map"), Pose(Point(v[index.qr[0]], v[index.qr[1]], 0), Quaternion(*quaternion_from_euler(0, 0, v[index.qr[2]])))) for k, v in output.items() if k[0] == 'x']
        curr_path = Path(Header(None, None, "map"), poses)
        path.publish(curr_path)

        if not do_mpc:
            rate.sleep()
            continue

        [fl, fr, bl, br, *_] = output["x01"]
        m1.publish(fl)
        m2.publish(fr)
        m3.publish(bl)
        m4.publish(br)

        status = DiagnosticStatus()
        status_arr = [status]
        if exitflag == 1:
            status.level = 0
        elif exitflag < 0:
            rospy.logerr(f"Did not converge:  {exitflag}")
            print("objective ", solver.objective(z, p=p))
            status.level = 2
        else:
            rospy.logerr(f"Did not converge:  {exitflag}")
            status.level = 1

        status.name = "Solver output"
        status.message = f"{exitflag:3}:  FL = {fl:.2f} V, FR = {fr:.2f}, BL = {bl:.2f}, BR = {br:.2f}"
        status.hardware_id = "MPC"

        solve_status.publish(DiagnosticArray(status=status_arr))
        rate.sleep()


CRASH_IF_WRONG = False


def assert_valid(function, z, p, s, status_arr, crash_if_wrong=CRASH_IF_WRONG):
    a, b = function(z, p=p, stage=s)

    if not isinstance(a, ndarray):
        a = np.array(a)
    if not isinstance(b, ndarray):
        b = np.array(b)
    a = a.reshape((-1,))
    b = b.reshape((-1,))

    def err_array():
        return [*[KeyValue(f"{function.__name__} result 1, var {i}", str(v)) for i, v in enumerate(a)],
                *[KeyValue(f"{function.__name__} Result 2, var {i}", str(v)) for i, v in enumerate(b)]]

    a_res = np.isnan(a).any() or np.isinf(b).any()
    if a_res:
        rospy.logerr(f"Nan or inf in {function}:  {a.reshape((-1,))} at state {s}")
        status_arr.append(DiagnosticStatus(1, f"NaN/inf in {function.__name__}", f"Result 1. {a}", "MPC", err_array()))

    if b_res := np.isnan(b).any() or np.isinf(b).any():
        rospy.logerr(f"Nan or inf in {function}:  {b.reshape((-1,))} at stage {s}")
        status_arr.append(DiagnosticStatus(1, f"NaN/inf in {function.__name__}", f"Result 2. {b}", "MPC", err_array()))

    if not a_res and not b_res:
        status_arr.append(DiagnosticStatus(0, f"NaN/inf in {function.__name__}", "OK", "MPC", err_array()))

    if crash_if_wrong:
        assert not a_res, f"Nan or inf in {function}:  {a.reshape((-1,))} at state {s}"
        assert not b_res, f"Nan or inf in {function}:  {b.reshape((-1,))} at stage {s}"

    return a_res or b_res


def main():
    try:
        rospy.init_node("MPC controller")
        __main()
    finally:
        rospy.signal_shutdown("shutting down...")


if __name__ == '__main__':
    main()
