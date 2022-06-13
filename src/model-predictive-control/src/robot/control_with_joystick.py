import math

import rospy
from geometry_msgs.msg import Twist, Vector3, TwistStamped, Wrench, WrenchStamped, PoseWithCovarianceStamped, \
    PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Header, Empty

MAX_JOYSTICK_VALUE = 1


def calculate_motor_powers(x_mov: float, y_mov: float, turn_mov: float) -> (float, float, float, float):
    results = y_mov + turn_mov + x_mov, y_mov - turn_mov - x_mov, y_mov + turn_mov - x_mov, y_mov - turn_mov + x_mov
    max_voltage = abs(max(results))
    if max_voltage > 12:
        results = (v * 12 / max_voltage for v in results) # scale down

    return results

DEAD_ZONE = .05

def __main():
    m1 = rospy.Publisher("fl", Float64, queue_size=1)
    m2 = rospy.Publisher("fr", Float64, queue_size=1)
    m3 = rospy.Publisher("bl", Float64, queue_size=1)
    m4 = rospy.Publisher("br", Float64, queue_size=1)

    joystick_direction = rospy.Publisher("joystick_direction", WrenchStamped, queue_size=1)

    odometry_pose = rospy.Publisher("odom_pose", PoseWithCovarianceStamped, queue_size=1)

    last_odom = Odometry()
    def odom_callback(odom):
        nonlocal last_odom
        last_odom = odom
    odometry = rospy.Subscriber("odom", Odometry, queue_size=1, callback=odom_callback)

    mpc_toggle = rospy.Publisher("mpc_toggle", Empty, queue_size=1)
    goal = rospy.Publisher("goal", PoseStamped)

    def send_joystick_input(joy: Joy):
        [x_mov, y_mov, _, turn_mov, *_] = map(lambda v: v / MAX_JOYSTICK_VALUE, joy.axes)
        x_mov = -x_mov
        turn_mov = -turn_mov
        (fl, fr, bl, br) = map(lambda a: 0 if abs(a) < DEAD_ZONE else a, calculate_motor_powers(x_mov, y_mov, turn_mov))
        fl *= 12
        fr *= 12
        bl *= 12
        br *= 12

        m1.publish(fl)
        m2.publish(fr)
        m3.publish(bl)
        m4.publish(br)

        if joy.buttons[2]:
            odometry_pose.publish(PoseWithCovarianceStamped())

        if joy.buttons[4]:
            goal.publish(PoseStamped(last_odom.header, last_odom.pose.pose))


        if joy.buttons[5]:
            mpc_toggle.publish(Empty())

        twist = WrenchStamped(Header(None, None, "map"), Wrench(force=Vector3(x_mov, y_mov, 0), torque=Vector3(0, 0, -turn_mov)))
        joystick_direction.publish(twist)

    rospy.Subscriber("joy", Joy, queue_size=10, callback=send_joystick_input)
    while not rospy.is_shutdown():
        rospy.spin()


def main():
    try:
        rospy.init_node("Joystick driver")
        __main()
    finally:
        rospy.signal_shutdown("Shutting down...")


if __name__ == '__main__':
    main()
