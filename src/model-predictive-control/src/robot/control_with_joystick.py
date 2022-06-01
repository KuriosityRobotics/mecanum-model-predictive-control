import math

import rospy
from geometry_msgs.msg import Twist, Vector3, TwistStamped, Wrench, WrenchStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Header

MAX_JOYSTICK_VALUE = 2 ** 16 - 1


def calculate_motor_powers(x_mov: float, y_mov: float, turn_mov: float) -> (float, float, float, float):
    return y_mov + turn_mov + x_mov, y_mov - turn_mov - x_mov, y_mov + turn_mov - x_mov, y_mov - turn_mov + x_mov


def __main():
    m1 = rospy.Publisher("fl", Float64, queue_size=1)
    m2 = rospy.Publisher("fr", Float64, queue_size=1)
    m3 = rospy.Publisher("bl", Float64, queue_size=1)
    m4 = rospy.Publisher("br", Float64, queue_size=1)

    joystick_direction = rospy.Publisher("joystick_direction", WrenchStamped, queue_size=1)

    def send_joystick_input(joy: Joy):
        [x_mov, y_mov, turn_mov, *_] = map(lambda v: v / MAX_JOYSTICK_VALUE, joy.axes)
        (fl, fr, bl, br) = calculate_motor_powers(x_mov, y_mov, turn_mov)
        fl *= 12 * 2
        fr *= 12 * 2
        bl *= 12 * 2
        br *= 12 * 2

        if joy.axes[4] > 1:
            m1.publish(fl)
            m2.publish(fr)
            m3.publish(bl)
            m4.publish(br)

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
