import rospy
from sensor_msgs.msg import Joy
from inputs import get_gamepad, devices, GamePad, XinputState, XinputGamepad


def __main():
    joy = rospy.Publisher("joy", Joy, queue_size=10)
    rate = rospy.Rate(50)
    gamepad: GamePad = devices.gamepads[0]
    while not rospy.is_shutdown():
        state: XinputGamepad = gamepad._GamePad__read_device().gamepad
        event = Joy(None, [state.l_thumb_x, state.l_thumb_y, state.r_thumb_x, state.r_thumb_y, state.left_trigger,
                           state.right_trigger], list(map(int, bin(state.buttons)[2:])))
        joy.publish(event)
        rate.sleep()


def main():
    try:
        rospy.init_node("Joystick")
        __main()
    finally:
        rospy.signal_shutdown("Shutting down...")

if __name__ == "__main__":
    main()