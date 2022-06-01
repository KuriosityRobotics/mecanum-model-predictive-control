import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from numpy import clip
from std_msgs.msg import Float64, Header
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from .robot import Robot


def __main():
    robot = Robot()

    m1 = rospy.Subscriber("fl", Float64, callback=lambda x: robot.set_motor(1, min(max(x.data, -12), 12)))
    m2 = rospy.Subscriber("fr", Float64, callback=lambda x: robot.set_motor(2, min(max(x.data, -12), 12)))
    m3 = rospy.Subscriber("bl", Float64, callback=lambda x: robot.set_motor(3, min(max(x.data, -12), 12)))
    m4 = rospy.Subscriber("br", Float64, callback=lambda x: robot.set_motor(4, min(max(x.data, -12), 12)))

    odometry = rospy.Publisher("odom", Odometry, queue_size=1)

    def set_robot_pose(target: PoseWithCovarianceStamped):
        robot.position[0] = target.pose.pose.position.x
        robot.position[1] = target.pose.pose.position.y
        robot.position[2] = euler_from_quaternion([target.pose.pose.orientation.x, target.pose.pose.orientation.y, target.pose.pose.orientation.z, target.pose.pose.orientation.w])[2]
        print(f"Reset to:  {robot.position}")

    odometry_pose = rospy.Subscriber("odom_pose", PoseWithCovarianceStamped, callback=set_robot_pose)

    rate = rospy.Rate(500)
    i = 0
    while not rospy.is_shutdown():
        robot.tick(rate.sleep_dur.to_sec())

        if i % 10 == 0: # every 20 ticks
            packet = robot.get_odometry()
            odometry.publish(packet)

        i += 1
        rate.sleep()


def main():
    try:
        rospy.init_node("Drivetrain simulator")
        __main()
    finally:
        rospy.signal_shutdown("Shutting down...")


if __name__ == '__main__':
    main()
