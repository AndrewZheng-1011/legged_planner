#! /usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import math
import sys


"""
    cmd_vel_pub is a demo file to publish velocity commands to the legged_control repository. 
    Note, that cmd_vel atm uses a fix TIME_TO_TARGET, therefore, it is more of a position command rather than velocity command
"""

# Define global parameters
rospy.init_node('traj_pub_demo', anonymous=True)
# Parameters
period = 20  # Desired period [s]
A = 0.5  # Amplitude
B = 2*math.pi/period
rate_mult = 0.01  # Publish rate a factor of the period

# rate = rospy.Rate(1/(rate_mult*period))  # Hz
rate = rospy.Rate(50)
dt = rate_mult*period  # Discrete time step


def cmd_vel_talker():
    print("Running cmd_vel mode (or position mode w/ varying vel)")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    msg = Twist()
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        curr_time = rospy.Time.now()
        msg.linear.x = 0.0
        msg.linear.y = A*math.sin(B*(curr_time.to_sec() - start_time.to_sec()))
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        pub.publish(msg)
        rate.sleep()


def target_pose_talker():

    print("Running target position mode w/ fixed vel")
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    msg = PoseStamped()

    # Parameters
    target_displacement_vel = 0.5  # TODO (AZ): Read from .info files
    global rate, rate_mult, period
    rate_mult = 0.01
    rate = rospy.Rate(1/(rate_mult*period))
    # rate = rospy.Rate(1)
    # TODO (AZ) : ATM Target Reaching Time is NaN

    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        msg.header.frame_id = "odom"
        curr_time = rospy.Time.now()
        msg.header.stamp = curr_time
        # msg.pose.position.x = A * \
        #     math.cos(B*(curr_time.to_sec()-start_time.to_sec()))
        msg.pose.position.x = 0
        # Move like sine, derivative of sine
        msg.pose.position.y = A * \
            math.sin(B*(curr_time.to_sec()-start_time.to_sec()))
        # msg.pose.position.y = 0.0
        msg.pose.position.z = 0.325  # Go1 spec | Doesn't matter
        # RPY 0 0 0 -> Quaternion 1 0 0 0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        pub.publish(msg)
        rate.sleep()


def main():
    if len(sys.argv) <= 1:
        print(
            "Please rerun and enter w/ type of publishing mode: 'cmd_vel' OR 'target_pose'")
        return
    type_pub = sys.argv[1]
    if type_pub == "cmd_vel":
        cmd_vel_talker()
    elif type_pub == "target_pose":
        target_pose_talker()
    else:
        print("Non-valid publishing mode. Please choose either cmd_vel or target_pose")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
