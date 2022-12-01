#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def turtlePoseCallback(pose):
    rospy.loginfo("Robot X = %f, Y = %f, Z = %f.", pose.x, pose.y, pose.theta)

def turtleMove(linVel, angVelFactor):
    rospy.init_node('moveTurtle', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, turtlePoseCallback)

    vel = Twist()
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    t = 0
    rateHz = 10
    rate = rospy.Rate(rateHz)
    while not rospy.is_shutdown():
        vel.linear.x = linVel
        vel.linear.y = 0
        vel.linear.z = 0

        t += 1 / rateHz
        angVel = t / angVelFactor
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = angVel

        rospy.loginfo("Linear vel = %f. Angular vel = %f.", linVel, angVel)
        pub.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        turtleMove(1, 10)
    except rospy.ROSInterruptException:
        pass
