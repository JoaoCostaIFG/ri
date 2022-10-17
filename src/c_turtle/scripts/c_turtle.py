#!/usr/bin/env python3

from random import random
from math import radians, inf

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rad225 = radians(22.5)


def clamp(val, minVal, maxVal):
    return max(min(val, maxVal), minVal)


class CTurtle:
    maxLinVel = 2.0
    maxAngVel = 2.0
    maxLinAcc = 1.5
    maxLinDec = 0.7
    minDistFromWall = 1.0

    def __init__(self) -> None:
        self.vel = Twist()

        rospy.init_node("CTurtle", anonymous=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
        rospy.Subscriber("/scan", LaserScan, self.scanCallback)

    def scanCallback(self, scan):
        # TODO
        rospy.loginfo(scan.ranges)
        dirs = {
            "back": inf,
            "back_right": inf,
            "right": inf,
            "front_right": inf,
            "front": inf,
            "front_left": inf,
            "left": inf,
            "back_left": inf,
        }

        angle = scan.angle_min
        for dist in scan.ranges:
            absAngle = abs(angle)
            if absAngle <= rad225:
                key = "front"
            elif absAngle <= rad225 * 3:
                key = "front_left" if angle > 0 else "front_right"
            elif absAngle <= rad225 * 5:
                key = "left" if angle > 0 else "right"
            elif absAngle <= rad225 * 7:
                key = "back_left" if angle > 0 else "back_right"
            else:
                key = "back"

            dirs[key] = min(dist, dirs[key])
            angle += scan.angle_increment

        rospy.loginfo(dirs)

    def setLinVel(self, linVel):
        self.vel.linear.x = clamp(linVel, -CTurtle.maxLinVel, CTurtle.maxLinVel)

    def setAngVel(self, angVel):
        self.vel.angular.z = clamp(angVel, -CTurtle.maxAngVel, CTurtle.maxAngVel)

    def wiggle(self):
        self.setLinVel(self.maxLinVel)
        self.setAngVel(
            self.vel.angular.z + (random() if random() > 0.5 else -random()) / 10
        )
        self.moveTurtle()

    def moveTurtle(self):
        self.pub.publish(self.vel)


if __name__ == "__main__":
    cTurtle = CTurtle()
    cTurtle.setAngVel(1)
    while not rospy.is_shutdown():
        # cTurtle.wiggle()
        # print(rospy.get_time())
        #  cTurtle.moveTurtle()
        cTurtle.wiggle()
        pass
