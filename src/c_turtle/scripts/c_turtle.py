#!/usr/bin/env python3

from random import random
from math import radians, inf, isnan

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

laserRange = 3
laserFreq = 10
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
            if not isnan(dist):
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

        self.reactToScan(dirs)
        self.moveTurtle()

    def reactToScan(self, dirs):
        rospy.loginfo(dirs)
        if min(dirs.values()) == inf:
            self.wiggle()
            return

        self.setLinVel(
            CTurtle.maxLinVel * (dirs["front"] - CTurtle.minDistFromWall) / laserRange
        )

        right = min(dirs["right"], dirs["front_right"])
        left = min(dirs["left"], dirs["front_left"])

        rospy.loginfo("%f %f", right, left)
        if right < left:
            rospy.loginfo("right < left")
            self.adjustRight(right)
        elif right > left:
            rospy.loginfo("left < right")
            self.adjustLeft(left)

    def adjustRight(self, right):
        if right < CTurtle.minDistFromWall:
            v = (
                CTurtle.maxAngVel
                * (CTurtle.minDistFromWall - right)
                / CTurtle.minDistFromWall
            )
        else:
            v = (
                -CTurtle.maxAngVel
                * (right - CTurtle.minDistFromWall)
                / (laserRange - CTurtle.minDistFromWall)
            )
        rospy.loginfo("adjust right %f", v)
        self.setAngVel(v)

    def adjustLeft(self, left):
        if left < CTurtle.minDistFromWall:
            v = (
                -CTurtle.maxAngVel
                * (CTurtle.minDistFromWall - left)
                / CTurtle.minDistFromWall
            )
        else:
            v = (
                CTurtle.maxAngVel
                * (left - CTurtle.minDistFromWall)
                / (laserRange - CTurtle.minDistFromWall)
            )
        rospy.loginfo("adjust left %f", v)
        self.setAngVel(v)

    def setLinVel(self, linVel):
        self.vel.linear.x = clamp(linVel, -CTurtle.maxLinVel, CTurtle.maxLinVel)

    def setAngVel(self, angVel):
        self.vel.angular.z = clamp(angVel, -CTurtle.maxAngVel, CTurtle.maxAngVel)

    def wiggle(self):
        rospy.loginfo("wiggling")
        self.setLinVel(self.maxLinVel)
        #  self.setAngVel(random() * 2 if random() > 0.5 else -2)
        # TODO reset wandering angle when limit reached?

    def moveTurtle(self):
        self.pub.publish(self.vel)


if __name__ == "__main__":
    cTurtle = CTurtle()
    while not rospy.is_shutdown():
        pass
