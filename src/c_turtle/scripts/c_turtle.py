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
    maxAngVel = 1.0
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

        # reset v
        self.linVel = 0
        self.angVel = 0

        self.reactToScan(dirs)
        self.moveTurtle()

    def reactToScan(self, dirs):
        if min(dirs.values()) == inf:
            self.wiggle()
            return

        self.linVel = CTurtle.maxLinVel * (dirs["front"] - CTurtle.minDistFromWall) / laserRange

        if dirs["right"] != inf and dirs["front_right"] != inf:
            self.turnLeft(CTurtle.maxAngVel)
        elif dirs["left"] != inf and dirs["front_left"] != inf:
            self.turnRight(CTurtle.maxAngVel)
        elif dirs["right"] < dirs["left"]:
            self.turnRight(CTurtle.maxAngVel)
        elif dirs["right"] > dirs["left"]:
            self.turnLeft(CTurtle.maxAngVel)

    def turnRight(self, right):
        rospy.loginfo("turn right %f", right)
        self.angVel = -right

    def turnLeft(self, left):
        rospy.loginfo("turn left %f", left)
        self.angVel = left

    @property
    def linVel(self):
        return self.vel.linear.x

    @linVel.setter
    def linVel(self, newLinVel):
        self.vel.linear.x = clamp(newLinVel, -CTurtle.maxLinVel, CTurtle.maxLinVel)

    @property
    def angVel(self):
        return self.vel.angular.z

    @angVel.setter
    def angVel(self, newAngVel):
        self.vel.angular.z = clamp(newAngVel, -CTurtle.maxAngVel, CTurtle.maxAngVel)

    def wiggle(self):
        rospy.loginfo("wiggling")
        self.linVel = self.maxLinVel
        if random() > 0.5:
            self.turnRight(random())
        else:
            self.turnLeft(random())
        # TODO reset wandering angle when limit reached?

    def moveTurtle(self):
        self.pub.publish(self.vel)


if __name__ == "__main__":
    cTurtle = CTurtle()
    while not rospy.is_shutdown():
        pass
