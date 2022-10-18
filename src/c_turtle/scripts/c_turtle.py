#!/usr/bin/env python3

from random import random
from math import radians, inf, isnan, pi, sin, cos, degrees

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

laserRange = 3
laserFreq = 10
rad225 = radians(22.5)


def clamp(val, minVal, maxVal):
    return max(min(val, maxVal), minVal)


class CTurtle:
    clockwise = True

    maxLinVel = 2.0
    maxAngVel = 2.0
    linAcc = 1.5
    linDec = 1.3
    angAcc = 40.0
    angDec = 40.0

    minDistFromWall = 1.0
    k = 3

    def __init__(self) -> None:
        self.vel = Twist()

        rospy.init_node("CTurtle", anonymous=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scanCallback)

    def scanCallback(self, scan):
        dirs = {
            "front": {"dist": inf, "ang": 0},
            "front_left": {"dist": inf, "ang": rad225 * 2},
            "front_right": {"dist": inf, "ang": -rad225 * 2},
            "left": {"dist": inf, "ang": rad225 * 4},
            "right": {"dist": inf, "ang": -rad225 * 4},
            "back_left": {"dist": inf, "ang": rad225 * 6},
            "back_right": {"dist": inf, "ang": -rad225 * 6},
            "back": {"dist": inf, "ang": pi},
        }

        minDir = "front"
        minDist = inf
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

                if dist < dirs[key]["dist"]:
                    # update sector
                    dirs[key]["dist"] = dist
                    dirs[key]["ang"] = angle
                    if dist < minDist:
                        # update global info
                        minDir = key
                        minDist = dist

            # loop increment
            angle += scan.angle_increment

        self.reactToScan(dirs, minDir)

    def reactToScan(self, dirs, minDir):
        # reset v
        self.linVel = 0
        self.angVel = 0

        if dirs[minDir]["dist"] == inf:
            # can't see anything => random walk
            self.wiggle()
            return

        if dirs["right"]["dist"] != inf and dirs["front_right"]["dist"] == inf and dirs["back_right"]["dist"] == inf:
            rospy.logerr("Right end")
        elif dirs["left"]["dist"] != inf and dirs["front_left"]["dist"] == inf and dirs["back_left"]["dist"] == inf:
            rospy.logerr("Left end")

        if minDir == "right" or minDir == "front_right":
            front = min(dirs["front"]["dist"], dirs["front_left"]["dist"])
        elif minDir == "left" or minDir == "front_left":
            front = min(dirs["front"]["dist"], dirs["front_right"]["dist"])
        else:
            front = min(
                dirs["front"]["dist"],
                dirs["front_right"]["dist"],
                dirs["front_left"]["dist"],
            )
        self.linVel = CTurtle.maxLinVel * front / laserRange

        if CTurtle.clockwise:
            angDistTerm = cos(dirs[minDir]["ang"]) + (
                CTurtle.minDistFromWall - dirs[minDir]["dist"]
            )
        else:
            angDistTerm = cos(pi - dirs[minDir]["ang"]) + (
                dirs[minDir]["dist"] - CTurtle.minDistFromWall
            )

        self.angVel = -CTurtle.k * self.linVel * angDistTerm

        self.moveTurtle()

    def turnRight(self, right):
        rospy.loginfo("turn right %f", right)
        self.angVel += -right

    def turnLeft(self, left):
        rospy.loginfo("turn left %f", left)
        self.angVel += left

    @property
    def linVel(self):
        return self.vel.linear.x

    @linVel.setter
    def linVel(self, newLinVel):
        desiredVel = clamp(newLinVel, -CTurtle.maxLinVel, CTurtle.maxLinVel)

        # v = v0 + a * t
        # desiredVel = self.linVel + a * (1/laserFreq)
        a = (desiredVel - self.linVel) * laserFreq
        if a > 0:
            if a <= CTurtle.linAcc:
                self.vel.linear.x = desiredVel
            else:
                # exceeded max acceleration
                self.vel.linear.x = self.linVel + CTurtle.linAcc / laserFreq
        elif a < 0:
            if a >= CTurtle.linDec:
                self.vel.linear.x = desiredVel
            else:
                # exceeded max decelaration
                self.vel.linear.x = self.linVel - CTurtle.linDec / laserFreq

    @property
    def angVel(self):
        return self.vel.angular.z

    @angVel.setter
    def angVel(self, newAngVel):
        desiredVel = clamp(newAngVel, -CTurtle.maxAngVel, CTurtle.maxAngVel)
        # TODO
        self.vel.angular.z = desiredVel
        return

        a = (desiredVel - self.angVel) * laserFreq
        if a > 0:
            if a <= CTurtle.angAcc:
                self.vel.angular.z = desiredVel
            else:
                # exceeded max acceleration
                self.vel.angular.z = self.angVel + CTurtle.angAcc / laserFreq
        elif a < 0:
            if a >= CTurtle.angDec:
                self.vel.angular.z = desiredVel
            else:
                # exceeded max decelaration
                self.vel.angular.z = self.angVel - CTurtle.angDec / laserFreq

    def wiggle(self):
        rospy.loginfo("wiggling")
        self.linVel = self.maxLinVel
        v = CTurtle.maxAngVel * random()
        if random() > 0.5:
            self.angVel += v
        else:
            self.angVel -= v
        # reset wandering angle when limit reached
        if abs(self.angVel) >= CTurtle.maxAngVel:
            self.angVel = 0

        self.moveTurtle()

    def moveTurtle(self):
        self.pub.publish(self.vel)


if __name__ == "__main__":
    cTurtle = CTurtle()
    while not rospy.is_shutdown():
        pass
