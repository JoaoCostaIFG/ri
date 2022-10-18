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
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
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
        left = min(dirs["left"], dirs["front_left"])
        right = min(dirs["right"], dirs["front_right"])
        front = min(dirs["front"], dirs["front_right"], dirs["front_left"])

        rospy.loginfo("%f, %f", left, right)

        if dirs["right"] != inf and self.angVel < 0 and dirs["front_right"] == inf and dirs["back_right"] == inf:
            rospy.logerr("Ended through right")
        elif dirs["left"] != inf and self.angVel > 0 and dirs["front_left"] == inf and dirs["back_left"] == inf:
            rospy.logerr("Ended through left")

        # reset v
        self.linVel = 0
        self.angVel = 0

        if min(dirs.values()) == inf:
            self.wiggle()
            return

        laserWallDist = laserRange - CTurtle.minDistFromWall
        self.linVel = (
            CTurtle.maxLinVel * (front - CTurtle.minDistFromWall) / laserWallDist
        )

        if dirs["right"] != inf and dirs["front_right"] != inf:
            perc = 1 - (right - CTurtle.minDistFromWall) / laserWallDist
            self.turnLeft(CTurtle.maxAngVel * perc)
        elif dirs["left"] != inf and dirs["front_left"] != inf:
            # looking at wall -> look away
            perc = 1 - (left - CTurtle.minDistFromWall) / laserWallDist
            self.turnRight(CTurtle.maxAngVel * perc)
        elif right < left:
            # not looking at nearby wall -> look at it
            perc = (right - CTurtle.minDistFromWall) / laserWallDist
            self.turnRight(CTurtle.maxAngVel * perc)
        elif left < right:
            # not looking at nearby wall -> look at it
            perc = (left - CTurtle.minDistFromWall) / laserWallDist
            self.turnLeft(CTurtle.maxAngVel * perc)

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
        v = CTurtle.maxAngVel * random()
        if random() > 0.5:
            self.angVel += v
        else:
            self.angVel -= v
        # reset wandering angle when limit reached
        if abs(self.angVel) >= CTurtle.maxAngVel:
            self.angVel = 0
        # TODO
        self.angVel = 0

    def moveTurtle(self):
        self.pub.publish(self.vel)


if __name__ == "__main__":
    cTurtle = CTurtle()
    while not rospy.is_shutdown():
        pass
