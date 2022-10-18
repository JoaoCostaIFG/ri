#!/usr/bin/env python3

from random import random
from math import radians, inf, isnan, pi, sin

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
    k = 3

    def __init__(self) -> None:
        self.vel = Twist()

        rospy.init_node("CTurtle", anonymous=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scanCallback)

    def scanCallback(self, scan):
        hit = {"dir": "front", "dist": inf, "ang": 0}

        angle = scan.angle_min
        for dist in scan.ranges:
            if dist < hit["dist"]:
                hit["dist"] = dist
                hit["ang"] = angle

                absAngle = abs(angle)
                if absAngle <= rad225:
                    hit["dir"] = "front"
                elif absAngle <= rad225 * 3:
                    hit["dir"] = "front_left" if angle > 0 else "front_right"
                elif absAngle <= rad225 * 5:
                    hit["dir"] = "left" if angle > 0 else "right"
                elif absAngle <= rad225 * 7:
                    hit["dir"] = "back_left" if angle > 0 else "back_right"
                else:
                    hit["dir"] = "back"

            angle += scan.angle_increment

        self.reactToScan(hit)
        self.moveTurtle()

    def reactToScan(self, hit):
        # reset v
        self.linVel = 0
        self.angVel = 0

        if hit["dist"] == inf:
            # can't see anything
            self.wiggle()
            return

        #  if wallDirection == "right":
        #  front = min(dirs["front"], dirs["front_left"])
        #  elif wallDirection == "left":
        #  front = min(dirs["front"], dirs["front_right"])
        #  else:
        #  front = min(dirs["front"], dirs["front_right"], dirs["front_left"])
        #  self.linVel = CTurtle.maxLinVel * front / laserRange
        self.linVel = 1.0

        self.angVel = (
            -CTurtle.k
            * self.linVel
            * (sin(pi / 2 - hit["ang"]) - (hit["dist"] - CTurtle.minDistFromWall))
        )

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
