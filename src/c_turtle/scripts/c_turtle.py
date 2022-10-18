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

        if dirs["right"] == inf and dirs["left"] != inf:
            self.turnLeft(dirs)
        elif dirs["right"] != inf and dirs["left"] == inf:
            self.turnRight(dirs)
        elif dirs["right"] != inf and dirs["left"] != inf:
            if dirs["right"] < dirs["left"]:
                self.turnLeft(dirs)
            else:
                self.turnRight(dirs)
        #  elif dirs["front_right"] > dirs["front_left"]:
            #  self.turnRight(dirs)
        #  elif dirs["front_right"] < dirs["front_left"]:
            #  self.turnLeft(dirs)

    def turnRight(self, dirs):
        self.setAngVel(-CTurtle.maxAngVel * (dirs["right"] - CTurtle.minDistFromWall) / laserRange)

    def turnLeft(self, dirs):
        self.setAngVel(CTurtle.maxAngVel * (dirs["left"] - CTurtle.minDistFromWall) / laserRange)

    def setLinVel(self, linVel):
        self.vel.linear.x = clamp(linVel, -CTurtle.maxLinVel, CTurtle.maxLinVel)

    def setAngVel(self, angVel):
        self.vel.angular.z = clamp(angVel, -CTurtle.maxAngVel, CTurtle.maxAngVel)

    def wiggle(self):
        rospy.loginfo("wiggling")
        self.setLinVel(self.maxLinVel)
        self.setAngVel(random() * 2 if random() > 0.5 else -2)
        # TODO reset wandering angle when limit reached?

    def moveTurtle(self):
        self.pub.publish(self.vel)


if __name__ == "__main__":
    cTurtle = CTurtle()
    while not rospy.is_shutdown():
        pass
