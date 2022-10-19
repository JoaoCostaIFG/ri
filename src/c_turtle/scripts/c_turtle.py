#!/usr/bin/env python3

from random import random, uniform
from math import degrees, radians, inf, isnan, pi, cos, sin
from sys import argv

import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from flatland_msgs.srv import MoveModel

laserRange = 3
laserFreq = 10
rad225 = radians(22.5)


def clamp(val, minVal, maxVal):
    return max(min(val, maxVal), minVal)


class CTurtle:
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

    def subScan(self):
        rospy.Subscriber("/scan", LaserScan, self.scanCallback)

    def scanCallback(self, scan):
        lasers = [0] * len(scan.ranges)
        angle = scan.angle_min
        for i in range(len(scan.ranges)):
            dist = scan.ranges[i]
            lasers[i] = (angle, dist if not isnan(dist) else inf)
            angle += scan.angle_increment

        dirs = {
            "front": {"dist": inf, "ang": 0},
            "front_left": {"dist": inf, "ang": rad225 * 2},
            "front_right": {"dist": inf, "ang": -rad225 * 2},
            "left": {"dist": inf, "ang": rad225 * 4},
            "right": {"dist": inf, "ang": -rad225 * 4},
            "back_left": {"dist": inf, "ang": rad225 * 6},
            "back_right": {"dist": inf, "ang": -rad225 * 6},
            "back": {"dist": inf, "ang": pi},
            "edges": {
                "front": inf,
                "left": inf,
                "front_left": inf,
                "back_left": inf,
                "right": inf,
                "front_right": inf,
                "back_right": inf,
            }
        }

        minDir = "front"
        minDist = inf
        for laser in lasers:
            angle = laser[0]
            dist = laser[1]

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

            if angle < scan.angle_increment * 2:
                dirs["edges"]["front"] = min(dirs["edges"]["front"], dist)
            elif abs(angle - rad225 * 2) < scan.angle_increment * 2:
                dirs["edges"]["front_left"] = min(dirs["edges"]["front_left"], dist)
            elif abs(angle - -rad225 * 2) < scan.angle_increment * 2:
                dirs["edges"]["front_right"] = min(dirs["edges"]["front_right"], dist)
            elif abs(angle - rad225 * 4) < scan.angle_increment * 2:
                dirs["edges"]["left"] = min(dirs["edges"]["left"], dist)
            elif abs(angle - -rad225 * 4) < scan.angle_increment * 2:
                dirs["edges"]["right"] = min(dirs["edges"]["right"], dist)
            elif abs(angle - rad225 * 6) < scan.angle_increment * 2:
                dirs["edges"]["back_left"] = min(dirs["edges"]["back_left"], dist)
            elif abs(angle - -rad225 * 6) < scan.angle_increment * 2:
                dirs["edges"]["back_right"] = min(dirs["edges"]["back_right"], dist)

            if dist < dirs[key]["dist"]:
                # update sector
                dirs[key]["dist"] = dist
                dirs[key]["ang"] = angle
                if dist < minDist:
                    # update global info
                    minDir = key
                    minDist = dist

        self.reactToScan(dirs, minDir)

    def reactToScan(self, dirs, minDir):
        # reset v
        self.linVel = 0
        self.angVel = 0

        minDist = dirs[minDir]["dist"]
        minAng = dirs[minDir]["ang"]

        if minDist == inf:
            # can't see anything => random walk
            self.wiggle()
            return

        if dirs["edges"]["left"] == inf and dirs["edges"]["back_left"] == inf and dirs["edges"]["front"] == inf and dirs["edges"]["front_left"] != inf:
            rospy.loginfo(dirs["edges"])
            rospy.logerr("Foi pela esquierda")
        #elif dirs["edges"]["right"] != inf and dirs["edges"]["front_right"] == inf and dirs["edges"]["back_right"] == inf:
        #    rospy.loginfo(dirs["edges"])
        #    rospy.logerr("Foi pela direita")

        if minDir.endswith("right"):
            front = min(dirs["front"]["dist"], dirs["front_left"]["dist"])
        elif minDir.endswith("left"):
            front = min(dirs["front"]["dist"], dirs["front_right"]["dist"])
        else:
            front = min(
                dirs["front"]["dist"],
                dirs["front_right"]["dist"],
                dirs["front_left"]["dist"],
            )
        self.linVel = CTurtle.maxLinVel * front / laserRange

        if minDir == "front":
            minLeft = min(dirs["front_left"]["dist"], dirs["left"]["dist"])
            minRight = min(dirs["front_right"]["dist"], dirs["right"]["dist"])
            if minLeft < minRight:
                turnDir = "left"
            else:
                turnDir = "right"
        elif minDir.endswith("left"):
            turnDir = "left"
        else:
            turnDir = "right"


        if turnDir == "left":
            angDistTerm = cos(minAng) + (CTurtle.minDistFromWall - minDist)
        else:
            angDistTerm = cos(pi - minAng) + \
                (minDist - CTurtle.minDistFromWall)
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

    def reset(self):
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        self.pub.publish(vel)

        moveService = rospy.ServiceProxy("/move_model", MoveModel)
        moveService("CTurtle", Pose2D(uniform(-9, 8),
                    uniform(-4, 14), uniform(0, 359)))


if __name__ == "__main__":
    cTurtle = CTurtle()
    if len(argv) > 1:
        if argv[1] == "reset":
            cTurtle.reset()
        elif argv[1] == "follow_wall":
            cTurtle.subScan()
            rospy.spin()
    else:
        cTurtle.reset()
        cTurtle.subScan()
        rospy.spin()
