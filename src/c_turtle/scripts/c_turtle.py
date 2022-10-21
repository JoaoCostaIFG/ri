#!/usr/bin/env python3

from random import random, uniform
from math import degrees, radians, inf, isnan, pi, cos, sin, sqrt
from sys import argv

import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from flatland_msgs.srv import MoveModel

LASER_RANGE = 3
LASER_FREQ = 10
RAD22_5 = radians(22.5)
RAD45 = radians(45)
RAD67_5 = radians(67.5)
RAD90 = radians(90)
RAD112_5 = radians(112.5)
RAD135 = radians(135)
RAD157_5 = radians(157.5)


def clamp(val, minVal, maxVal):
    return max(min(val, maxVal), minVal)


class CTurtle:
    maxLinVel = 2.0
    maxAngVel = 3.0
    linAcc = 1.5
    linDec = 3.0
    angAcc = 4.0
    angDec = 4.0

    minDistFromWall = 1.0
    k = 3

    def __init__(self) -> None:
        self.vel = Twist()

        rospy.init_node("CTurtle", anonymous=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def subScan(self):
        rospy.Subscriber("/scan", LaserScan, self._scanCallback)

    def _scanCallback(self, scan):
        lasers = [0] * len(scan.ranges)
        angle = scan.angle_min
        for i in range(len(scan.ranges)):
            dist = scan.ranges[i]
            lasers[i] = (angle, dist if not isnan(dist) else inf)
            angle += scan.angle_increment

        dirs = self._processLasers(lasers, scan.angle_increment)
        self.reactToScan(dirs)

    def _processLasers(self, lasers, angleIncrement):
        angleMin = lasers[0][0]

        dirs = {
            "front": {"dist": inf, "ang": 0},
            "front_left": {"dist": inf, "ang": RAD22_5 * 2},
            "front_right": {"dist": inf, "ang": -RAD22_5 * 2},
            "left": {"dist": inf, "ang": RAD22_5 * 4},
            "right": {"dist": inf, "ang": -RAD22_5 * 4},
            "back_left": {"dist": inf, "ang": RAD22_5 * 6},
            "back_right": {"dist": inf, "ang": -RAD22_5 * 6},
            "back": {"dist": inf, "ang": pi},
            "minDir": "front",
            "edges": {
                "left": inf,
                "front_left": inf,
                "back_left": inf,
            }
        }

        dirs["edges"]["left"] = self._calcWallLen(
            lasers, int((RAD90 - angleMin) / angleIncrement))
        dirs["edges"]["right"] = self._calcWallLen(
            lasers, int((-RAD90 - angleMin) / angleIncrement))

        minDist = inf
        for laser in lasers:
            angle = laser[0]
            dist = laser[1]

            absAngle = abs(angle)
            if absAngle <= RAD22_5:
                key = "front"
            elif absAngle <= RAD67_5:
                key = "front_left" if angle > 0 else "front_right"
            elif absAngle <= RAD112_5:
                key = "left" if angle > 0 else "right"
            elif absAngle <= RAD157_5:
                key = "back_left" if angle > 0 else "back_right"
            else:
                key = "back"

            if abs(angle - RAD22_5) < angleIncrement * 2:
                dirs["edges"]["front_left"] = dist
            elif abs(angle - -RAD22_5) < angleIncrement * 2:
                dirs["edges"]["front_right"] = dist
            elif abs(angle - RAD135) < angleIncrement * 2:
                dirs["edges"]["back_left"] = dist
            elif abs(angle - -RAD135) < angleIncrement * 2:
                dirs["edges"]["back_right"] = dist

            if dist < dirs[key]["dist"]:
                # update sector
                dirs[key]["dist"] = dist
                dirs[key]["ang"] = angle
                if dist < minDist:
                    # update global info
                    dirs["minDir"] = key
                    minDist = dist

        return dirs

    def _calcWallLen(self, lasers, idx):
        if lasers[idx][1] != inf:
            frontPoint = backPoint = lasers[idx]
            i = 1
            while idx + i < len(lasers):
                newBackPoint = lasers[idx + i]
                if newBackPoint[1] != inf:
                    backPoint = newBackPoint
                i += 1
            i = 1
            while idx - i >= 0:
                newFrontPoint = lasers[idx - i]
                if newFrontPoint[1] != inf:
                    frontPoint = newFrontPoint
                i += 1
            frontCoord = (frontPoint[1] * cos(frontPoint[0]),
                          frontPoint[1] * sin(frontPoint[0]))
            backCoord = (backPoint[1] * cos(backPoint[0]),
                         backPoint[1] * sin(backPoint[0]))
            return sqrt(
                (frontCoord[0] - backCoord[0]) ** 2 + (frontCoord[1] - backCoord[1]) ** 2)
        else:
            return inf

    def reactToScan(self, dirs):
        minDir = dirs["minDir"]
        minDist = dirs[minDir]["dist"]
        minAng = dirs[minDir]["ang"]

        if minDist == inf:
            # can't see anything => random walk
            self.wiggle()
            return

        if abs(minDist - CTurtle.minDistFromWall) < 0.1 * CTurtle.k:
            if 1.75 < dirs["edges"]["left"] < 1.9 and dirs["edges"]["back_left"] == inf:
                rospy.logerr("Foi pela esquierda")
                rospy.loginfo(dirs["edges"]["left"])
                self.linVel = 0
                self.angVel = 0
                self.moveTurtle()
                return
            elif 1.78 < dirs["edges"]["right"] < 1.9  and dirs["edges"]["back_right"] == inf:
                rospy.logerr("Foi pela direita")
                rospy.loginfo(dirs["edges"]["right"])
                self.linVel = 0
                self.angVel = 0
                self.moveTurtle()
                return

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
        self.linVel = CTurtle.maxLinVel * front / LASER_RANGE

        if minDir == "front":
            if dirs["left"]["dist"] == inf and dirs["right"]["dist"] != inf:
                # was following wall on right and found obstacle in front => circle obstacle
                wallSide = "right"
            elif dirs["left"]["dist"] != inf and dirs["right"]["dist"] == inf:
                # was following wall on left and found obstacle in front => circle obstacle
                wallSide = "left"
            else:
                # found obstacle in front, which way to turn? The closest
                minLeft = min(dirs["front_left"]["dist"], dirs["left"]["dist"])
                minRight = min(dirs["front_right"]["dist"], dirs["right"]["dist"])
                wallSide = "left" if minLeft < minRight else "right"
        elif minDir.endswith("left"):
            # keep following left
            wallSide = "left"
        else:
            # keep following right
            wallSide = "right"

        if wallSide == "left":
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
        # desiredVel = self.linVel + a * (1/LASER_FREQ)
        a = (desiredVel - self.linVel) * LASER_FREQ
        if a > 0:
            if a <= CTurtle.linAcc:
                self.vel.linear.x = desiredVel
            else:
                # exceeded max acceleration
                self.vel.linear.x = self.linVel + CTurtle.linAcc / LASER_FREQ
        elif a < 0:
            if a >= CTurtle.linDec:
                self.vel.linear.x = desiredVel
            else:
                # exceeded max decelaration
                self.vel.linear.x = self.linVel - CTurtle.linDec / LASER_FREQ

    @property
    def angVel(self):
        return self.vel.angular.z

    @angVel.setter
    def angVel(self, newAngVel):
        desiredVel = clamp(newAngVel, -CTurtle.maxAngVel, CTurtle.maxAngVel)
        self.vel.angular.z = desiredVel
        return

        a = (desiredVel - self.angVel) * LASER_FREQ
        if a > 0:
            if a <= CTurtle.angAcc:
                self.vel.angular.z = desiredVel
            else:
                # exceeded max acceleration
                self.vel.angular.z = self.angVel + CTurtle.angAcc / LASER_FREQ
        elif a < 0:
            if a >= CTurtle.angDec:
                self.vel.angular.z = desiredVel
            else:
                # exceeded max decelaration
                self.vel.angular.z = self.angVel - CTurtle.angDec / LASER_FREQ

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
        moveService("CTurtle", Pose2D(uniform(-6, 6),
                    uniform(-5, 7), uniform(0, 359)))


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
