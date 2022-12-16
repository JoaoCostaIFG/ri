#!/usr/bin/env python3

from random import random, uniform
from math import radians, inf, isnan, pi, cos, sin, sqrt

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from flatland_msgs.srv import MoveModel

LASER_RANGE = 3
LASER_FREQ = 10

STOP_WALL_LEN = 1.95
STOP_TOLERANCE = 0.1
STOP_MIN = STOP_WALL_LEN * (1 - STOP_TOLERANCE)
STOP_MAX = STOP_WALL_LEN * (1 + STOP_TOLERANCE)

RAD22_5 = radians(22.5)
RAD45 = radians(45)
RAD67_5 = radians(67.5)
RAD90 = radians(90)
RAD112_5 = radians(112.5)
RAD135 = radians(135)
RAD157_5 = radians(157.5)


def clamp(val, minVal, maxVal):
    return max(min(val, maxVal), minVal)


def pointDist(p1, p2) -> float:
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def laserToPoint(laser):
    return (laser[1] * cos(laser[0]), laser[1] * sin(laser[0]))

class CTurtle(Node):
    doStop = True
    doOdometry = False

    maxLinVel = 2.0
    maxAngVel = 3.0
    linAcc = 1.5
    linDec = 3.0
    angAcc = 4.0
    angDec = 4.0

    minDistFromWall = 1.0
    k = 3

    def __init__(self) -> None:
        super().__init__("CTurtle")

        # self.doOdometry = self.get_parameter("~do-odometry")
        # self.doStop = self.get_parameter("~do-stop")
        # self.maxLinVel = self.get_parameter("~max-linVel")
        # self.maxAngVel = self.get_parameter("~max-angVel")
        # self.linAcc = self.get_parameter("~linAcc")
        # self.linDec = self.get_parameter("~linDec")
        # self.angAcc = self.get_parameter("~angAcc")
        # self.angDec = self.get_parameter("~angDec")

        self.vel = Twist()

        self.pub:Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        if CTurtle.doOdometry:
            print('"seq","sec","x","y"')
            self.create_subscription(
                Odometry, "/odometry/ground_truth", self._odometryGroundTruth, 1
            )

    def subScan(self):
        self.create_subscription(LaserScan, "/scan", self._scanCallback, 1)

    def _odometryGroundTruth(self, odometry):
        print(
            f"{odometry.header.seq},{odometry.header.stamp.secs + odometry.header.stamp.nsecs / 1000000000},{odometry.pose.pose.position.x},{odometry.pose.pose.position.y}"
        )

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
            },
        }

        dirs["edges"]["left"] = self._calcWallLen(
            lasers, int((RAD90 - angleMin) / angleIncrement)
        )
        dirs["edges"]["right"] = self._calcWallLen(
            lasers, int((-RAD90 - angleMin) / angleIncrement)
        )

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
            firstIdx = lastIdx = idx
            i = 1
            while idx + i < len(lasers):
                newLastIdx = idx + i
                if lasers[newLastIdx][1] != inf:
                    lastIdx = newLastIdx
                i += 1
            i = 1
            while idx - i >= 0:
                newFirstIdx = idx - i
                if lasers[newFirstIdx][1] != inf:
                    firstIdx = newFirstIdx
                i += 1

            ret = 0
            points = map(
                lambda idx: laserToPoint(lasers[idx]), range(firstIdx, lastIdx + 1)
            )
            try:
                p1 = next(points)
                while True:
                    p2 = next(points)
                    ret += pointDist(p1, p2)
                    p1 = p2
            except StopIteration:
                pass
            return ret
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

        if CTurtle.doStop and abs(minDist - CTurtle.minDistFromWall) < 0.1 * CTurtle.k:
            if (
                dirs["edges"]["back_left"] == inf
                and STOP_MIN < dirs["edges"]["left"] < STOP_MAX
            ):
                self.get_logger().info("End left: %f", dirs["edges"]["left"])
                self.linVel = 0
                self.angVel = 0
                self.moveTurtle()
                return
            elif (
                dirs["edges"]["back_right"] == inf
                and STOP_MIN < dirs["edges"]["right"] < STOP_MAX
            ):
                self.get_logger().info("End right: %f", dirs["edges"]["right"])
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
            angDistTerm = cos(pi - minAng) + (minDist - CTurtle.minDistFromWall)
        self.angVel = -CTurtle.k * self.linVel * angDistTerm

        self.moveTurtle()

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
        #  rospy.loginfo("wiggling")
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

        client = self.create_client(MoveModel, "/move_model")
        client.wait_for_service()
        request = MoveModel.Request()
        request.name = "CTurtle"
        request.pose = Pose2D(uniform(-6, 6), uniform(-5, 7), uniform(0, 359))
        client.call(request)


def main(args = None):
    rclpy.init()
    
    cTurtle = CTurtle()

    # if cTurtle.get_parameter("~do-reset"):
    #     cTurtle.reset()

    cTurtle.subScan()
    rclpy.spin(cTurtle)

if __name__ == "__main__":
    main()
