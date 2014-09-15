#!/usr/bin/env python

# Written by Eric Schneider for the CompRobo warmup project
# 9/14/2014
#
# Stuff about how it works...
#
# In the code below I couldn't get the input_keys methods to apprpriately
# pass a class object. The sm_robot class was either not passed into the
# states, or it was passed as a constant. I left the code in there for
# documentation and in case I could get it working later


import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from copy import deepcopy
from math import pi, cos, sin, atan2, copysign


class NeatoFollower():
    def __init__(self):
        self.max_linear = 0.3
        self.min_linear = 0.0
        self.max_angular = 0.3
        self.min_angular = 0.0

        self.last_scan = LaserScan()
        self.scan_hist = []
        self.hist_length = 10
        self.valid_points = {}
        self.valid_hist = []

        # Computed by /odom
        self.last_pos
        self.odom_hist = []

        # Has a moving object been detected?
        self.movement_detected = False
        # If movement has been detected, where is it w/ reference to the robot?
        self.movement_position = (0.0, 0.0)

        # Has a wall been detected?
        self.wall_detected = True
        # Coordinates and angle of closest wall
        self.closest_wall = Twist()

        # At 0.5 meters, the sensitivity will be zero
        self.obstacle_sensitivity = 0.5

        laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        # odom_sub = rospy.Subscriber('/odom', )
        vel_pub = rospy.Publisher('/cmd_vel', Twist)

    def laser_callback(self, msg):
        self.last_scan = deepcopy(msg)
        self.compute_valid()
        self.scan_hist.append(self.last_scan)
        self.odom_hist.append(self.last_pos)
        while len(scan_hist) > self.hist_length:
            self.scan_hist.pop(0)
        while len(odom_hist) > self.hist_length:
            self.odom_hist.pop(0)

        self.detect_movement()
        self.detect_walls

    def odom_callback(self, msg):
        self.last_pos = deepcopy(msg)

    def detect_movement(self):
        # ang = self.find_point_changes()
        self.movement_detected = True

    def detect_walls(self, msg):
        self.wall_detected = False

    # Takes the laser scan and creates a dictionary of the valid points, with
    # degree = key and value = value
    def compute_valid(self):
        self.valid_points = {}
        for i in range(len(self.last_scan.ranges)):
            if not self.last_scan.ranges[i] <= 0 and\
                    not self.last_scan.ranges[i] > 4.0:
                self.valid_points[i] = self.last_scan.ranges[i]

        self.valid_hist.append(self.valid_points)
        while len(self.valid_hist) > self.hist_length:
            self.valid_hist.pop(0)

    def find_point_changes(self):
        # Cycles to search back through
        search_depth = 1
        # Change in meters to be considered suspicious
        dx = 0.25
        count = 0
        success_count

        for i in range(search_depth):
            index = -1 - search_depth
            for point in self.valid_points.keys():
                if point in self.valid_hist[index].keys()
                    p1 = point_pos(point, self.valid_hist[index][point],
                                    ODOM_DATA[index])
                    p2 = point_pos(point, valid_points[point],
                                    ODOM_DATA)
                    if vector_mag(p1, p2) > dx:
                        count += 1

    def command_motors(self, cmd_vector):
        avoid_vector = self.obstacle_avoid()
        self.drive(vector_add(cmd_vector, avoid_vector))

    # Computes a Vector3 that points away from sensed obstacles, kicking in
    # strongly at 1/5 meters
    def obstacle_avoid(self):
        v = Vector3()
        temp_vector = Vector3()
        for point in self.valid_points.keys():
            reaction = -self.valid_points[point] * self.obstacle_sensitivity
            unit_vector = [cos(point * (pi / 180.0)),
                           sin(point * (pi / 180.0))]
            rospy.logwarn("Check: Unit vector of %d is %s?", point,
                          str(unit_vector))
            temp_vector.x = min(reaction * cos(point * (pi / 180.0)), 0.0)
            temp_vector.y = min(reaction * sin(point * (pi / 180.0)), 0.0)
            v = vector_add(v, temp_vector)
            v = vector_multiply(v, 0.5)
        return v

    # Publishes a given vector command to the cmd_vel topic
    def drive(self, vector):
        cmd = Twist()
        ang = vector_ang(vector)
        if abs(ang) < 45:
            cmd.linear.x = self.max_linear
        elif abs(ang) > 90:
            cmd.linear.x = self.min_linear
        else:
            # Linear fit, 1.0 at 45 and 0.0 at 90 degrees
            cmd.linear.x = 1 - ((abs(ang) - 45) / 45.0)

        if abs(ang) < 90:
            cmd.angular.z = ang / 90.0 * self.max_angular
        else:
            cmd.angular.z = copysign(1, ang) * self.max_angular

        vel_pub.publish(cmd)

    # Publishes an empty vector, stopping the robot
    def stop(self):
        cmd = Twist()
        self.vel_pub.publish(cmd)


def vector_add(v1, v2):
    v = Vector3
    v.x = (v1.x + v2.x) / 2
    v.y = (v1.y + v2.y) / 2
    v.z = (v1.z + v2.z) / 2
    return v

def vector_multiply(v1, scalar):
    v = Vector3
    v.x = v1.x * scalar
    v.y = v1.y * scalar
    v.z = v1.z * scalar
    return v

def vector_mag(v):
    mag = pow(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2), 0.5)
    return mag

# Asngle assumes 2D vector, returns in degrees
# Vertical (0, 1) is an angle of 0, sweeps (+/-) going (CCW/CW)
def vector_ang(v):
    ang = atan2(v.x, v.y)
    return ang * (180 / pi)