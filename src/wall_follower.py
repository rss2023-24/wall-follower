#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math
from visualization_tools import *
from PID import PID
from std_msgs.msg import Float32

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT

    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    WALL_TOPIC = "/wall"
    
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")


    def __init__(self):
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.min_dist_pub = rospy.Publisher('/min_distance', Float32, queue_size=1) 
        rospy.Subscriber('scan', LaserScan, self.laser_callback)

        self.controller = PID()
        
        self.x_list = []
        self.y_list = []
        self.numLoops = 0

    def drive(self, angle):
        ts = AckermannDriveStamped()
        ts.drive.speed = self.VELOCITY
        ts.drive.acceleration = 0
        ts.drive.steering_angle = angle

        self.drive_pub.publish(ts)
    
    def slice_data(self, scan_pairs):
        # front_of = lambda lst, prop: lst[ 0 : int(math.floor(prop * len(lst))) ]
        # back_of = lambda lst, prop: lst[ int(math.ceil((1.0 - prop) * len(lst))) : ]
        
        segment_len = len(scan_pairs) / 3
        right, forward, left = scan_pairs[0 : segment_len], scan_pairs[segment_len : 2 * segment_len], scan_pairs[2 * segment_len : ]

        directional_choice_constant = 0.3
        side_choice = left if self.SIDE == 1 else right
        directional_lst = list(sorted(side_choice, key=lambda p: p[1]))
        directional = directional_lst[0 : int(math.ceil(directional_choice_constant * len(directional_lst)))]

        sight_threshold = 4.0 * self.DESIRED_DISTANCE
        include_threshold = 2.2 * self.DESIRED_DISTANCE

        avg_forward_dist = np.median( [dist for _, dist in forward] )

        see_wall = avg_forward_dist <= sight_threshold

        if see_wall:
            # pairs = filter( (lambda p: p[1] <= include_threshold), directional + forward )
            pairs = filter( (lambda p: p[1] <= sight_threshold), directional + forward )
        else:
            pairs = directional
            # min_dist = min(directional, key=lambda p: p[1])[1]
            # pairs = filter((lambda p: p[1] <= 1.3 * min_dist), directional)

        # print("Max: {}".format(max(degs)))
        # print("Min: {}".format(min(degs)))

        return pairs, see_wall

    def laser_callback(self, data):
        ls = data
        numLoops = 0

        sp = make_scan_pairs(ls)
        pairs, see_wall = self.slice_data(sp)

        if not pairs:
            print("Data is empty")
            self.drive(0)
            return

        laser_x, laser_y = get_points(pairs)
        m, c = get_line_regression(laser_x, laser_y)
        
        line_x, line_y = generate_line_points(laser_x, m, c)

        min_dist = min( (x ** 2.0 + y ** 2.0) ** (1.0/2.0) for x, y in zip(line_x, line_y) )

        wall_translation_constant = 1.5
        min_dist = min_dist if not(see_wall) else max(min_dist - wall_translation_constant, 0.0)

        drive_angle = self.controller.step(min_dist)
        self.drive(drive_angle if self.SIDE == -1 else -drive_angle)
        # self.drive(0)

        if (see_wall):
            print("Wall sighted")
        else:
            print("Parallel to wall")

        y = get_loss_function(self.DESIRED_DISTANCE, min_dist)

        self.min_dist_pub.publish(min_dist)

        # VisualizationTools.plot_line(correct_x, correct_y, self.line_pub, 'green', frame="/laser")
        # VisualizationTools.plot_line(incorrect_x, incorrect_y, self.line_pub, 'red', frame="/laser")

        # delta = 0.2
        # if min_dist > self.DESIRED_DISTANCE * (1.0 + delta) or min_dist < self.DESIRED_DISTANCE * (1.0 - delta):
        #     color = 'red'
        # else:
        #     color = 'green'

        # VisualizationTools.plot_line(line_x, line_y, self.line_pub, color, frame="/laser")

        # rospy.loginfo("Min distance: {}\n\n".format(min_dist))

def rad_to_deg(rad):
    return rad * 180.0 / (math.pi)

def make_scan_pairs(ls):
    scan_pairs = []
    for ind, range in enumerate(ls.ranges):
        angle = ls.angle_min + ind * ls.angle_increment
        dist = range
        scan_pairs.append( (angle, dist) )
    return scan_pairs

def get_points(scan_pairs):
    x = []
    y = []
    for angle, dist in scan_pairs:
        x.append(dist * math.cos(angle))
        y.append(dist * math.sin(angle))
    return x, y

def get_line_regression(x, y):
    x_arr = np.array(x)
    y_arr = np.array(y)
    A = np.vstack([x_arr, np.ones(len(x_arr))]).T
    m, c = np.linalg.lstsq(A, y_arr)[0]
    return m, c

def generate_line_points(x_arr, m, c):
    predicted_y = [m * x + c for x in x_arr]
    return x_arr[:], predicted_y

def get_loss_function(desired_dist, actual_dist):
    error = desired_dist - actual_dist
    return abs(error)

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
