#!/usr/bin/env python3

import rospy
import actionlib
import math
import random
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from time import time


class PID:
    def __init__(self, Kp, Ki, Kd, distance=1):
        self.Kp = Kp  # proportional gain
        self.Ki = Ki  # integral gain
        self.Kd = Kd  # derivative gain

        self.distance = distance
        self.d_error = 0  # derivative error
        self.i_error = 0  # integral error

    def update(self, cur_distance):
        error = self.distance - cur_distance
        P = self.Kp * error 

        self.i_error += error 
        I = self.Ki * self.i_error 

        derivative = (error - self.d_error) 
        D = self.Kd * derivative

        self.d_error = error

        return P + I + D

class Hider:
    def __init__(self):
        rospy.init_node('hider')

        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
    
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.map_data = None
        self.map_received = False
        self.hider_state = "explore"

        self.twist = Twist()
        self.MOVE_SPEED = 0.2
        self.TURN_SPEED = 0.5
        
        self.wall_distance = 1.0
        self.pid_controller = PID(Kp=1.0, Ki=0.0, Kd=0.1, distance=self.wall_distance)
        self.corner_threshold = 0.5
        self.cur_wall_dist = float('inf')
        
        self.start_time = time()
        self.TIMER = 60 * 5  # 5-minute deadline to hide
        self.explore_time = 10  # ? seconds of initial exploration
        self.follow_time = 20  # 20 seconds max for wall following

        self.explore_start_time = None
        self.wall_follow_start_time = None

        self.corner_detected = False
        self.corner_dist = 0.3  # Distance to stop from corner, 
        self.corner_angle = 0 

    def wait_for_map(self):
        timeout = rospy.Time.now() + rospy.Duration(10)  

        while not self.map_received and rospy.Time.now() < timeout:
            rospy.loginfo("Waiting for map...")
            rospy.sleep(1)
        
        if not self.map_received:
            rospy.logerr("Couldn't receive map within timeout")

    def map_cb(self, msg):
        self.map_data = msg
        self.map_received = True

    def scan_cb(self, msg):
        left_ranges = msg.ranges[0:45]
        right_ranges = msg.ranges[315:360]
        left_distance = min(left_ranges)
        right_distance = min(right_ranges)
        self.cur_wall_dist = min(left_distance, right_distance)

        if self.hider_state == "explore":
            if self.explore_start_time is None:
                self.explore_start_time = time()

            if time() - self.explore_start_time >= self.explore_time:
                if self.cur_wall_dist <= self.wall_distance:
                    rospy.loginfo("Wall detected, switching to follow_wall.")
                    self.hider_state = "follow_wall"
                    self.wall_follow_start_time = time()

        elif self.hider_state == "follow_wall":
            if self.detect_corner(msg.ranges):
                rospy.loginfo("Corner detected, switching to hide.")
                self.hider_state = "hide"
            
            elif time() - self.wall_follow_start_time >= self.follow_time:
                rospy.loginfo("Timeout following wall. Switching to explore.")
                self.hider_state = "explore"
                self.explore_start_time = time()

    def detect_corner(self, msg):
        # ranges_diff = np.diff(ranges)
        # sharp_changes = np.abs(ranges_diff) > self.corner_threshold

        # consecutive_sharp_changes = np.sum(sharp_changes) > 3  # need multiple sharp changes
        # return consecutive_sharp_changes
        left = min(msg.ranges[0:90])  
        right = min(msg.ranges[270:360])  
        
        if left < 1.0 and right < 1.0:
            left_indices = list(range(90))
            right_indices = list(range(270, 360))
            
            left_min_idx = left_indices[msg.ranges[:90].index(left)]
            right_min_idx = right_indices[msg.ranges[270:360].index(right)]
            
            corner_angle = (left_min_idx + right_min_idx) / 2.0 # average of min points
            
            return {'detected': True, 'angle': (corner_angle - 180) * msg.angle_increment}
        
        return None
    
    def approach_corner(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.hider_state == "approach_corner":

            self.twist.linear.x = 0
            self.twist.angular.z = self.TURN_SPEED if self.corner_angle > 0 else -self.TURN_SPEED
            self.cmd_vel_pub.publish(self.twist)
            
            if abs(self.corner_angle) < 0.1:

                self.twist.linear.x = self.MOVE_SPEED
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                
                if self.cur_wall_dist <= self.corner_dist:
                    self.hider_state = "hide"
                    break
            
            rate.sleep()

    def follow_wall(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.hider_state == "follow_wall":

            signal = self.pid_controller.update(self.cur_wall_dist)
            self.twist.linear.x = self.MOVE_SPEED

            if abs(signal) > self.TURN_SPEED:
                self.twist.angular.z = self.TURN_SPEED if signal > 0 else -self.TURN_SPEED
            else:
                self.twist.angular.z = signal 

            self.cmd_vel_pub.publish(self.twist)

            if self.hider_state != "follow_wall":
                break

            rate.sleep()

    def move_to_corner(self):
        rospy.loginfo("Hiding in corner.")

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    # def send_goal(self, x, y):
    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = 'map'
    #     goal.target_pose.header.stamp = rospy.Time.now()
        
    #     goal.target_pose.pose.position.x = x
    #     goal.target_pose.pose.position.y = y
    #     goal.target_pose.pose.orientation.w = 1.0

    #     self.move_base.send_goal(goal)
    #     self.move_base.wait_for_result(rospy.Duration(5))

    # def find_explored_point(self):
    #     if self.map_data is None:
    #         rospy.logwarn("Map not available yet")
    #         return None

    #     width = self.map_data.info.width
    #     height = self.map_data.info.height
    #     resolution = self.map_data.info.resolution
    #     origin = self.map_data.info.origin

    #     map_array = np.array(self.map_data.data).reshape((height, width))

    #     explored_points = []

    #     free = 0  #thresholds
    #     wall = 50
    #     unknown = -1

    #     for y in range(1, height - 1):  
    #         for x in range(1, width - 1):
    #             # Look for free space that has been explored
                
    #             if map_array[y, x] <= free:
    #                 # Check if the point is somewhat isolated (not in a dense cluster)
    #                 # neighborhood_free = np.sum(map_array[y-1:y+2, x-1:x+2] <= free)
    #                 # neighborhood = [   # north south east west
    #                 #     map_array[y-1, x],  map_array[y+1, x],   
    #                 #     map_array[y, x-1],   map_array[y, x+1],   
    #                 # ]
    #                 # neighborhood_free = sum(neighborhood)

    #                 # if 3 <= neighborhood_free <= 6:  # Avoid  dense clusters
    #                 world_x = x * resolution + origin.position.x
    #                 world_y = y * resolution + origin.position.y

    #                 explored_points.append(Point(world_x, world_y, 0))

    #     if len(explored_points) == 0:
    #         rospy.loginfo("No explored points found")
    #         return None

    #     chosen_point = random.choice(explored_points)
    #     rospy.loginfo(f"Chosen point: ({chosen_point.x}, {chosen_point.y})")
    #     return chosen_point

    def explore(self):
        rate = rospy.Rate(10) 

        self.wait_for_map()
        self.explore_start_time = time()
        
        while not rospy.is_shutdown():
            elapsed_time = time() - self.start_time

            if elapsed_time >= self.TIMER:
                rospy.loginfo("didn't hide in time o.o")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
                break

            if self.hider_state == "explore":
                # explored_point = self.find_explored_point()
                
                # if explored_point:
                #     rospy.loginfo(f"Going to explored point: ({explored_point.x}, {explored_point.y})")
                #     self.send_goal(explored_point.x, explored_point.y)
                # else:
                #     rospy.loginfo("No more points to explore")
                #     break

                #use random exploration
                self.twist.linear.x = self.MOVE_SPEED/2
                # self.twist.angular.z = 0.5
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(0.1)

            elif self.hider_state == "hide":
                self.move_to_corner()
                break

            elif self.hider_state == "follow_wall":
                self.follow_wall()

            rate.sleep()


if __name__ == '__main__':
    hide = Hider()
    hide.explore()