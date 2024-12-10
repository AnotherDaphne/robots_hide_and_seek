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


class PID:
    def __init__(self, Kp, Ki, Kd, distance = 1):
        self.Kp = Kp #proportional gain
        self.Ki = Ki #inregral gain
        self.Kd = Kd #derivative gain

        self.distance = distance
        self.d_error = 0 #derivative error
        self.i_error = 0 #integral error

    def update(self, cur_distance):
        error = self.distance - cur_distance
        P = self.Kp * error 

        self.i_error += error 
        I = self.Ki * self.i_error 

        derivative = (error - self.d_error) 
        D = self.Kd * derivative

        self.d_error = error

        return P + I + D

class hider:
    def __init__(self):
        rospy.init_node('hider', anonymous=True)

        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5)) #idk how long this takes
    
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
        self.map_data = None
        self.map_received = False

        self.hider_state = "explore"

        self.twist = Twist()
        self.MOVE_SPEED = 0.2
        # self.TURN_SPEED = 

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.wall_distance = 1
        self.pid_values = PID(Kp = 1.0, Ki = 0.0, Kd = 0.1, distance = self.wall_distance)
        self.current_wall_dist = 0.0
        self.corner_threshold = 0.5

    def wait_for_map(self):
        timeout = rospy.Time.now() + rospy.Duration(10)  

        while not self.map_received and rospy.Time.now() < timeout:
            rospy.loginfo("waiting for map...")
            rospy.sleep(1)
        
        if not self.map_received:
            rospy.logerr("couldn't recieve map within timeout")

    def map_cb(self, msg):
        self.map_data = msg
        self.map_received = True

        # self.map_sub.unregister()  #do once get map, saves resources?

    def scan_cb(self, msg):
        left_distance = min(msg.ranges[0:30])
        right_distance = min(msg.ranges[330:360])

        left_min = np.min(left_distance)
        right_min = np.min(right_distance)
        self.cur_wall_dist = min(left_min, right_min)
        # cur_distance = min(left_distance, right_distance)

        # if cur_distance <= wall_distance:
        #     if abs(np.diff(ranges)) > self.corner_threshold:
        #         self.hider_state = "hide"
        # # signal = self.pid_values.update(cur_distance - wall_distance)

        if self.cur_wall_dist <= self.wall_distance and self.hider_state == "explore":
            rospy.loginfo("Wall detected. Following the wall.")
            self.hider_state = "follow_wall"

        if self.hider_state == "follow_wall" and self.detect_corner(msg.ranges):
            rospy.loginfo("Corner found. Hiding...")
            self.hider_state = "hide"
    
    def detect_corner(self, ranges):
        ranges_diff = np.diff(ranges)
        sharp_changes = np.abs(ranges_diff) > self.corner_threshold

        return np.any(sharp_changes)
    
    def follow_wall(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown() and self.hider_state == "follow_wall":

            signal = self.pid_values.update(self.cur_wall_dist - self.wall_distance)

            self.twist.linear.x = MOVE_SPEED  
            self.twist.angular.z = control_signal 

            self.cmd_pub.publish(self.twist)


            if self.detect_corner():
                rospy.loginfo("Corner detected")
                self.hider_state = "hide"
                break

            rate.sleep()

    def move_to_corner(self):
        self.twist.linear.x = 0.0
        self.twist.angualr.z = 0.0
        self.cmd_pub.publish(self.twist)

        rospy.sleep(1)

        # current_time = rospy.Time.now()
        # if (current_time - self.corner_turn_start_time).to_sec() < 4:  # turns for x seconds  (4 works better)
        #         self.twist.linear.x = MOVE_SPEED * 0.4
        #         self.twist.angular.z = TURN_SPEED * 1.1
        #         print("turning_corner")
        #         print(wall_distance)

    def send_goal(self, x, y):
        goal = MoveBaseGoal()  #create goal
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x #goal pos
        goal.target_pose.pose.position.y = y
        
        orientation_z = random.uniform(-1, 1) #random so don't get stuck
        goal.target_pose.pose.orientation.z = orientation_z
        goal.target_pose.pose.orientation.w = math.sqrt(1 - orientation_z**2)

        self.move_base.send_goal(goal)  #send goal
        success = self.move_base.wait_for_result(rospy.Duration(5)) #idk how long this takes


        state = self.move_base.get_state()
        if success and state == actionlib.GoalStatus.SUCCEEDED:
            print(f"reached goal: ({x}, {y})")
            return True
        else:
            rospy.logwarn(f"didn't reach goal: ({x}, {y}). State: {state}")
            return False

    def find_unexplored_point(self):
        if self.map_data is None:
            rospy.logwarn(" map not available yet")
            return None

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin

        map_array = np.array(self.map_data.data).reshape((height, width))  #numpy more efficient

        unexplored_indicies = []

        free = 0     #thresholds
        wall = 100
        unknown = -1

        for y in range(1, height - 1):  
            for x in range(1, width - 1):

                if map_array[y, x] == unknown:  #current cell not explored

                    neighborhood = [   # north south east west
                        map_array[y-1, x],  map_array[y+1, x],   
                        map_array[y, x-1],   map_array[y, x+1],   
                    ]

                    for cell in neighborhood:  #if any adjecent free, make point
                        if (cell <= free):
                            world_x = x * resolution + origin.position.x   #grid to world coordinates
                            world_y = y * resolution + origin.position.y
                            unexplored_indicies.append(Point(world_x, world_y, 0))

        if len(unexplored_indices) == 0:
            rospy.loginfo("No more unexplored cells")
            return None

        return random.choice(valid_unexplored_points)

    def explore(self):
        rate = rospy.Rate(10) 

        self.wait_for_map()

        while not rospy.is_shutdown():
            if self.hider_state == "explore":
                unexplored_point = self.find_unexplored_point()  #go to unexplored
                
                if unexplored_point:
                    print(f"going to unexplored point: ({unexplored_point.x}, {unexplored_point.y})")
                    self.send_goal(unexplored_point.x, unexplored_point.y)
                else:
                    print("exploration done")
                    break

                rate.sleep()
            elif self.hider_state == "hiding":
                self.move_to_corner()

            elif self.hider_state == "follow_wall":
                self.follow_wall()


if __name__ == '__main__':
    hide = hider()
    hide.explore()