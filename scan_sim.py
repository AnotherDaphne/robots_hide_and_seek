#!/usr/bin/env python3

#http://cosi119r.s3-website-us-west-2.amazonaws.com/content/topics/robotics/ros_book/100_prr_localization/math
import rospy
from geometry_msgs.msg import Twist
import actionlib
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import tf


RATE = 1
PI = math.pi
POINT_RADIUS = 2


class ScanSim:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.my_scan_pub = rospy.Publisher("my_scan", Float32MultiArray, queue_size=1)

        self.tf_listener = tf.TransformListener()

        self.range_max = -999
        self.range_min = -999
        self.ranges = []

    def get_robot_position(self):
        """Get the robot's current position using tf."""
        try:
            # Wait for the transform between 'map' and 'base_link'
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            
            x, y, z = trans  # Position in 3D space
            # Convert quaternion to Euler angles to get yaw
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            
            return x, y, yaw  # Return position and yaw angle
        except (tf.Exception, tf.ConnectivityException, tf.LookupException) as e:
            rospy.logerr(f"TF Error: {e}")
            return None, None, None

    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""

        self.range_max = msg.range_max
        self.range_min = msg.range_min
        self.ranges = self.clean(msg.ranges)
    
    def clean(self, ranges):
        clean = []
        for i in ranges:
            if i > self.range_max or i < self.range_min:
                clean.append(float('nan'))
            else:
                clean.append(i)
        return clean
    
    def centroid(self, points):
        (x1, y1), (x2, y2), (x3, y3) = points
        
        # Calculate centroid coordinates
        centroid_x = (x1 + x2 + x3) / 3
        centroid_y = (y1 + y2 + y3) / 3

        print(f"1: {x1} {y1}")
        print(f"2: {x2} {y2}")
        print(f"3: {x3} {y3}")
        return centroid_x, centroid_y
    
    def get_coord(self, distance, radians, coordinate):
        x = coordinate[0]
        y = coordinate[1]
        new_x = x + distance * math.cos(radians)
        new_y = y + distance * math.sin(radians)
        return new_x, new_y
    
    def set_mid(self, array, averaged):
        if len(array) != 0:
            if len(array) % 2 == 0:
                localmax = array[len(array)//2]
                localmax_index = averaged.index(localmax)
                return localmax_index, localmax
            else:
                localmax = array[(len(array) - 1)//2]
                localmax_index = averaged.index(localmax)
                return localmax_index, localmax
    
    def find(self, ranges):
        print("--------------------------------------")
        sensitivity = 0.2
        #get averages
        #print(ranges)
        averaged = []
        min_amt = 0
        max_amt = 5
        while max_amt < len(ranges) + 1:
            value = round(np.nanmean(ranges[min_amt:max_amt]), 4)
            averaged.append(value)
            min_amt += 6
            max_amt += 6
            if max_amt > len(ranges) + 1:
                max_amt = len(ranges) + 1

        #look for increasing on both sides
        spots = []
        localmin_1 = -999
        localmin_1_index = -999
        localmax = -999
        localmax_index = -999
        localmin_2 = -999
        localmin_2_index = -999

        last_max = -999
        maxes = []

        increasing = []
        increase_sens = 0.1
        increase_lock = False
        plateau = []
        plateau_sens = 0.05
        plateau_lock = False
        decreasing = []
        decreasing_sens = 0.1

        go_back = 0
        for index in range(len(averaged) + go_back):
            i = averaged[index - go_back]
            print(i)

            if math.isnan(i):
                if len(increasing) != 0 and len(plateau) != 0 and len(decreasing) != 0:
                    print("pocket found")
                    spots.append([localmin_1, localmax, localmin_2, localmin_1_index, localmax_index, localmin_2_index])
                    if index >= 2:
                        go_back += 2
                print("Resetting: Nan")
                increasing = []
                plateau = []
                decreasing = []
                increase_lock = False
                plateau_lock = False
            elif len(increasing) == 0:
                increasing.append(i)
                localmin_1 = increasing[0]
                localmin_1_index = index
                print(f"first increase {increasing}")
            elif abs(i-increasing[-1]) >= increase_sens and i > increasing[-1]:
                increasing.append(i)
                localmin_1 = increasing[0]
                localmin_1_index = index
                print(f"add increase {increasing}")
            elif len(plateau) == 0 and i > increasing[-1]:
                plateau.append(i)
                increase_lock = True
                temp_index, temp_max = self.set_mid(plateau, averaged)
                localmax_index = temp_index
                localmax = temp_max
                print(f"first plateau {plateau}")
            elif increase_lock and abs(i-plateau[-1]) >= plateau_sens:
                plateau.append(i)
                temp_index, temp_max = self.set_mid(plateau, averaged)
                localmax_index = temp_index
                localmax = temp_max
                print(f"add plateau {plateau}")
            elif increase_lock and len(decreasing) == 0 and i < plateau[-1]:
                decreasing.append(i)
                plateau_lock = True
                localmin_2 = decreasing[-1]
                localmin_2_index = index
                print(f"first decrease {decreasing}")
            elif increase_lock and plateau_lock and abs(i-decreasing[-1]) >= decreasing_sens and i < decreasing[-1]:
                decreasing.append(i)
                localmin_2 = decreasing[-1]
                localmin_2_index = index
                print(f"add decrease {decreasing}")
            else:
                if len(increasing) != 0 and len(plateau) != 0 and len(decreasing) != 0:
                    print("pocket found")
                    spots.append([localmin_1, localmax, localmin_2, localmin_1_index, localmax_index, localmin_2_index])
                    if index >= 2:
                        go_back += 2
                print("resetting")
                if index >= 1:
                        go_back += 1
                increasing = []
                plateau = []
                decreasing = []
                increase_lock = False
                plateau_lock = False

        print(f"All: {averaged}")
        
        new_spots = []
        x, y, yaw = self.get_robot_position()
        for pocket in spots:
            print(f"Spot: {pocket}")
            distance = pocket[1]
            radians = math.radians(pocket[3] * 6)
            #radians = ranges.index(pocket[2])
            min_x_1, min_y_1 = self.get_coord(pocket[0], math.radians(pocket[3] * 6), [x, y])
            max_x, max_y = self.get_coord(pocket[1], math.radians(pocket[4] * 6), [x, y])
            min_x_2, min_y_2 = self.get_coord(pocket[2], math.radians(pocket[5] * 6), [x, y])

            final_x = (min_x_1 + min_x_2) / 2
            final_y = (min_y_1 + min_y_2) / 2
            
            final_x, final_y = self.centroid([(min_x_1, min_y_1), (max_x, max_y), (min_x_2, min_y_2)])
            # new_spots.append(final_x)
            # new_spots.append(final_y)
            print(f"Coord: {final_x}, {final_y}")

        return new_spots


        #return array with possible spots
        
    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        rate = rospy.Rate(RATE)
        count = 20
        while count > 0:
            count -= 1

        while not rospy.is_shutdown():

            #print(self.ranges)
            #print(f"Publish function: {self.find(self.ranges)}")

            x, y, yaw = self.get_robot_position()
            # if x is not None:
            #     print(f"Robot Position: x={x}, y={y}, yaw={yaw}")
            #self.find(self.ranges)
            msg = Float32MultiArray()
            #msg.data = self.ranges
            msg.data = self.find(self.ranges)
            #print(msg.data)
            self.my_scan_pub.publish(msg)
            rate.sleep
        
if __name__ == '__main__':
    rospy.init_node('scan_sim')
    ScanSim().publish_data()















        # for i in averaged:
        #     if last_point != -999:
        #         if abs(last_point - i) > 0.5:
        #             last_point = i
        #             if localmin_1 != -999 and localmax != -999 and localmin_2 != -999:
        #                 spots.append([localmin_1, localmax, localmin_2, localmax_index])
        #             localmin_1 = -999
        #             localmax = -999
        #             localmin_2 = -999
        #             localmin_1_unlocked = True
        #             break
        #         else:
        #             last_point = i
        #     if i == float('nan') or (i < localmin_1 and localmin_1_unlocked):
        #         if localmin_1 != -999 and localmax != -999 and localmin_2 != -999:
        #             spots.append([localmin_1, localmax, localmin_2, localmax_index])
        #         localmin_1 = -999
        #         localmax = -999
        #         localmin_2 = -999
        #         localmin_1_unlocked = True
        #     if localmin_1 == -999 or (i == localmin_1 and localmin_1_unlocked):
        #         localmin_1 = i
        #         #localmin_1_index = index
        #         localmin_1_index = averaged.index(i)
        #     elif i > localmin_1 and i > localmax:
        #         localmax = i
        #         #localmax_index = index
        #         localmax_index = averaged.index(i)
        #         localmin_1_unlocked = False
        #     elif (i < localmax and (localmin_2 == -999 or i <= localmin_2))and localmin_1_unlocked == False:
        #         localmin_2 = i
        #         #localmin_2_index = index
        #         localmin_2_index = averaged.index(i)
        #     else:
        #         if localmin_1 != -999 and localmax != -999 and localmin_2 != -999:
        #             spots.append([localmin_1, localmax, localmin_2, localmin_1_index, localmax_index, localmin_2_index])
        #         localmin_1 = -999
        #         localmax = -999
        #         localmin_2 = -999
        #         localmin_1_unlocked = True
        #         #print(f"Averages: {averaged}")
        #     index += 1


        # for index in range(len(averaged)):
        #     i = averaged[index - go_back]

        #     print(i)
        #     if math.isnan(i):
        #         if localmin_1 != -999 and localmax != -999 and localmin_2 != -999:
        #             print("pocket found")
        #             spots.append([localmin_1, localmax, localmin_2, localmin_1_index, localmax_index, localmin_2_index])
        #             if index >= 2:
        #                 go_back += 2
        #         localmin_1 = -999
        #         localmax = -999
        #         localmin_2 = -999
        #         last_max = -999
        #         maxes = []
        #         localmin_1_unlocked = True
        #     elif localmin_1 == -999 or (i <= localmin_1 and localmin_1_unlocked): #if localmin has not been set or there is an identical value
        #         print(f"{i} has been set as localmin 1")
        #         localmin_1 = i
        #         localmin_1_index = averaged.index(i)
        #     elif i > localmin_1 and (last_max == -999 or abs(last_max - i) <= sensitivity):
        #         print(f"{i} has been added to maxes")
        #         last_max = i
        #         maxes.append(i)
        #         temp_index, temp_max = self.set_mid(maxes, averaged)
        #         localmax_index = temp_index
        #         localmax = temp_max
        #         localmin_1_unlocked = False
        #     elif (abs(last_max - i) >= sensitivity and (localmin_2 == -999 or i <= localmin_2)) and localmin_1_unlocked == False and len(maxes) > 1:
        #         print(f"{i} has been set as localmin 2")
        #         localmin_2 = i
        #         localmin_2_index = averaged.index(i)
        #     else:
        #         if localmin_1 != -999 and localmax != -999 and localmin_2 != -999:
        #             spots.append([localmin_1, localmax, localmin_2, localmin_1_index, localmax_index, localmin_2_index])
        #             if index >= 2:
        #                 go_back += 2
        #             print(f"spot found")
        #         print("resetting")
        #         if index >= 1:
        #                 go_back += 1
        #         localmin_1 = -999
        #         localmax = -999
        #         last_max = -999
        #         maxes = []
        #         localmin_2 = -999
        #         localmin_1_unlocked = True
        #         #print(f"Averages: {averaged}")
