#!/usr/bin/env python3

#http://cosi119r.s3-website-us-west-2.amazonaws.com/content/topics/robotics/ros_book/100_prr_localization/math

# roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:="/my_ros_data/catkin_ws/hider_sim_1.yaml"
# rosrun hider hider_sim.py
# rosrun hider scan_sim.py 

import rospy
from geometry_msgs.msg import Twist
import actionlib
import math
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Float32MultiArray

RATE = 30
PI = math.pi
POINT_RADIUS = 8

POS_X = 4
NEG_X = -4
POS_Y = 4
NEG_Y = -4.5

SPEED = 0.1
HIDE_TIME = 30
START_COORD = [(0, 2.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
START_1 = [(2.0, 4.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
START_2 = [(4.0, 0.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)]
START_3 = [(2.0, -4.5, 0.0),
      (0.0, 0.0, 0.0, 1.0)]

hide_spots = []



class Hider:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.my_scan_sub = rospy.Subscriber('my_scan', Float32MultiArray, self.my_scan_cb)
        self.range_max = -999
        self.range_min = -999
        self.ranges = []
        self.spots = []
        self.queue = []
        self.visited = []
    
    def my_scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        pockets = msg.data

        while len(pockets) > 0:
            coord_x = round(pockets[0], 3)
            coord_y = round(pockets[1], 3)
            coord = [(coord_x, coord_y, 0.0),(0.0, 0.0, 0.0, 1.0)]
            
            not_queue = True
            not_visited = True
            
            if len(self.queue) < 10:

                for i in self.queue:
                    spot = i
                    spot_x = round(spot[0][0], 3)
                    spot_y = round(spot[0][1], 3)

                    if coord_x == spot_x and coord_y == spot_y:
                        not_queue = False
                    
                    if round(abs(coord_x - spot_x) <= 0.5, 2) and round(abs(coord_y - spot_y) <= 0.5, 2):
                        not_queue = False
                
                for i in self.visited:
                    spot = i
                    spot_x = spot[0][0]
                    spot_y = spot[0][1]

                    if coord_x == spot_x and coord_y == spot_y:
                        not_visited = False
                    
                    if round(abs(coord_x - spot_x) <= 0.5, 2) and round(abs(coord_y - spot_y) <= 0.5, 2):
                        not_visited = False

                if not_queue and not_visited:
                    self.queue.append(coord)
                pockets = pockets[2:]
        #print(self.spots)

    
    def goal_pose(self, pose):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

    def goto(self, pose):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # wait for action server to be ready
        client.wait_for_server()
        goal = self.goal_pose(pose)
        print(f"Going for goal: {pose}{goal}")
        client.send_goal(goal)
        # client.wait_for_result()
        # print("success")

        #success = client.wait_for_result(rospy.Duration(30))  # Add timeout if needed
        success = client.wait_for_result()

        # Check status of the goal
        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            print("Goal reached successfully!")
        elif state == GoalStatus.ABORTED:
            print("Goal aborted! Could not reach the goal.")
        elif state == GoalStatus.PREEMPTED:
            print("Goal preempted! The goal was cancelled.")
        else:
            print(f"Goal failed with state: {state}")
    
    
    def hide(self):
        # go to the far wall to get away from the hider
        rand_start = random.randint(1, 3)
        coord = 0
        #rand_start = 2
        print(rand_start)
        if rand_start == 1:
            coord = START_1
            self.goto(START_1)
        elif rand_start == 2:
            coord = START_2
            self.goto(START_2) 
        elif rand_start == 3:
            coord = START_3
            self.goto(START_3)
        self.visited.append(coord)

        while not rospy.is_shutdown():
            if len(self.queue) > 0:
                print("GOing to a spot!")
                print(self.queue)
                print(self.queue[0])
                self.goto(self.queue[0])
                self.visited.append(self.queue[0])
                self.queue = self.queue[1:]
            else:
                start_x = coord[0][0]
                start_y = coord[0][1]
                start_z = coord[0][2]
                x = round(random.uniform((start_x - POINT_RADIUS), (start_x + POINT_RADIUS)), 1)
                while x > POS_X or x < NEG_Y:
                    x = round(random.uniform((start_x - POINT_RADIUS), (start_x + POINT_RADIUS)), 1)
                
                y = round(random.uniform((start_y - POINT_RADIUS), (start_y + POINT_RADIUS)), 1)
                while y > POS_Y or y < NEG_Y:
                    y = round(random.uniform((start_y - POINT_RADIUS), (start_y + POINT_RADIUS)), 1)
                
                self.goto([(x, y, 0.0),
            (0.0, 0.0, 0.0, 1.0)])
                self.visited.append(coord)
                coord = [(x, y, 0.0),
                (0.0, 0.0, 0.0, 1.0)]
            

      #drive around to a bunch of different spots and constantly check lidar for "low higher low" to indicate a 3 wall spot
      #when the an inside pocket is detected, get the angle and distance from the lidar to calculate the coordinate
      #go to the coordinates
      #go the average distance in the 180 degrees the opening is facing
      #rank accordingly
      # save the ccoord to list

        #choose to go left or right
        # randint = random.randint(1, 2)

        # if randint == 1: #go to the right
            
        # elif randint ==2: #go to the left
    
    # def hide()
    #     #go towards the back wall and turn back towards seeker

    #     #initiate wanderbot

    #     #if obstable found
    #         #wall follow around it until inside corner found
    #             #align so that obstable is to the left
    #         #check 360 lidar to determine how many quadrants have a wall around it
    #         #determine if orientation is away from seeker start position
    #     pass

if __name__ == '__main__':
    rospy.init_node('hider')
    Hider().hide()



# #!/usr/bin/env python3

# import math
# import rospy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# import numpy as np
# from geometry_msgs.msg import Point, Pose, Twist
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# RATE = 30

# PI = math.pi

# waypoints = [
#     [ (-1.0, 0.0, 0.0),
#       (0.0, 0.0, 0.0, 1.0)],
#     [ (-1.0, 2.0, 0.0),
#       (0.0, 0.0, 0.0, 1.0)]
# ]

# class Hider:
#     def __init__(self):
#         self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
#         #self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
#         #self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)

#     #sample code from class
#     def goal_pose(self, pose):
#         goal_pose = MoveBaseGoal()
#         goal_pose.target_pose.header.frame_id = 'map'
#         goal_pose.target_pose.pose.position.x = pose[0][0]
#         goal_pose.target_pose.pose.position.y = pose[0][1]
#         goal_pose.target_pose.pose.position.z = pose[0][2]
#         goal_pose.target_pose.pose.orientation.x = pose[1][0]
#         goal_pose.target_pose.pose.orientation.y = pose[1][1]
#         goal_pose.target_pose.pose.orientation.z = pose[1][2]
#         goal_pose.target_pose.pose.orientation.w = pose[1][3]
#         return goal_pose

#     #main function that runs
#     def goto(self):
#         """Makes the robot follow a wall."""
#         rate = rospy.Rate(RATE)

#         #sleep at the start so odom can gather some data
#         count = 10
#         while count > 0:
#             count -= 1
#             rate.sleep()

#         #while running
#         while not rospy.is_shutdown():
#                 rate.sleep()
    
#     def test_movebase(self):
        
#         client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#         while not rospy.is_shutdown():
#         # repeat the waypoints over and over again
#             for pose in waypoints:
#                 goal = self.goal_pose(pose)
#                 print("Going for goal: ", goal)
#                 client.send_goal(goal)
#                 client.wait_for_result()
            
# if __name__ == '__main__':
#     rospy.init_node('hider')
    

#     Hider().test_movebase()
