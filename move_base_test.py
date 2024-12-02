#!/usr/bin/env python

# rosrun hider move_base_test.py
import rospy
import actionlib

# move_base is the package that takes goals for navigation
# there are different implemenetations with a common interface
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# You need to know the coordinates that the map you are working
# in is. I experimented with these numbers and the turtlebot3_stage_4
# map from Turtlebot3. The first array is x,y,z location. The second one
# is a "quaternion" defining an orientation. Quaternions are a different
# mathematical represetnation for "euler angles", yaw, pitch and roll.

waypoints = [
    [ (-1.0, 0.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)],
    [ (-1.0, 2.0, 0.0),
      (0.0, 0.0, 0.0, 1.0)],
    [ (0.0, 0.0, 0.0),
      (0.0, 0.0, 0.0, 0.0)],
]

# Function to generate a proper MoveBaseGoal() from a two dimensional array
# containing a location and a rotation. This is just to make the waypoints array
# simpler.

def goal_pose(pose):
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

def goto(pose):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # wait for action server to be ready
    client.wait_for_server()
    goal = goal_pose(pose)
    print(f"Going for goal: {pose}")
    print(goal)
    client.send_goal(goal)
    client.wait_for_result()
    print("success")

# Main program starts here
if __name__ == '__main__':

    # A node called 'patrol' which is an action client to move_base
    rospy.init_node('patrol')
    origin = False
    other = True
    if origin:
      goto([(0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0)])
    if other:
      x = -0.6437218679310709
      y = -2.554388503760109
      goto([(x, y, 0.0),
      (0.0, 0.0, 0.0, 1.0)])

# 1: -1.7735081466642917, -0.997734177613789
# 2: -2.1011816676138357, -0.6534200537776607
# 3: -2.5539556956726637, -1.0061623205332584
    