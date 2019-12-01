#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion
from std_srvs.srv import Empty
import rosbag
import subprocess
import os
import csv

waypoints = []
result = Float64()
result.data = 0
x_offset = 50
y_offset = 50
resolution = 5
maxSimulations = 1

with open('tst.txt', 'rb') as file:
    reader = csv.reader(file, delimiter=',')

    for row in reader:
        ponto = Point()
        ponto.x = float(row[0])
        ponto.y = float(row[1])
        waypoints.append(ponto)

    waypoints = list(reversed(waypoints))
    #print(waypoints)

def goal_pose(pose):
    goal_tmp = Odometry()
    pose_tmp = Point()
    goal_tmp.header.stamp = rospy.Time.now()
    goal_tmp.header.frame_id = 'world'

    pose_tmp.x = pose.x*resolution + x_offset
    pose_tmp.y = pose.y*resolution + y_offset
    goal_tmp.pose.pose.position = pose_tmp
    rospy.loginfo("(%d, %d) <------> (%d, %d)", pose.x, pose.y, pose_tmp.x, pose_tmp.y)
    return goal_tmp

def get_result(result_aux):
    global result
    result.data = result_aux.data

if __name__ == '__main__':
    global proc 

    pub = rospy.Publisher('sailboat/move_usv/goal', Odometry, queue_size=10)
    rospy.init_node('patrol')
    rate = rospy.Rate(1) # 10h
    rospy.Subscriber("sailboat/move_usv/result", Float64, get_result)

    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.wait_for_service('/gazebo/pause_physics')
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause()

    simulationNumber = 1

    while not rospy.is_shutdown():    
        for pose in waypoints:
            rospy.loginfo("Going to point number %d", simulationNumber)
            goal = goal_pose(pose)
            pub.publish(goal)
            rate.sleep()

            while result.data == 0.0:
                pub.publish(goal)
                rate.sleep()

            simulationNumber = simulationNumber + 1

            if simulationNumber > len(waypoints):
                pause()
                break;