#!/usr/bin/env python
# primary node - basically the master node for this project

from __future__ import print_function

import tf
import math
import rospy
import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import Map
from Planners import BaseGlobalPlanner
from Graph import OccupancyGraph
    
def LocalPlan():
    pass

msg = 0
def SensoryProcess(data):
    global msg
    msg = data

def Rotate(turn_direction, bot_publisher):
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0.75 * (turn_direction)
    bot_publisher.publish(vel_msg)

def Translate(bot_publisher):
    vel_msg = Twist()
    vel_msg.linear.x = 1.5
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    bot_publisher.publish(vel_msg)

# this is my main function
def main():
    # make a quick parameter check
    if not rospy.has_param('goalx'):
        rospy.set_param('goalx', 4.5)
    if not rospy.has_param('goaly'):
        rospy.set_param('goaly', 9.0)
    if not rospy.has_param('startx'):
        rospy.set_param('startx', -8.0)
    if not rospy.has_param('starty'):
        rospy.set_param('starty', -2.0)

    rospy.init_node('driver', anonymous=True)
    era_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    rospy.Subscriber("/base_pose_ground_truth", Odometry, SensoryProcess)

    # define graph
    graph = OccupancyGraph(Map.grid)

    # get and set start parameters
    x = rospy.get_param('startx')
    y = rospy.get_param('starty')
    x, y = Map.ToMapCoord(x, y)
    graph.SetStart(y, x)

    # get and set goal parameters
    x = rospy.get_param('goalx')
    y = rospy.get_param('goaly')
    x, y = Map.ToMapCoord(x, y)
    graph.SetGoal(y, x)

    # get path
    path = graph.FindAStarPath()

    # set up global planner
    bismark = BaseGlobalPlanner()
    bismark.SetPath(path)
    bismark.PrintPlan()

    rospy.sleep(1.0)
    # while no shutdown command from rospy and bismark has a plan
    while not rospy.is_shutdown() and bismark.HasPlan():
        # get current position
        x1 = msg.pose.pose.position.x
        y1 = msg.pose.pose.position.y
        theta1 = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2] * 180/math.pi
        # print('x1, y1   :', x1, y1)

        # get next goal point based on current position
        x2, y2 = bismark.GetGoal(x1, y1)
        # print('x2, y2   :', x2, y2)

        # calculate rotation and movement towards current goal
        r, theta2 = bismark.GetDirections(x1, y1, theta1)
        # print('r , theta:', r, theta2)

        # check if bot need to rotate
        if not -(bismark.tolerance * 10) < theta2 < (bismark.tolerance * 10) :
            # if yes, rotate
            Rotate(math.copysign(1, theta2), era_publisher)
        else:
            # if not, check if bot needs to translate
            if r > bismark.tolerance:
                # if yes, translate
                Translate(era_publisher)
        
        # end of iteration
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: 
	    pass
