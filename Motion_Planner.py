#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from gazebo_msgs.msg import ModelStates
from math import sqrt

class MotionPlanner:
    def __init__(self):
        self.start_point = None
        self.goal_point = None
        self.start_goal_pub = rospy.Publisher('/start_goal', Float64MultiArray, queue_size=10)
	self.reference_pub = rospy.Publisher('/reference_pose', Float64MultiArray, queue_size=10)
        self.trajectory_sub = rospy.Subscriber('/trajectory', Float64MultiArray, self.trajectory_callback)
        #self.target_pose_sub = rospy.Subscriber('/target_pose', Float64MultiArray, self.target_pose_callback)
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

	self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - self.pose_x), 2) +
                    pow((goal_pose[1] - self.pose_y), 2))
    def trajectory_callback(self, msg):
        print("Received trajectory: ", msg.data)
	layout = MultiArrayLayout()
        layout.dim.append(MultiArrayDimension())
        layout.dim[0].label = "position"
        layout.dim[0].size = 2
        layout.dim[0].stride = 2	
	
	distance_tolerance = 0.1
	j=0
	for i in msg.data:
		if j % 2 == 0:
			prev_num = i
		else:
			self.goal_pose_x = float(prev_num)
        		self.goal_pose_y = float(i)
        		self.goal_pose_theta = 0.0
			goal_pose = [self.goal_pose_x, self.goal_pose_y, self.goal_pose_theta]
			data = [prev_num,i]			
			goal_pose_msg = Float64MultiArray(layout=layout, data=data)			
			print("X,Y target is: ",goal_pose_msg.data)
			if j==1:
				self.reference_pub.publish(goal_pose_msg)
			while self.euclidean_distance(goal_pose) >= distance_tolerance:
			
								
				self.reference_pub.publish(goal_pose_msg)
		j=j+1

    def update_pose(self, data):
	"""Callback function which is called when a new message of type ModelStates is
        received by the subscriber."""
       
        self.pose_x = round(data.pose[1].position.x, 4)
        self.pose_y = round(data.pose[1].position.y, 4)
        self.pose_theta = data.pose[1].orientation.z
    #def target_pose_callback(self, target):
	

    def ask_for_start_goal_points(self):
        while self.start_point is None:
            #input_str = raw_input("Please enter the start point coordinates (x,y): ")
            #try:
            #x, y = map(float, input_str.split(','))
	    x = 0
	    y = 0                
	    self.start_point = PointStamped()
            self.start_point.header.stamp = rospy.Time.now()
            self.start_point.header.frame_id = "map"
            self.start_point.point.x = x
            self.start_point.point.y = y
            #except ValueError:
            #print("Invalid input. Please enter two numbers separated by a comma.")
        while self.goal_point is None:
            input_str = raw_input("Please enter the goal point coordinates (x,y): ")
            try:
                x, y = map(float, input_str.split(','))
                self.goal_point = PointStamped()
                self.goal_point.header.stamp = rospy.Time.now()
                self.goal_point.header.frame_id = "map"
                self.goal_point.point.x = x
                self.goal_point.point.y = y
            except ValueError:
                print("Invalid input. Please enter two numbers separated by a comma.")

    def run(self):
        self.ask_for_start_goal_points()

        start_goal_msg = Float64MultiArray()
        start_goal_msg.data = [self.start_point.point.x, self.start_point.point.y, self.goal_point.point.x, self.goal_point.point.y]
        self.start_goal_pub.publish(start_goal_msg)
        print("Published start and goal points: ", start_goal_msg.data)

        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('motion_planner_node', anonymous=True)
    motion_planner = MotionPlanner()
    motion_planner.run()

