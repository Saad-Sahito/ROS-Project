#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from gazebo_msgs.msg import ModelStates

from std_msgs.msg import Float64MultiArray
import tf
from math import radians, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import os
import time


import h5py



class NN_Control:

    def __init__(self):
        
        # unique node (using anonymous=True).
        rospy.init_node('NN_Control_US', anonymous=False)

        # Publisher which will publish to the topic 'cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel',
                                                  Twist, queue_size=5)

        # A subscriber to the topic '/gazebo/model_states'. self.update_pose is called
        # when a message of type ModelStates is received.
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)
	self.ref_pose_subscriber = rospy.Subscriber('/reference_pose', Float64MultiArray, self.reference_pose)
        
        
        ############### AGENT'S POSE in the gazebo environment #######################
        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0

        
	
	#### goal poses ########
	self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0

	self.rate = rospy.Rate(10)


    def reference_pose(self, ref_pose):
	
	self.goal_pose_x = float(ref_pose.data[0])
	self.goal_pose_y = float(ref_pose.data[1])
	
	
	

    def update_pose(self, data):
        """Callback function which is called when a new message of type ModelStates is
        received by the subscriber."""
       
        self.pose_x = round(data.pose[1].position.x, 4)
        self.pose_y = round(data.pose[1].position.y, 4)
        rotations_rads = euler_from_quaternion([data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z, data.pose[1].orientation.w])
        self.pose_theta = rotations_rads[2]


    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
	
        return sqrt(pow((goal_pose[0] - self.pose_x), 2) +
                    pow((goal_pose[1] - self.pose_y), 2))
   


    # Load the H5 Weights5
    with h5py.File('/home/eecs195/sahito_saad_ws/src/NN_TurtleBot/src/trained_model_MPC_weights.h5', 'r') as f:
	    
	    wet1 = str(f.keys()[0])
	    wet2 = str(f.keys()[1])
	    wet3 = str(f.keys()[2])
	    # Access the weights and biases for each layer
	    weights_1 = f[wet1][wet1]['kernel:0'][:]
	    biases_1 = f[wet1][wet1]['bias:0'][:]
	    weights_2 = f[wet2][wet2]['kernel:0'][:]
	    biases_2 = f[wet2][wet2]['bias:0'][:]
	    weights_output = f[wet3][wet3]['kernel:0'][:]
	    biases_output = f[wet3][wet3]['bias:0'][:]
    # Define your custom neural network architecture
    class MyNetwork:
	    def __init__(self):
		# Define the dimensions of the network
		self.input_size = 3
		self.hidden_size = 128
		self.output_size = 2

		# Initialize weights and biases
		self.W1 = np.zeros((self.input_size, self.hidden_size))
		self.b1 = np.zeros(self.hidden_size)
		self.W2 = np.zeros((self.hidden_size, self.hidden_size))
		self.b2 = np.zeros(self.hidden_size)
		self.W_output = np.zeros((self.hidden_size, self.output_size))
		self.b_output = np.zeros(self.output_size)

	    def forward(self, x):
		# Perform forward propagatio3
		h1 = np.dot(x, self.W1) + self.b1
		h1_relu = np.maximum(0, h1)
		h2 = np.dot(h1_relu, self.W2) + self.b2
		h2_relu = np.maximum(0, h2)
		y_pred = np.dot(h2_relu, self.W_output) + self.b_output
		return y_pred

	    def set_weights(self, weights_1, biases_1, weights_2, biases_2, weights_output, biases_output):
		# Set the loaded weights to the network
		self.W1 = weights_1
		self.b1 = biases_1
		self.W2 = weights_2
		self.b2 = biases_2
		self.W_output = weights_output
		self.b_output = biases_output

    # Create an instance of your custom network
    network = MyNetwork()

    # Set the loaded weights to the network
    network.set_weights(weights_1, biases_1, weights_2, biases_2, weights_output, biases_output)

    def NN_Controller(self, input_data):
	output_array = np.zeros(self.network.output_size)
        try:
	    
            # Preprocess the input_data if needed
            input_array = np.array([input_data])
            # Generate output using the network
            output_array = self.network.forward(input_array)
	    return output_array
        except rospy.ROSInterruptException:
            pass

    def move2goal(self):

        while not rospy.is_shutdown():

        
            goal_pose = [self.goal_pose_x, self.goal_pose_y, self.goal_pose_theta]

        
            distance_tolerance = 0.05

            vel_msg = Twist()
	    
            while (self.euclidean_distance(goal_pose) > distance_tolerance):
                #input_data = [(self.goal_pose_x-self.pose_x), (self.goal_pose_y-self.pose_y), self.pose_theta]
		input_data = [self.euclidean_distance(goal_pose), (self.goal_pose_x-self.pose_x), (self.goal_pose_y-self.pose_y)]
                NN_Output = self.NN_Controller(input_data)
		print(NN_Output)
                vel_msg.linear.x = NN_Output[0][0]
		vel_msg.angular.z = NN_Output[0][1]
                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)
		print(vel_msg)
                # Publish at the desired rate.
                self.rate.sleep()

            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)



if __name__ == '__main__':
    try:
        x = NN_Control()
        x.move2goal()
    except rospy.ROSInterruptException:
	pass
        








