#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray

from rrt import find_path_RRT


class RRTNode:
    def __init__(self):
        rospy.init_node('RRT_node')
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber('/start_goal', Float64MultiArray, self.goal_callback)
        self.traj_pub = rospy.Publisher('/trajectory', Float64MultiArray, queue_size=10)
        self.current_map = None
        self.x_origin = None
        self.y_origin = None
        self.resolution = None

    def map_callback(self, msg):
        self.current_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.x_origin = msg.info.origin.position.x
        self.y_origin = msg.info.origin.position.y
        self.resolution = msg.info.resolution

    def goal_callback(self, msg):
        x_start_real, y_start_real, x_goal_real, y_goal_real = msg.data
        
	x_start_index, y_start_index = self.get_index_from_coordinates(x_start_real, y_start_real)
        x_goal_index, y_goal_index = self.get_index_from_coordinates(x_goal_real, y_goal_real)
        start, goal = ([x_start_index, y_start_index], [x_goal_index, y_goal_index])
        path, _ = find_path_RRT(start, goal, self.current_map)
        traj_msg = Float64MultiArray()
	print(path)
        traj_msg.data = [coord for point in path for coord in self.get_coordinates_from_index(point)]
        self.traj_pub.publish(traj_msg)

    
    def get_index_from_coordinates(self, x_real, y_real):
        x_index = int(round((x_real - self.x_origin) / self.resolution))
        y_index = int(round((y_real - self.y_origin) / self.resolution))
        return x_index, y_index

    def get_coordinates_from_index(self, point):
        x_real = point[0] * self.resolution + self.x_origin
        y_real = point[1] * self.resolution + self.y_origin
        return x_real, y_real


if __name__ == '__main__':
    try:
        node = RRTNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


