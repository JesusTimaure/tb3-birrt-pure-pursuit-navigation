#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
import actionlib
from project_venturino.msg import ControllerGoal, ControllerAction

class LocalPlanner(object):
	def __init__(self, action_server_name, increment, resolution, min_distance):
		self.action_server_name = action_server_name
		self.client = actionlib.SimpleActionClient(action_server_name, ControllerAction)
		self.current_p = []
		self.current_path = []
		self.current_waypoint_idx = None
		self.R = increment * resolution
		self.min_distance = min_distance
		rospy.Subscriber("global_planner/path", Path, self.path_callback)
		rospy.Subscriber("odom", Odometry, self.odom_callback)
		self.path_pub = rospy.Publisher("local_planner/path", Path, queue_size=2)

	def path_callback(self, msg):
		self.current_path = []
		for i in range(0, len(msg.poses)):
			p = [msg.poses[i].pose.position.x, msg.poses[i].pose.position.y, msg.poses[i].pose.position.z]
			self.current_path.append(p)
		self.current_waypoint_idx = None

	@staticmethod
	def feedback_callback(feedback):
		rospy.loginfo(feedback.state)

	def publish_path(self, x):
		if len(x) < 1:
			return
		header = Header()
		header.frame_id = "map"
		header.stamp = rospy.Time.now()
		path_msg = Path()
		path_msg.header = header
		for i in range(0, len(x)):
			pose_stamped = PoseStamped()
			pose_stamped.pose.position.x = x[i][0]
			pose_stamped.pose.position.y = x[i][1]
			pose_stamped.pose.position.z = x[i][2]
			path_msg.poses.append(pose_stamped)
		self.path_pub.publish(path_msg)

	def send_goal(self, point):
		goal = ControllerGoal()
		goal.target_point.x = point[0]
		goal.target_point.y = point[1]
		self.client.send_goal(goal)

	def select_waypoint(self):
		path = np.array(self.current_path)
		if self.current_waypoint_idx is None:
			self.current_waypoint_idx = 0
			next_point = path[0]
			points = [next_point]
			self.publish_path(points)
			rospy.sleep(0.1)
			self.send_goal(next_point)
			return
		idx_final = path.shape[0] - 1
		final_point = self.current_path[idx_final]
		if np.linalg.norm(final_point[0:2] - np.array(self.current_p)) < self.min_distance:
			self.current_path = []
			self.current_waypoint_idx = None
			return
		idx_next = min(self.current_waypoint_idx + 1, path.shape[0] - 1)
		next_point = self.current_path[idx_next]
		if np.linalg.norm(next_point[0:2] - np.array(self.current_p)) <= self.R * 3:
			points = [self.current_path[self.current_waypoint_idx], next_point]
			self.current_waypoint_idx = idx_next
			self.publish_path(points)
			rospy.sleep(0.1)
			self.send_goal(next_point)

	def odom_callback(self, msg):
		self.current_p = [msg.pose.pose.position.x, msg.pose.pose.position.y]

def main():
	rospy.init_node("local_planner_node", anonymous=True)
	Ts = rospy.get_param('~Ts_robot')
	rate = rospy.Rate(1 / Ts)
	increment = rospy.get_param('~increment')
	min_distance = rospy.get_param('~min_distance')
	resolution = rospy.get_param('~resolution')
	rospy.wait_for_message("odom", Odometry)

	action_server_name = "controller_as"
	node = LocalPlanner(action_server_name, increment, resolution, min_distance)

	rospy.loginfo('Waiting for Controller ' + action_server_name)
	node.client.wait_for_server()
	rospy.loginfo('Controller found...' + action_server_name)

	while not rospy.is_shutdown():
		if len(node.current_path):
			node.select_waypoint()

		rate.sleep()


if __name__ == '__main__':
	main()
