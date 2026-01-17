#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import time

class FakeScheduler(object): #class constructor
	def __init__(self, min_distance):
		self.min_distance = min_distance
		self.p0 = []
		self.last_goal_idx = None
		self.goals = [[-5, -0.5, -1], [-1.5, -2.9, 1], [-5, -0.5, 1],[0.5, 2, 1], [-3, 6, -1], [0.5, 2.4, 1], [5.25, 5.5, 1]] 
		rospy.Subscriber("/odom", Odometry, self.odom_callback)
		self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=2)
		self.start_time = 0
		self.no_other_goals = False
	def send_goal(self, p):
		goal = PoseStamped()
		goal.pose.position.x = p[0]
		goal.pose.position.y = p[1]
		goal.pose.orientation.w = p[2]
		goal.header.frame_id = "map" 
		self.goal_pub.publish(goal)
	def odom_callback(self, msg):
		self.p0 = [msg.pose.pose.position.x, msg.pose.pose.position.y]
	def select_goal(self):
		if self.no_other_goals:
			return
		if self.last_goal_idx is None:
			self.last_goal_idx = 0
			self.send_goal(self.goals[self.last_goal_idx])
			self.start_time = time.time()
			return
		current_goal = np.array(self.goals[self.last_goal_idx])
		if np.linalg.norm(current_goal[0:2] - np.array(self.p0)) <= self.min_distance * 3:
			if self.last_goal_idx == 1 or self.last_goal_idx == 4 or self.last_goal_idx == 6:
				end_time = time.time() - self.start_time
				rospy.logwarn("Goal achieved in %0.4f sec" % end_time) 
			if self.last_goal_idx == len(self.goals) - 1:
				self.no_other_goals = True
				return
			if self.last_goal_idx == 1 or self.last_goal_idx == 4 or self.last_goal_idx == 6:
				self.start_time = time.time()
			self.last_goal_idx += 1
			self.send_goal(self.goals[self.last_goal_idx])
			return

def main():
	rospy.init_node("scheduler_node", anonymous=True)
	Ts_robot = rospy.get_param('~Ts_robot')
	min_distance = rospy.get_param('~min_distance')
	rate = rospy.Rate(1/Ts_robot)
	node = FakeScheduler(min_distance)
	rospy.wait_for_message("map", OccupancyGrid)
	rospy.wait_for_message("odom", Odometry)
	time.sleep(10) #delay execution by 10 seconds
	while not rospy.is_shutdown():
		node.select_goal()
		rate.sleep()

if __name__ == '__main__':
	main()
