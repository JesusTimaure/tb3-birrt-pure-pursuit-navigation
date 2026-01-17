#!/usr/bin/env python3
import rospy
import actionlib
import numpy as np
from project_timaure.msg import ControllerFeedback, ControllerResult, ControllerAction
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
from math import atan2, sqrt, pow, sin

class PurePursuit(object):
	_feedback = ControllerFeedback()
	_result = ControllerResult()
	
	def __init__(self, lookahead_distance, Ts_robot, goal_tolerance):
		#waffle
		#self.v_max = 0.26				
		#self.w_max = 1.82
		#burger
		self.v_max = 0.22
		self.w_max = 2.84
		self.Ts_robot = Ts_robot
		self.goal_tolerance = goal_tolerance
		self.x = np.zeros(3)
		self.lookahead_distance = lookahead_distance
		self.path = []
		rospy.Subscriber("/odom", Odometry, self.odom_callback)
		rospy.Subscriber("/local_planner/path", Path, self.path_callback)
		#rospy.Subsriber
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
		self._as = actionlib.SimpleActionServer("controller_as", ControllerAction, self.goal_callback, False)
		self._as.start()
	@staticmethod
	def quaternion_to_yaw(q):
		return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
	
	def odom_callback(self, msg):
		q = msg.pose.pose.orientation
		self.x[0] = msg.pose.pose.position.x
		self.x[1] = msg.pose.pose.position.y
		self.x[2] = self.quaternion_to_yaw(q)
		
	def path_callback(self, msg):
		self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
		
	def euclidean_distance(self, p1, p2):
		distance = np.linalg.norm(np.array(p1) - np.array(p2))
		return distance
		
	def find_lookahead_point(self):
		if not self.path:
			rospy.logwarn("No local path was received")
			return None
		for point in self.path:
			if self.euclidean_distance(self.x[0:2], point) >= self.lookahead_distance:
				return point
		return self.path[-1]
	
	def compute_control_command(self, target_point):
		dx = target_point[0] - self.x[0]
		dy = target_point[1] - self.x[1]
		alpha = atan2(dy, dx) - self.x[2]
		alpha = (alpha + np.pi)%(2*np.pi)-np.pi
		if abs(alpha)>np.pi/2:
			linear_speed = 0.0
			angular_speed = np.clip(self.w_max*(alpha/abs(alpha)), -self.w_max, self.w_max)
		else:
			k = 2*sin(alpha)/self.lookahead_distance
			linear_speed = self.v_max*(1-min(abs(k),1))
			linear_speed = max(0.19, linear_speed)
			#linear_speed = self.v_max
			angular_speed = k*abs(linear_speed)
			angular_speed = np.clip(angular_speed, -self.w_max, self.w_max)
		return linear_speed, angular_speed

	
	def goal_callback(self, goal):			
		r = rospy.Rate(1/self.Ts_robot)
		success = True
		target_point = (goal.target_point.x, goal.target_point.y)
		while True:
			# Check preemption
			if self._as.is_preempt_requested():
				rospy.loginfo("The goal has been cancelled/preempted")
				self._as.set_preempted()
				success = False
				break
			lookahead_point = self.find_lookahead_point()
			if not lookahead_point:
				rospy.logwarn("No path to follow")
				success = False
				break
			linear_speed, angular_speed = self.compute_control_command(lookahead_point)
			control_msg = Twist()
			control_msg.linear.x = linear_speed
			control_msg.angular.z = angular_speed
			self.pub.publish(control_msg)
			
			distance_to_goal = self.euclidean_distance(self.x[0:2], target_point)
			self._feedback.fb = Float64(distance_to_goal)
			self._as.publish_feedback(self._feedback)
			if distance_to_goal <= self.goal_tolerance:
				break
			r.sleep()
		control_msg = Twist()
		self.pub.publish(control_msg)
		
		if success:
			self._result.res = Float64(0.0)
			self._as.set_succeeded(self._result)
			
if __name__ == "__main__":
	rospy.init_node("controller_server_node")
	lookahead_distance = rospy.get_param("~lookahead_distance")
	Ts_robot = rospy.get_param("~Ts_robot")
	goal_tolerance = rospy.get_param("~goal_tolerance")
	PurePursuit(lookahead_distance, Ts_robot, goal_tolerance)
	rospy.spin()
