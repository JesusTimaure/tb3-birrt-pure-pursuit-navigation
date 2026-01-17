#!/usr/bin/env python3

import time
import rospy
import numpy as np
import random
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from scipy.ndimage import binary_dilation
import scipy.interpolate as interp
from std_msgs.msg import ColorRGBA


class Node:
	def __init__(self, pos, parent=None):
		self.pos = np.array(pos)
		self.parent = parent


class BiRRTPlanner:
	def __init__(self, inflate_radius, step_size, max_iterations, clearance = 0.15):
		self.inflate_radius = inflate_radius
		self.step_size = step_size
		self.max_iterations = max_iterations
		self.clearance = clearance

		self.tree_pub = rospy.Publisher("/global_planner/markers", MarkerArray, queue_size=2)
		self.costmap_pub = rospy.Publisher("/global_costmap", OccupancyGrid, queue_size=1)
		self.path_pub = {"g_path": rospy.Publisher("/global_planner/path", Path, queue_size=2),
                         "r_path": rospy.Publisher("/global_planner/raw_path", Path, queue_size=2)}
		self.width = 0
		self.height = 0
		self.resolution = 0
		self.origin = None
		self.costmap = None
		self.map_received = False
		self.start = None
		self.goal = None

		rospy.Subscriber("/goal", PoseStamped, self.goal_callback)
		rospy.Subscriber("/odom", Odometry, self.start_callback)

	def goal_callback(self, msg):
		rospy.loginfo("New target goal received!")
		self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
		odom_msg = rospy.wait_for_message("/odom", Odometry)
		self.start = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])
		success = self.plan_path()
		if not success:
			rospy.loginfo("Unable to generate path")

	def start_callback(self, msg):
		self.start = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

	def receive_map(self):
		msg = rospy.wait_for_message("map", OccupancyGrid)
		self.width = msg.info.width
		self.height = msg.info.height
		self.resolution = msg.info.resolution
		self.origin = msg.info.origin

		inflate_radius_cells = int(self.inflate_radius / self.resolution)
		self.costmap = np.array(np.reshape(msg.data, (self.height, self.width)) / 100, dtype=bool)
		self.costmap = binary_dilation(self.costmap, iterations=inflate_radius_cells)
		self.map_received = True
		self.publish_global_costmap()

	def publish_global_costmap(self):
		occupancy_grid = OccupancyGrid()
		occupancy_grid.header.stamp = rospy.Time.now()
		occupancy_grid.header.frame_id = "map"
		occupancy_grid.info.width = self.width
		occupancy_grid.info.height = self.height
		occupancy_grid.info.resolution = self.resolution
		occupancy_grid.info.origin = self.origin
		occupancy_grid.data = self.costmap.flatten().astype(int).tolist()
		self.costmap_pub.publish(occupancy_grid)

	def is_collision_free(self, point):
		x, y = point
		ix = int((x - self.origin.position.x) / self.resolution)
		iy = int((y - self.origin.position.y) / self.resolution)
		if 0 <= ix < self.width and 0 <= iy < self.height:
			clearance_cells = int(self.clearance/self.resolution)
			for dx in range(-clearance_cells, clearance_cells+1):
				for dy in range(-clearance_cells, clearance_cells+1):
					nx, ny = ix+dx, iy+dy
					if 0 <= nx < self.width and 0 <= ny < self.height:
						if self.costmap[ny, nx]:
							return False
			return True
		return False

	def get_random_point(self, goal_bias=0.35):
		while True:
			if random.uniform(0, 1) < goal_bias and self.goal is not None:
				point = self.goal + np.random.normal(0, self.step_size, size=2)
			else:
				x = random.uniform(self.origin.position.x, self.origin.position.x + self.width * self.resolution)
				y = random.uniform(self.origin.position.y, self.origin.position.y + self.height * self.resolution)
				point = np.array([x, y])
			if self.is_collision_free(point):  # Buffer check
				return point

	def steer(self, from_node, to_point):
		direction = to_point - from_node.pos
		distance = np.linalg.norm(direction)
		if distance > self.step_size:
			direction = (direction / distance) * self.step_size
		new_pos = from_node.pos + direction
		if self.is_collision_free_path(from_node.pos, new_pos) and self.is_collision_free(new_pos):
			return Node(new_pos, parent=from_node)
		return None

	def find_nearest(self, tree, target):
		nearest_node = None
		min_dist = float("inf")
		for node in tree:
			dist = np.linalg.norm(node.pos - target)
			if dist < min_dist:
				nearest_node = node
				min_dist = dist
		return nearest_node

	def build_path(self, start_tree, goal_tree, start_connection, goal_connection):
		start_path = []
		current = start_connection
		while current:
			start_path.append(current.pos)
			current = current.parent

		start_path.reverse()
		start_path.append(goal_connection.pos)
        
		goal_path = []
		current = goal_connection
		while current:
			goal_path.append(current.pos)
			current = current.parent

		return start_path + goal_path

	def plan_path(self):
		if self.start is None or self.goal is None:
			rospy.logwarn("Start or goal not defined!")
			return
        
		for retry in range(20):
			start_tree = [Node(self.start)]
			goal_tree = [Node(self.goal)]
			connection_threshold = self.step_size

			for iteration in range(self.max_iterations):
				rand_point = self.get_random_point()
				nearest_start = self.find_nearest(start_tree, rand_point)
				new_start = self.steer(nearest_start, rand_point)
				if new_start and self.is_collision_free(new_start.pos):
					start_tree.append(new_start)
					nearest_goal = self.find_nearest(goal_tree, new_start.pos)
					if np.linalg.norm(new_start.pos - nearest_goal.pos) < connection_threshold:
						raw_path = self.build_path(start_tree, goal_tree, new_start, nearest_goal)
						if raw_path:
							self.publish_path(raw_path, "r_path")
							smoothed_path = self.smooth_path(raw_path)
							interpolated_path = self.interpolate_path(smoothed_path, self.step_size)
							self.publish_path(interpolated_path, "g_path")
							self.publish_trees(start_tree, goal_tree)
							return True

				rand_point = self.get_random_point()
				nearest_goal = self.find_nearest(goal_tree, rand_point)
				new_goal = self.steer(nearest_goal, rand_point)
				if new_goal and self.is_collision_free(new_goal.pos):
					goal_tree.append(new_goal)
					nearest_start = self.find_nearest(start_tree, new_goal.pos)
					if np.linalg.norm(new_goal.pos - nearest_start.pos) < connection_threshold:
						raw_path = self.build_path(start_tree, goal_tree, nearest_start, new_goal)
						if raw_path:
							self.publish_path(raw_path, "r_path")
							smoothed_path = self.smooth_path(raw_path)
							interpolated_path = self.interpolate_path(smoothed_path, self.step_size)
							self.publish_path(interpolated_path, "g_path")
							self.publish_trees(start_tree, goal_tree)
							return True

			rospy.logwarn("Failed to connect trees after maximum number of attempts!")
			self.publish_trees(start_tree, goal_tree)
			return False
    
	def smooth_path(self, path):
		reversed_path = path[::-1]
		smoothed_path = [reversed_path[0]]
		for i in range(1, len(reversed_path)):
			if not self.is_collision_free_path(smoothed_path[-1], reversed_path[i]):
				smoothed_path.append(reversed_path[i - 1])
		smoothed_path.append(reversed_path[-1])
		smoothed_path.reverse()
		return smoothed_path

	def is_collision_free_path(self, start, end):
		distance = np.linalg.norm(end - start)
		steps = max(1, int(distance / self.resolution))
		for i in range(steps + 1):
			intermediate_point = start + (i / steps) * (end - start)
			if not self.is_collision_free(intermediate_point):
				return False
		return True

	def publish_trees(self, start_tree, goal_tree):
		marker_array = MarkerArray()
		clear_marker = Marker()
		clear_marker.action = Marker.DELETEALL
		marker_array.markers.append(clear_marker)
		id_counter = 0  # Unique ID counter for markers
		
		def add_tree_markers(tree, color):
			nonlocal id_counter
			for node in tree:
				marker = Marker()
				marker.header.frame_id = "map"
				marker.header.stamp = rospy.Time.now()
				marker.id = id_counter  # Unique ID for each marker
				id_counter += 1  # Increment the ID counter
				marker.type = Marker.SPHERE
				marker.action = Marker.ADD
				marker.pose.position.x = node.pos[0]
				marker.pose.position.y = node.pos[1]
				marker.pose.position.z = 0.0
				marker.pose.orientation.x = 0.0
				marker.pose.orientation.y = 0.0
				marker.pose.orientation.z = 0.0
				marker.pose.orientation.w = 1.0
				marker.scale.x = marker.scale.y = marker.scale.z = 0.1
				marker.color = color
				marker_array.markers.append(marker)

        # Add markers for the start and goal trees
		add_tree_markers(start_tree, ColorRGBA(0.0, 1.0, 0.0, 0.5))  # Green for start tree
		add_tree_markers(goal_tree, ColorRGBA(1.0, 0.0, 0.0, 0.5))  # Red for goal tree

        # Publish the MarkerArray
		self.tree_pub.publish(marker_array)

	def interpolate_path(self, path, max_distance):
		# Extract x and y coordinates
		x = [point[0] for point in path]
		y = [point[1] for point in path]

		# Calculate cumulative distance along the path
		distances = np.cumsum([0] + [np.linalg.norm(np.array([x[i], y[i]]) - np.array([x[i - 1], y[i - 1]])) for i in range(1, len(x))])
		total_length = distances[-1]

		# Interpolate at evenly spaced intervals
		num_points = int(np.ceil(total_length / max_distance)) + 1
		interp_distances = np.linspace(0, total_length, num_points)

		# Create splines for interpolation
		spline_x = interp.interp1d(distances, x, kind='linear')
		spline_y = interp.interp1d(distances, y, kind='linear')
		
		# Generate interpolated path
		interpolated_path = [(spline_x(d), spline_y(d)) for d in interp_distances]
		return interpolated_path

    
	def publish_path(self, path, topic = ""):
		path_msg = Path()
		path_msg.header.frame_id = "map"
		path_msg.header.stamp = rospy.Time.now()
		for point in path:
			pose_stamped = PoseStamped()
			pose_stamped.pose.position.x = point[0]
			pose_stamped.pose.position.y = point[1]
			path_msg.poses.append(pose_stamped)
		self.path_pub[topic].publish(path_msg)



def main():
	rospy.init_node('bi_rrt_global_planner', anonymous=True, log_level=rospy.DEBUG)
	inflate_radius = rospy.get_param('~inflate_radius')
	step_size = rospy.get_param('~step_size')
	max_iterations = rospy.get_param('~max_iterations')
	planner = BiRRTPlanner(inflate_radius, step_size, max_iterations)
	planner.receive_map()
	rospy.spin()

if __name__ == '__main__':
	main()

