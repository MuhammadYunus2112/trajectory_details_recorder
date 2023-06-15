#! /usr/bin/env python3
import rospy

import time
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from geometry_msgs.msg import Point

print("OK")

class trajectory_path:
	#=====================================
	#         Class constructor
	#  Initializes node and subscribers
	#=====================================
	def __init__(self):
		rospy.init_node('pose_to_path')

		self.loadParams()
		self.trajectory_path_msg = Path()
		self.previous_pose_position = Point()

		rospy.loginfo("Init Publisher")
		#Setup trajectory path publisher
		self.trajectory_path_pub = rospy.Publisher('/robot_path', Path, queue_size=10, latch=True)
		#Setup subscriber to odometry
		rospy.loginfo("Init Subscriber")
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
		rospy.loginfo("Ready")


	#=====================================
	#       function for loading
	#    from ROS parameter server
	#=====================================
	def loadParams(self):
		rospy.loginfo("Loading Params")
		#Load maximum number of poses in actual path
		self.max_poses = rospy.get_param('~max_poses', -1)
		#Load threshold for adding a pose to actual path
		self.threshold = rospy.get_param('~movement_threshold', 0.01)
		#Load parent frame id for the trajectory
		self.frame_id = rospy.get_param('~frame_id', 'odom')


	#=====================================
	#      Callback function when
	#  receiving odometry message
	#=====================================
	def odom_callback(self, odom_msg):
		# rospy.loginfo("received odom message with position: " + str(odom_msg.pose.pose.position))
		#Process message position and add it to path
		if odom_msg.header.frame_id == self.frame_id :
			self.publish_trajectory_path(odom_msg.pose.pose.position)
		else:
			rospy.logerr("Odometry message frame:"+odom_msg.header.frame_id+" does not correspond to trajectory frame"+self.frame_id)

	def calculate_path_length(self, path_msg):
		rospy.loginfo("Calculating Path Length...")
		path_length = 0
		start = time.time()
		for i in range(len(path_msg.poses) - 1):
			position_a_x = path_msg.poses[i].pose.position.x
			position_b_x = path_msg.poses[i+1].pose.position.x
			position_a_y = path_msg.poses[i].pose.position.y
			position_b_y = path_msg.poses[i+1].pose.position.y

			path_length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2))
		elapsed_time = time.time() - start
		rospy.loginfo("Elapsed Time: %f s", elapsed_time) 
		return path_length

	#=====================================
	#        Add pose and publish 
	#      trajectory path message
	#=====================================
	def publish_trajectory_path(self, position):
	#If the pose has move more than a set threshold, add it to the path message and publish
		if ((abs(self.previous_pose_position.x - position.x) > self.threshold)
		 or (abs(self.previous_pose_position.y - position.y) > self.threshold)
		 or (abs(self.previous_pose_position.z - position.z) > self.threshold)):
			rospy.loginfo('Exceding threshold, adding pose to path')

			#Add current pose to path
			self.trajectory_path_msg.header.stamp = rospy.Time.now()
			self.trajectory_path_msg.header.frame_id = self.frame_id
			pose_stamped_msg = PoseStamped()
			pose_stamped_msg.header.stamp = rospy.Time.now()
			pose_stamped_msg.pose.position.x = position.x
			pose_stamped_msg.pose.position.y = position.y
			pose_stamped_msg.pose.position.z = position.z
			pose_stamped_msg.pose.orientation.w = 1.0

			#If max number of poses in path has not been reach, just add pose to message
			if self.max_poses == -1 or len(self.trajectory_path_msg.poses) < self.max_poses:
				self.trajectory_path_msg.poses.append(pose_stamped_msg)
			#Else rotate the list to dismiss oldest value and add newer value at the end
			else:
				rospy.loginfo('Max number of poses reached, erasing oldest pose')
				self.trajectory_path_msg.poses = self.trajectory_path_msg.poses[1:]
				self.trajectory_path_msg.poses.append(pose_stamped_msg)

			self.previous_pose_position = pose_stamped_msg.pose.position
			self.trajectory_path_pub.publish(self.trajectory_path_msg)
			rospy.loginfo("Path Length: %.03f", self.calculate_path_length(self.trajectory_path_msg))


#=====================================
#               Main
#=====================================
if __name__ == '__main__':
	rospy.loginfo("starting trajectory path publisher")
	trajectoryPath = trajectory_path()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
