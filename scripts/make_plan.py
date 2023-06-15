#! /usr/bin/env python3
import rospy

import time
import math
import numpy as np
import datetime
import sys

from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import actionlib
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospkg

GOAL_X = 7.5 #meters
GOAL_Y = -6.5  #meters
GOAL_YAW = 90.0 #degree

GOAL_POSITION = Pose()
GOAL_POSITION.position.x = GOAL_X
GOAL_POSITION.position.y = GOAL_Y
q = quaternion_from_euler(0, 0, math.radians(GOAL_YAW))
GOAL_POSITION.orientation.x = q[0]
GOAL_POSITION.orientation.y = q[1]
GOAL_POSITION.orientation.z = q[2]
GOAL_POSITION.orientation.w = q[3]

rospy.loginfo("Goal defined! x: %.03f y: %0.3f yaw: %.03f" %(GOAL_X, GOAL_Y, GOAL_YAW))

class trajectory_details:
	def __init__(self):
		rospy.init_node('trajectory_details_recorder')

		self.loadParams()
		# self.trajectory_path_msg = Path()
		# self.previous_pose_position = Point()
		self.current_pose = Pose()
		self.global_plan_path = Path()
		rospy.loginfo("Init Subscriber")
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

		rospy.wait_for_service('/move_base/make_plan')
		rospy.loginfo("/move_base/GlobalPlanner/make_plan service is ready!")
		self.get_global_plan_client = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
		self.global_plan_pub = rospy.Publisher("/global_path_plan", Path, queue_size=1)
		self.initialpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
		rospy.wait_for_service('/move_base/clear_costmaps')
		self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

		rospy.loginfo("Ready!")
		rospy.loginfo("Creating a path plan to goal...")

		rospack = rospkg.RosPack()
		package_path = rospack.get_path("trajectory_details_recorder")
		suffix = datetime.datetime.now().strftime("%m_%d_%Y-%H_%M_%S")

		save_to_filename = "%s/data/data_of_%s_at_%s.txt" % (package_path, self.method_name_, suffix)
		self.write_to_file(save_to_filename, "w")
		self.generate_plan(GOAL_POSITION, save_to_filename)
		self.move_base_waypoint(GOAL_POSITION.position.x, GOAL_POSITION.position.y, GOAL_POSITION.orientation)
		rospy.loginfo("Done.")
		sys.exit()

	def move_base_waypoint(self, x, y, quaternion, duration = -1.0):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id= "map"
		goal.target_pose.header.stamp= rospy.Time.now()
		#goal.target_pose.pose.position.x= 1.0
		#goal.target_pose.pose.position.y= 0.0
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y
		goal.target_pose.pose.position.z = 0.0
		# goal.target_pose.pose.orientation.w = 1.0
		goal.target_pose.pose.orientation.x = quaternion.x
		goal.target_pose.pose.orientation.y = quaternion.y
		goal.target_pose.pose.orientation.z = quaternion.z
		goal.target_pose.pose.orientation.w = quaternion.w

		self.move_base_client.send_goal(goal)
		if (duration == -1):
			wait = self.move_base_client.wait_for_result()
		else:
			wait = self.move_base_client.wait_for_result(rospy.Duration(duration))

		print(self.move_base_client.get_result())
		# cancel_goal_pub.publish() #try to change to cancelAllGoal function
		# rospy.sleep(0.2)

		if not wait:
			rospy.logerr("Action server not available!")
		    # rospy.signal_shutdown("Action server not available!")

		rospy.loginfo("Finished")
		return self.move_base_client.get_result()

	def generate_plan(self, goal_position, filename):
		current_pose_stamped = PoseStamped()
		current_pose_stamped.header.stamp = rospy.Time.now()
		current_pose_stamped.header.frame_id = "map"
		current_pose_stamped.pose = self.current_pose

		goal_pose_stamped = PoseStamped()
		goal_pose_stamped.header.stamp = rospy.Time.now()
		goal_pose_stamped.header.frame_id = "map"
		goal_pose_stamped.pose = goal_position

		start = time.time()
		response = self.get_global_plan_client(current_pose_stamped, goal_pose_stamped, 0.1)
		elapsed_time = time.time() - start
		#rospy.loginfo("Elapsed Time: %f s", elapsed_time) #
		rospy.loginfo("Elapsed Time: %.05f s", elapsed_time) 
		
		self.global_plan_path = response.plan
		path_length = self.calculate_path_length(response.plan)

		text_data = "[Planning] elapsed_time(s): %.05f - length(m): %.05f\n" % (elapsed_time, path_length)
		self.write_to_file(filename, "a", text_data)

	def write_to_file(self, data_path, type, msg=""):
		if type=="w":
			text = msg
		elif type=="a":
			text = msg
		else:
			return False
		file = open(data_path, type)
		file.write(text)
		file.close()


	#=====================================
	#       function for loading
	#    from ROS parameter server
	#=====================================
	def loadParams(self):
		rospy.loginfo("Loading Params")
		#Check method
		self.rrt_star_method_ = rospy.get_param('/move_base/RRTStarPlanner/method')
		if (self.rrt_star_method_ == 0):
			self.method_name_ = "RRT_STAR"
		elif (self.rrt_star_method_ == 1):
			self.method_name_ = "RRT"
		elif (self.rrt_star_method_ == 2):
			self.method_name_ = "INFORMED RRT STAR"
		print("RRT Star Method: %s" %self.method_name_)


	#=====================================
	#      Callback function when
	#  receiving odometry message
	#=====================================
	def odom_callback(self, odom_msg):
		# rospy.loginfo("received odom message with position: " + str(odom_msg.pose.pose.position))
		#Process message position and add it to path
		self.current_pose.position = odom_msg.pose.pose
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
	rospy.loginfo("starting trajectory details recorder node")
	trajectoryPath = trajectory_details()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
