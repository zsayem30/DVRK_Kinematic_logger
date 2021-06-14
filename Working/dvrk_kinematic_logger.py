#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import cv2
import numpy as np
import xlsxwriter
import dvrk 
import sys
from scipy.spatial.transform import Rotation as R
import os

if sys.version_info.major < 3:
    input = raw_input

class robot:

	def __init__(self, robot_name, ros_namespace = '/dvrk/'):
		"""Constructor. Initializes data members. It requires just a robot name. ex. r = robot('PSM1')"""
		self.__robot_name = robot_name
		self.__ros_namespace = ros_namespace
		self.__position_joint_current = []
		self.__measured_joint = []
		self.__position_jaw_current = []
		self.__position_cartesian_current = 0
		self.__orientation_current = []
		self.seq = 0
		self.rate = rospy.Rate(2)
		self.count = -1
		self.nsecs = 0
		self.secs = 0
		self.start_time = 0
		self.full_array = []

		full_ros_namespace = self.__ros_namespace + self.__robot_name
		
		#publisher
		self.set_position_joint = rospy.Publisher(self.__robot_name + '/move_jp', JointState, latch = True, queue_size = 1)

		#subscriber
		rospy.Subscriber(full_ros_namespace + '/state_joint_current', JointState, self.current_joint_state_callback, callback_args=(self.set_position_joint), queue_size = 1, buff_size = 1000000)
		rospy.Subscriber(self.__robot_name + '/measured_cp', TransformStamped, self.current_cartesian_position_callback, queue_size = 1, buff_size = 1000000)
		rospy.Subscriber(full_ros_namespace + '/state_jaw_current', JointState, self.current_jaw_state_callback, queue_size = 1, buff_size = 1)
		#callback

	def current_joint_state_callback(self, data, (pub)):
		self.__position_joint_current[:] = data.position
		self.full_array.append(self.__position_joint_current[:])
		self.seq = data.header.seq
		self.nsecs = data.header.stamp.nsecs
		self.secs = data.header.stamp.secs
		self.start_time = 0
		joint_state = JointState()
		joint_state.position[:] = data.position
		pub.publish(joint_state)

	def current_cartesian_position_callback(self, data):
		self.__position_cartesian_current = [data.transform.translation.x, data.transform.translation.y, data.transform.translation.z]
		self.__orientation_current = [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w]
	
	def current_jaw_state_callback(self, data):
		self.__position_jaw_current[:] = data.position

	#getters
	def get_current_cartesian_position(self):

		return self.__position_cartesian_current

	def get_current_orientation_matrix(self):

		r = R.from_quat(self.__orientation_current)
		rot_matrix = r.as_dcm().flatten()
		return rot_matrix

	def get_current_joint_position(self):
		
		return self.__position_joint_current

	def get_current_jaw_position(self):
		
		return self.__position_jaw_current
		
	def get_seq(self):
		
		return self.seq

	def increment_count(self):
		
		self.count = self.count + 1
		
		return self.count

	def get_nsecs(self):
		
		return self.nsecs*10**(-9)

	def get_secs(self):
		
		return self.secs

	def get_time(self):

		self.time = self.get_secs() + self.get_nsecs()
		return self.time

	def get_start_time(self):
		
		if self.count == 1:
			self.start_time = self.get_time()

		return self.start_time

	def get_count(self):
		return self.count

	def get_full_array(self):
		return self.full_array

		
class camera:


	def __init__(self, camera_name, ros_namespace = '/stereo/'):

		self.__camera_name = camera_name
		self.__ros_namespace = ros_namespace
		self.bridge = CvBridge()
		self.cv_image = []
		self.image_count = 1
		self.image_path = os.path.abspath(os.getcwd()) + '/Images/'


		full_ros_namespace = self.__ros_namespace + self.__camera_name

		#subscriber
		rospy.Subscriber(full_ros_namespace+ '/image_raw', Image, self.image_callback, queue_size = 1, buff_size = 1000000)

	def image_callback(self, data):

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	#saves the image in a folder.

		#getters
	def get_image(self):
		return self.cv_image

	def save_image(self):

		if self.cv_image.size != 0:
			cv2.imwrite(self.image_path + self.__camera_name+"/"+self.__camera_name+"_Camera" +"_" + str(self.image_count)+".png", self.cv_image)
			self.image_count = self.image_count + 1
			return True
		else:
			return False

if __name__ == '__main__':

	rospy.init_node('topic_publisher')
	rate = rospy.Rate(140)

	PSM1 = robot('PSM1')
	PSM2 = robot('PSM2')
	ECM = robot('ECM')
	left_cam = camera('left')
	right_cam = camera('right')

	input("		Press Enter to start logging...")

	workbook = xlsxwriter.Workbook('dvrk_kinematic_logger_test.xlsx')
	worksheet = workbook.add_worksheet()

	#start from the first cell
	#write the column headers for PSM1
	worksheet.write(0, 0, 'Time (Seconds)')
	worksheet.write(0, 1, 'Frame Number')
	worksheet.write(0, 2, 'PSM1_joint_1')
	worksheet.write(0, 3, 'PSM1_joint_2')
	worksheet.write(0, 4, 'PSM1_joint_3')
	worksheet.write(0, 5, 'PSM1_joint_4')
	worksheet.write(0, 6, 'PSM1_joint_5')
	worksheet.write(0, 7, 'PSM1_joint_6')
	worksheet.write(0, 8, 'PSM1_jaw_angle')
	worksheet.write(0, 9, 'PSM1_ee_x')
	worksheet.write(0, 10, 'PSM1_ee_y')
	worksheet.write(0, 11, 'PSM1_ee_z')

	worksheet.write(0, 12, 'PSM1_Orientation_Matrix_[1,1]')
	worksheet.write(0, 13, 'PSM1_Orientation_Matrix_[1,2]')
	worksheet.write(0, 14, 'PSM1_Orientation_Matrix_[1,3]')

	worksheet.write(0, 15, 'PSM1_Orientation_Matrix_[2,1]')
	worksheet.write(0, 16, 'PSM1_Orientation_Matrix_[2,2]')
	worksheet.write(0, 17, 'PSM1_Orientation_Matrix_[2,3]')

	worksheet.write(0, 18, 'PSM1_Orientation_Matrix_[3,1]')
	worksheet.write(0, 19, 'PSM1_Orientation_Matrix_[3,2]')
	worksheet.write(0, 20, 'PSM1_Orientation_Matrix_[3,3]')


	#write the column headers for PSM2
	worksheet.write(0, 21, 'PSM2_joint_1')
	worksheet.write(0, 22, 'PSM2_joint_2')
	worksheet.write(0, 23, 'PSM2_joint_3')
	worksheet.write(0, 24, 'PSM2_joint_4')
	worksheet.write(0, 25, 'PSM2_joint_5')
	worksheet.write(0, 26, 'PSM2_joint_6')
	worksheet.write(0, 27, 'PSM2_jaw_angle')

	worksheet.write(0, 28, 'PSM2_ee_x')
	worksheet.write(0, 29, 'PSM2_ee_y')
	worksheet.write(0, 30, 'PSM2_ee_z')

	worksheet.write(0, 31, 'PSM2_Orientation_Matrix_[1,1]')
	worksheet.write(0, 32, 'PSM2_Orientation_Matrix_[1,2]')
	worksheet.write(0, 33, 'PSM2_Orientation_Matrix_[1,3]')

	worksheet.write(0, 34, 'PSM2_Orientation_Matrix_[2,1]')
	worksheet.write(0, 35, 'PSM2_Orientation_Matrix_[2,2]')
	worksheet.write(0, 36, 'PSM2_Orientation_Matrix_[2,3]')

	worksheet.write(0, 37, 'PSM2_Orientation_Matrix_[3,1]')
	worksheet.write(0, 38, 'PSM2_Orientation_Matrix_[3,2]')
	worksheet.write(0, 39, 'PSM2_Orientation_Matrix_[3,3]')

	#write the column headers for ECM (4 joints)
	worksheet.write(0, 40, 'ECM_joint_1')
	worksheet.write(0, 41, 'ECM_joint_2')
	worksheet.write(0, 42, 'ECM_joint_3')
	worksheet.write(0, 43, 'ECM_joint_4')

	worksheet.write(0, 44, 'ECM_ee_x')
	worksheet.write(0, 45, 'ECM_ee_y')
	worksheet.write(0, 46, 'ECM_ee_z')

	worksheet.write(0, 47, 'ECM_Orientation_Matrix_[1,1]')
	worksheet.write(0, 48, 'ECM_Orientation_Matrix_[1,2]')
	worksheet.write(0, 49, 'ECM_Orientation_Matrix_[1,3]')

	worksheet.write(0, 50, 'ECM_Orientation_Matrix_[2,1]')
	worksheet.write(0, 51, 'ECM_Orientation_Matrix_[2,2]')
	worksheet.write(0, 52, 'ECM_Orientation_Matrix_[2,3]')

	worksheet.write(0, 53, 'ECM_Orientation_Matrix_[3,1]')
	worksheet.write(0, 54, 'ECM_Orientation_Matrix_[3,2]')
	worksheet.write(0, 55, 'ECM_Orientation_Matrix_[3,3]')
	#write the column headers for the cameras
	worksheet.write(0, 56, 'Left Camera Image')
	worksheet.write(0, 57, 'Right Camera Image')

	i = 0

	start_time = 0

	def callback_dummy(data):
		global i, start_time

		PSM1_jp = PSM1.get_current_joint_position()
		PSM1_jaw_angle = PSM1.get_current_jaw_position()
		PSM1_ee = PSM1.get_current_cartesian_position()
		PSM1_Orientation_Matrix = PSM1.get_current_orientation_matrix()

		PSM2_jp = PSM2.get_current_joint_position()
		PSM2_jaw_angle = PSM2.get_current_jaw_position()
		PSM2_ee = PSM2.get_current_cartesian_position()
		PSM2_Orientation_Matrix = PSM2.get_current_orientation_matrix()

		ECM_jp = ECM.get_current_joint_position()
		ECM_ee = ECM.get_current_cartesian_position()
		ECM_Orientation_Matrix = ECM.get_current_orientation_matrix()

		image_left = left_cam.get_image()
		image_right = right_cam.get_image()

		PSM1_data = np.concatenate((PSM1_jp, PSM1_jaw_angle, PSM1_ee, PSM1_Orientation_Matrix), axis = 0)
		PSM2_data = np.concatenate((PSM2_jp, PSM2_jaw_angle, PSM1_ee, PSM1_Orientation_Matrix), axis = 0)
		ECM_data = np.concatenate((ECM_jp, ECM_ee, ECM_Orientation_Matrix), axis = 0)
		print(ECM_data)
		all_data = np.concatenate((PSM1_data, PSM2_data, ECM_data), axis = 0)

		time = data.header.stamp.secs + data.header.stamp.nsecs*10**(-9)

		if i != 0:
			Sequence = i
			time = time - start_time
			info_frame = [time, Sequence]

			all_data = np.concatenate((info_frame, PSM1_data, PSM2_data, ECM_data), axis = 0)

			for col in range(len(all_data)):
				worksheet.write(i, col, all_data[col])

			image_saved_left = left_cam.save_image()

			if image_saved_left == True:
				worksheet.write(i, 56, "left"+"_Camera" +"_" + str(i)+".png")

			image_saved_right = right_cam.save_image()

			if image_saved_right == True:
				worksheet.write(i, 57, "right"+"_Camera" +"_" + str(i)+".png")

		else:
			start_time = time
		i = i + 1

	rospy.Subscriber('/dvrk/PSM1/state_jaw_current', JointState, callback_dummy, queue_size = 1, buff_size = 1000000)

	try:
		rospy.spin()
	except rospy.ROSInterruptException as e:
		print("Error Running ROS." + e)
		pass

	workbook.close()