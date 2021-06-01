#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import cv2
import numpy as np
import xlsxwriter
import dvrk

class robot:

	def __init__(self, robot_name, ros_namespace = '/dvrk/'):
		"""Constructor. Initializes data members. It requires just a robot name. ex. r = robot('PSM1')"""
		self.__robot_name = robot_name
		self.__ros_namespace = ros_namespace
		self.__position_joint_current = []
		self.seq = 0

		full_ros_namespace = self.__ros_namespace + self.__robot_name

		#subscriber
		rospy.Subscriber(full_ros_namespace + '/state_joint_current', JointState, self.current_joint_state_callback, queue_size = 1, buff_size = 1000000)

		#callback
	def current_joint_state_callback(self, data):
		self.__position_joint_current[:] = data.position
		self.seq = data.header.seq

		#getters
	def get_current_joint_position(self):
		return self.__position_joint_current

	def get_seq(self):
		return self.seq

class camera:

	def __init__(self, camera_name, ros_namespace = '/stereo/'):

		self.__camera_name = camera_name
		self.__ros_namespace = ros_namespace
		self.bridge = CvBridge()
		self.cv_image = []

		full_ros_namespace = self.__ros_namespace + self.__camera_name

		#subscriber
		rospy.Subscriber(full_ros_namespace+ '/image_raw', Image, self.image_callback, queue_size = 1, buff_size = 1000000)

	def image_callback(self, data):

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		#getters
	def get_image(self):
		return self.cv_image

if __name__ == '__main__':

	rospy.init_node('topic_publisher')
	PSM1 = robot('PSM1')
	PSM2 = robot('PSM2')
	ECM = robot('ECM')
	left_cam = camera('left')
	p = dvrk.psm('PSM2')

	image_path = '/home/fizzer/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04/Scripts/Images/'


	workbook = xlsxwriter.Workbook('arm_data_log.xlsx')
	worksheet = workbook.add_worksheet()

	#start from the first cell
	#write the column headers for PSM1
	worksheet.write(0, 0, 'Sequence')
	worksheet.write(0, 1, 'PSM1_joint_1')
	worksheet.write(0, 2, 'PSM1_joint_2')
	worksheet.write(0, 3, 'PSM1_joint_3')
	worksheet.write(0, 4, 'PSM1_joint_4')
	worksheet.write(0, 5, 'PSM1_joint_5')
	worksheet.write(0, 6, 'PSM1_joint_6')
	
	#write the column headers for PSM2
	worksheet.write(0, 7, 'PSM2_joint_1')
	worksheet.write(0, 8, 'PSM2_joint_2')
	worksheet.write(0, 9, 'PSM2_joint_3')
	worksheet.write(0, 10, 'PSM2_joint_4')
	worksheet.write(0, 11, 'PSM2_joint_5')
	worksheet.write(0, 12, 'PSM2_joint_6')

	#write the column headers for ECM (4 joints)
	worksheet.write(0, 13, 'ECM_joint_1')
	worksheet.write(0, 14, 'ECM_joint_2')
	worksheet.write(0, 15, 'ECM_joint_3')
	worksheet.write(0, 16, 'ECM_joint_4')
	worksheet.write(0, 17, 'ECM_joint_4')

	row = 1
	for i in range(1000):
		col = 0

		#get all the information to write
		Sequence = PSM1.get_seq()

		PSM1_jp = PSM1.get_current_joint_position()

		PSM2_jp = PSM2.get_current_joint_position()
		ECM_jp = ECM.get_current_joint_position()

		PSM2_jp = np.copy(PSM1_jp)
		p.move_jp(PSM2_jp)
		print(p.measured_jp())
		PSM2_jp = p.measured_jp()
		# left_cam_image = left_cam.get_image()
		# cv2.imwrite(image_path + "_" + str(i)+".png", left_cam_image)
		all_data = np.concatenate((PSM1_jp, PSM2_jp, ECM_jp), axis = 0)

		#write the values to an excel sheet

		worksheet.write(row, col, Sequence)

		for col in range(1, len(all_data) + 1 ):
			worksheet.write(row, col, all_data[col-1])

		row = row + 1

	for i in range(1, 1001):
		worksheet.insert_image(i, 17, image_path + "_" + str(i-1)+".png")

	workbook.close()
