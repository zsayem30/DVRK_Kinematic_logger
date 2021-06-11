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
import sys

if sys.version_info.major < 3:
    input = raw_input

class robot:

	def __init__(self, robot_name, ros_namespace = '/dvrk/'):
		"""Constructor. Initializes data members. It requires just a robot name. ex. r = robot('PSM1')"""
		self.__robot_name = robot_name
		self.__ros_namespace = ros_namespace
		self.__position_joint_current = []
		self.__position_jaw_current = []
		self.seq = 0
		self.rate = rospy.Rate(2)
		self.count = -1
		self.nsecs = 0
		self.secs = 0
		self.start_time = 0

		full_ros_namespace = self.__ros_namespace + self.__robot_name
		
		#publisher
		self.set_position_joint = rospy.Publisher(self.__robot_name + '/move_jp', JointState, queue_size = 1)
		self.set_position_jaw = rospy.Publisher(self.__robot_name + '/jaw/move_jp', JointState, queue_size = 1)
		#subscriber
		rospy.Subscriber(full_ros_namespace + '/state_jaw_current', JointState, self.current_jaw_state_callback, queue_size = 1, buff_size = 1)
		rospy.Subscriber(full_ros_namespace + '/state_joint_current', JointState, self.current_joint_state_callback, callback_args=(self.set_position_joint), queue_size = 1, buff_size = 1000000)

		#callback
	def current_joint_state_callback(self, data, (pub)):
		self.__position_joint_current[:] = data.position
		self.seq = data.header.seq
		self.nsecs = data.header.stamp.nsecs
		self.secs = data.header.stamp.secs
		self.start_time = 0
		joint_state = JointState()
		joint_state.position[:] = data.position
		pub.publish(joint_state)


	def current_jaw_state_callback(self, data):
		self.__position_jaw_current[:] = data.position

		#getters
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

	#movers
	def move_joint_direct(self, end_joint):

		joint_state = JointState()
		joint_state.position[:] = end_joint
		self.set_position_joint.publish(joint_state)

		return True

	def move_jaw_direct(self, end_jaw):
		joint_state = JointState()
		joint_state.position[:] = end_jaw
		self.set_position_jaw.publish(joint_state)
		
class camera:

	def __init__(self, camera_name, ros_namespace = '/stereo/'):

		self.__camera_name = camera_name
		self.__ros_namespace = ros_namespace
		self.bridge = CvBridge()
		self.cv_image = []
		self.image_count = 1
		self.image_path = '/home/fizzer/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04/Scripts/FK_Test/Images/' + self.__camera_name + '/'


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
			cv2.imwrite(self.image_path + self.__camera_name+"_Camera" +"_" + str(self.image_count)+".png", self.cv_image)
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
	psm1 = dvrk.psm('PSM1')
	psm2 = dvrk.psm('PSM2')
	ecm = dvrk.ecm('ECM')

	initial_joint_position_PSM1 = PSM1.get_current_joint_position()
	initial_joint_position_PSM2 = PSM2.get_current_joint_position()
	initial_joint_position_ECM = ECM.get_current_joint_position()

	psm1.move_jp(np.copy(initial_joint_position_PSM1))
	psm2.move_jp(np.copy(initial_joint_position_PSM2))
	ecm.move_jp(np.copy(initial_joint_position_ECM))

	input("		Press Enter to start logging...")

	# PSM1_jp = PSM1.get_current_joint_position()
	# PSM1_jaw_angle = PSM1.get_current_jaw_position()

	# PSM2_jp = PSM2.get_current_joint_position()
	# PSM2_jaw_angle = PSM2.get_current_jaw_position()

	# ECM_jp = ECM.get_current_joint_position()
	workbook = xlsxwriter.Workbook('dvrk_kinematic_logger_FK_test.xlsx')
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

		PSM1_jp = psm1.measured_jp()
		PSM2_jp = psm2.measured_jp()
		ECM_jp = ecm.measured_jp()

		PSM1_ee = [psm1.measured_cp().p.x(), psm1.measured_cp().p.y(), psm1.measured_cp().p.z()]
		PSM1_matrix = [psm1.measured_cp().M[0,0], psm1.measured_cp().M[0,1], psm1.measured_cp().M[0,2], psm1.measured_cp().M[1,0], psm1.measured_cp().M[1,1], psm1.measured_cp().M[1,2], psm1.measured_cp().M[2,0], psm1.measured_cp().M[2,1], psm1.measured_cp().M[2,2]]


		# PSM1_matrix_flatten = PSM1_matrix.shape()

		# print(PSM1_matrix_flatten)

		PSM2_ee = [psm2.measured_cp().p.x(), psm2.measured_cp().p.y(), psm2.measured_cp().p.z()]
		PSM2_matrix = [psm2.measured_cp().M[0,0], psm2.measured_cp().M[0,1], psm2.measured_cp().M[0,2], psm2.measured_cp().M[1,0], psm2.measured_cp().M[1,1], psm2.measured_cp().M[1,2], psm2.measured_cp().M[2,0], psm2.measured_cp().M[2,1], psm2.measured_cp().M[2,2]]
		
		ECM_ee = [ecm.measured_cp().p.x(), ecm.measured_cp().p.y(), ecm.measured_cp().p.z()]
		ECM_matrix = [ecm.measured_cp().M[0,0], ecm.measured_cp().M[0,1], ecm.measured_cp().M[0,2], ecm.measured_cp().M[1,0], ecm.measured_cp().M[1,1], ecm.measured_cp().M[1,2], ecm.measured_cp().M[2,0], ecm.measured_cp().M[2,1], ecm.measured_cp().M[2,2]]
		
		PSM1_jaw_angle = psm1.jaw.setpoint_js()[0]

		# PSM2_jp = PSM2.get_current_joint_position()
		PSM2_jaw_angle = psm2.jaw.setpoint_js()[0]

		# ECM_jp = ECM.get_current_joint_position()

		# state = PSM2.move_joint_direct(PSM1_jp_coppeliaSim)
		image_left = left_cam.get_image()
		image_right = right_cam.get_image()

		#write the data to the nodes for the psms
		

		# PSM2_jp_dvrk = p.measured_jp()
		# print(PSM2_jp_dvrk)

		# all_data = np.concatenate((PSM1_jp_coppeliaSim, PSM2_jp_dvrk, ECM_jp), axis = 0)

		PSM1_data = np.concatenate((PSM1_jp, PSM1_jaw_angle, PSM1_ee, PSM1_matrix), axis = 0)
		PSM2_data = np.concatenate((PSM2_jp, PSM2_jaw_angle, PSM2_ee, PSM2_matrix), axis = 0)
		ECM_data = np.concatenate((ECM_jp, ECM_ee, ECM_matrix), axis = 0)
		all_data = np.concatenate((PSM1_data, PSM2_data, ECM_data), axis = 0)

		time = data.header.stamp.secs + data.header.stamp.nsecs*10**(-9)

		if i != 0:

			# for col in range(1, len(PSM1_data) + 1 ):
			# 	worksheet.write(i, col, PSM1_data[col-1])

			# for col in range(11, len(PSM2_data) + 11):
			# 	worksheet.write(i, col, PSM2_data[col -11])

			# for col in range(21, len(ECM_jp) + 21):
			# 	worksheet.write(i, col, ECM_jp[col -21])
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