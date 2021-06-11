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
import arm
import camera

if sys.version_info.major < 3:
    input = raw_input

if __name__ == '__main__':

	rospy.init_node('topic_publisher')
	rate = rospy.Rate(140)

	PSM1 = arm.robot('PSM1')
	PSM2 = arm.robot('PSM2')
	ECM = arm.robot('ECM')
	left_cam = camera.camera('left')
	right_cam = camera.camera('right')
	
	p = dvrk.psm('PSM1')

	initial_joint_position = PSM1.get_current_joint_position()

	p.move_jp(np.copy(initial_joint_position))

	input("		Press Enter to start logging...")

	workbook = xlsxwriter.Workbook('arm_data_logger.xlsx')
	worksheet = workbook.add_worksheet()

	#start from the first cell
	#write the column headers for PSM1
	worksheet.write(0, 0, 'Sequence')
	worksheet.write(0, 1, 'PSM1_joint_1_dVRK')
	worksheet.write(0, 2, 'PSM1_joint_2_dVRK')
	worksheet.write(0, 3, 'PSM1_joint_3_dVRK')
	worksheet.write(0, 4, 'PSM1_joint_4_dVRK')
	worksheet.write(0, 5, 'PSM1_joint_5_dVRK')
	worksheet.write(0, 6, 'PSM1_joint_6_dVRK')
	
	#write the column headers for PSM2
	worksheet.write(0, 7, 'PSM1_joint_1_CoppeliaSim')
	worksheet.write(0, 8, 'PSM1_joint_2_CoppeliaSim')
	worksheet.write(0, 9, 'PSM1_joint_3_CoppeliaSim')
	worksheet.write(0, 10, 'PSM1_joint_4_CoppeliaSim')
	worksheet.write(0, 11, 'PSM1_joint_5_CoppeliaSim')
	worksheet.write(0, 12, 'PSM1_joint_6_CoppeliaSim')

	i = 1
	PSM1_jp_coppeliaSim_array = []
	PSM1_jp_dVRK_array = []
	def callback_dummy(data):
		global i, PSM1_jp_coppeliaSim_array, PSM1_jp_dVRK_array

		PSM1_jp_coppeliaSim = PSM1.get_current_joint_position()
		# print(PSM1_jp_coppeliaSim_array)
		# print("Next Array")
		PSM1_jp_coppeliaSim_array.append(PSM1_jp_coppeliaSim)

		ECM_jp = ECM.get_current_joint_position()
		image_left = left_cam.get_image()
		image_right = right_cam.get_image()

		#write the data to the nodes for the psms
		
		PSM1_jp_dvrk = p.measured_jp()
		PSM1_cp_dvrk = p.measured_cp()
		print(PSM1_cp_dvrk)
		PSM1_jp_dVRK_array.append(PSM1_jp_dvrk)

	i = i + 1

	print(PSM1_jp_coppeliaSim_array)
	rospy.Subscriber('/dvrk/PSM1/state_jaw_current', JointState, callback_dummy, queue_size = 1, buff_size = 1000000)

	try:
		rospy.spin()
	except rospy.ROSInterruptException as e:
		print("Error Running ROS." + e)
		pass
	print(i)
	PSM1_jp_dVRK_array = np.copy(PSM1_jp_dVRK_array)
	length = len(PSM1_jp_dVRK_array)

	PSM1_jp_dVRK_array = PSM1_jp_dVRK_array[1:, :]
	print(PSM1_jp_dVRK_array.shape)
	PSM1_jp_coppeliaSim_array = PSM1.get_full_array()
	PSM1_jp_coppeliaSim_array = np.copy(PSM1_jp_coppeliaSim_array)
	start = len(PSM1_jp_coppeliaSim_array) - length

	PSM1_jp_coppeliaSim_array = PSM1_jp_coppeliaSim_array[start:-1, :]
	print(PSM1_jp_coppeliaSim_array.shape)
	alldata = np.concatenate((PSM1_jp_dVRK_array, PSM1_jp_coppeliaSim_array), axis = 1)

	for i in range(1, len(alldata) + 1):
		for j in range(1, len(alldata[0]) + 1):
			worksheet.write(i, j, alldata[i-1,j-1])

	workbook.close()