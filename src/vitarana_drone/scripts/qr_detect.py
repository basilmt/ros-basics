#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''


from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode,ZBarSymbol

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode 
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		self.barcode_data = String()
		self.barcode_data_pub = rospy.Publisher('/barcode',String,queue_size=1)


	# Callback function of camera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			# print("new")
			detectedBarcodes = decode(self.img, symbols = [ZBarSymbol.QRCODE])
			for barcode in detectedBarcodes:
				# type(barcode.data) => 'str'
				# we can sent it over std msgs 'String' and split at destination 
				print(barcode.data)
				self.barcode_data = barcode.data
				self.barcode_data_pub.publish(self.barcode_data)

				
				# map(float,barcode.data.split(",")) decode string to float[]


		except CvBridgeError as e:
			print(e)
			return



if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()