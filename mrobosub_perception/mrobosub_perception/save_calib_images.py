#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import threading

class ImageSaverNode:
    def __init__(self):
        rospy.init_node('image_saver_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/bot_cam', Image, self.image_callback)
        self.image_count = 0
        self.save_directory = 'saved_images'
        self.save_flag = False

        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

        # Start save timer
        self.save_timer = threading.Timer(3, self.set_save_flag)
        self.save_timer.start()

    def image_callback(self, data):
        rospy.loginfo("Received new image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.save_flag:
                self.save_image(cv_image)
                self.save_flag = False
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    def set_save_flag(self):
        self.save_flag = True
        self.save_timer = threading.Timer(3, self.set_save_flag)
        self.save_timer.start()

    def save_image(self, image):
        filename = os.path.join(self.save_directory, f"image_{self.image_count}.jpg")
        cv2.imwrite(filename, image)
        rospy.loginfo("Image saved: %s", filename)
        self.image_count += 1

if __name__ == '__main__':
    try:
        image_saver_node = ImageSaverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

