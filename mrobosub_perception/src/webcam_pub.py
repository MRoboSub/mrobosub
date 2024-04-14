#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from std_srvs.srv import SetBool, SetBoolRequest

class WebcamPub():
    on = False

    def handle_on_service(self, req: SetBoolRequest):
        self.on = req.data

    def run(self):
        rospy.init_node('bot_pub_py', anonymous=True)

        pub = rospy.Publisher('bot_cam', Image, queue_size=10)
        rospy.Service('/bot_cam/on', SetBool, self.handle_on_service)

        rate = rospy.Rate(10)

        cap = cv2.VideoCapture("/dev/botcam")
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        #cap.set(cv2.CAP_PROP_EXPOSURE, -8)

        br = CvBridge()

        while not rospy.is_shutdown():
            if self.on:
                ret, frame = cap.read()

                if ret == True:
                    pub.publish(br.cv2_to_imgmsg(frame, encoding='bgr8'))

            rate.sleep()

if __name__ == '__main__':
    try:
        WebcamPub().run()
    except rospy.ROSInterruptException as e:
        raise e
