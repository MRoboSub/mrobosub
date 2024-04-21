#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from std_srvs.srv import SetBool, SetBoolRequest

class WebcamPub():
    def __init__(self) -> None:
        self.on = False

    def handle_on_service(self, req: SetBoolRequest):
        self.on = req.data

    def run(self):
        rospy.init_node('bot_pub_py', anonymous=True)

        pub = rospy.Publisher('bot_cam', Image, queue_size=10)
        rospy.Service('/bot_cam/on', SetBool, self.handle_on_service)

        # Create a VideoCapture object
        cap = cv2.VideoCapture("/dev/video10")
        # https://stackoverflow.com/a/66279297
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        cap.set(cv2.CAP_PROP_EXPOSURE, 100)
        #cap.set(cv2.CAP_PROP_FPS,10)
        #cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

        rate = rospy.Rate(10)

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
