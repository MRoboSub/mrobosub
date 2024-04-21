#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class WebcamPub():
    def __init__(self) -> None:
        self.on = False

    def handle_on_service(self, req: SetBoolRequest):
        if req.data == self.on: return SetBoolResponse(success=True)

        if req.data:
            self.open_capture()
        else:
            self.close_capture()
        self.on = req.data
        return SetBoolResponse(success=True)

    def open_capture(self):
        self.cap = cv2.VideoCapture(self.device_path)
        # https://stackoverflow.com/a/66279297
        #cap.set(cv2.CAP_PROP_FPS,10)
        #cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 100)

    def close_capture(self):
        self.cap.release()

    def run(self):
        rospy.init_node('bot_pub_py', anonymous=True)

        pub = rospy.Publisher('bot_cam', Image, queue_size=1)
        rospy.Service('/bot_cam/on', SetBool, self.handle_on_service)

        # Create a VideoCapture object
        self.device_path = "/dev/video10"

        rate = rospy.Rate(30)

        br = CvBridge()

        while not rospy.is_shutdown():
            if self.on:
                ret, frame = self.cap.read()

                if ret == True:
                    pub.publish(br.cv2_to_imgmsg(frame, encoding='bgr8'))

            rate.sleep()

if __name__ == '__main__':
    try:
        WebcamPub().run()
    except rospy.ROSInterruptException as e:
        raise e
