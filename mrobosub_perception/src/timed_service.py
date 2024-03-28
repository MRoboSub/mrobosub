#!/usr/bin/env python

import rospy

class TimedService():
    
    def __init__(self, service_name:str, ObjectType, timing_threshold:float):
        self.buffer = None
        self.last_time = rospy.get_time()
        self.objectType = ObjectType
        self.threshold = timing_threshold

        self.srv = rospy.ServiceProxy(service_name, ObjectType, self.handle_obj_request())
        


    def handle_obj_request(self):
        self.last_time = rospy.get_time()
        while self.buffer != None:
            rospy.sleep(.005)
        return self.buffer

    def set_result(self, result):
       self.buffer = result

    def should_run(self) -> bool:
        if((rospy.get_time() - self.last_time) > self.threshold):
            self.buffer = None
            return False
        else:
            return True
        