#!/usr/bin/env python

import rospy

class TimedService():
    
    def __init__(self, service_name:str, ObjectType, ResponseType, timing_threshold:float):
        srv = rospy.Service(service_name, ObjectType, self.handle_obj_request())
        self.last_time = rospy.get_time()
        self.buffer = None
        self.objectType = ObjectType
        self.threshold = timing_threshold


    def handle_obj_request(self):
        self.last_time = rospy.get_time()
        if self.buffer != None:
           return self.buffer
        else:
            return self.objectType()

    def set_result(self, result):
       buffer = result

    def should_run(self) -> bool:
        if((rospy.get_time() - self.last_time) > self.threshold):
            self.buffer = None
            return False
        else:
            return True
        


    


