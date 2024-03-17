#!/usr/bin/env python

import rospy

class TimedService():
    
    def __init__(self, serviceName, ObjectType, ResponseType, bufferSize, timingThreshold):
        srv = rospy.Service(serviceName, ObjectType, self.handle_obj_request())
        self.last_time = rospy.get_time()
        self.buffer_size =  bufferSize
        self.buffer = []
        self.object_type = ObjectType
        self.threshold = timingThreshold


    def handle_obj_request(self):
        self.last_time = rospy.get_time()
        if len(self.buffer) != 0:
           return self.buffer.pop(0)
        else:
            return self.object_type()

    def set_buffer(self, result):
        if len(self.buffer) < self.buffer_size:
            self.buffer.append(result)
        else:
            self.buffer.pop(0)
            self.buffer.append(result)

    def should_run(self) -> bool:
        if((rospy.get_time() - self.last_time) > self.threshold):
            self.buffer.clear()
            return False
        else:
            return True
        


    


