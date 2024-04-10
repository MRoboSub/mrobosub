#!/usr/bin/env python

import rospy

class TimedService():
    
    def __init__(self, service_name: str, ServiceType, timing_threshold: float):
        """TimedService Constructor
        
        service_name -- the name of the service
        ServiceType -- the type that is used by the Service (ServiceTypeResponse is returned by the service)
        timing_threshold -- if the service has been called more recently than this timing threshold then should_run is True
        """
        self.buffer = None
        self.last_time = rospy.get_time()
        self.threshold = timing_threshold
        self.srv = rospy.Service(service_name, ServiceType, self._handle_obj_request)


    def _handle_obj_request(self, _):
        self.last_time = rospy.get_time()
        while self.buffer == None:
            rospy.sleep(.005)
        return self.buffer

    def set_result(self, result):
       """This sets the most recent result that will be returned by the service when its called"""
       self.buffer = result

    def should_run(self) -> bool:
        """Returns true if the service has been called recently"""
        if((rospy.get_time() - self.last_time) > self.threshold):
            self.buffer = None
            return False
        else:
            return True
        
