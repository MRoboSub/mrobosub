import rclpy
import rclpy.node
from typing import Protocol, Any, TypeVar, NoReturn

class TimedResponseType(Protocol):
    """
    For typing, ensures response has the valid field
    """
    valid: bool

TimedResponse = TypeVar("TimedResponse", bound=TimedResponseType)

class TimedService():
    """
    Creates an abstraction of a service that once the service is called returns true
    for timing_threshold amount of time, allowing other work to be done, then returns false
    skipping doing additional work when it is not needed
    Ex. use case is as a check on whether or not to do processing for a subscriber callback
    """
    
    def __init__(self, node: rclpy.node.Node, service_name: str, ServiceType: Any, timing_threshold: float):
        """TimedService Constructor
        
        node -- the node which will own this service
        service_name -- the name of the service
        ServiceType -- the type that is used by the Service (ServiceTypeResponse is returned by the service)
        timing_threshold -- if the service has been called more recently than this timing threshold (s) then should_run is True
        """
        self.buffer = None
        self.node = node
        self.last_time = self.node.get_clock().now() # these times are in nanoseconds
        self.threshold = timing_threshold # threshold is in seconds
        self.srv = node.create_service(ServiceType, service_name, self._handle_obj_request)


    def _handle_obj_request(self, req: Any, res: TimedResponse) -> TimedResponse:
        """
        Returns the response in the buffer if it exists, if not returns response with valid field false
        """
        self.last_time = self.node.get_clock().get_time()
        if self.buffer == None:
            res.valid = False
            return res
        return self.buffer

    def set_result(self, result: TimedResponse) -> NoReturn:
       """
       This sets the most recent result that will be returned by the service when its called
       """
       self.buffer = result

    def should_run(self) -> bool:
        """
        Returns true if the service has been called recently
        """
        if((self.node.get_clock().get_time() - self.last_time)/10^9 > self.threshold):
            self.buffer = None
            return False
        else:
            return True