import rospy
from std_msgs.msg import Float32
from state_machine import *


class ExampleContext(Context):
    def __init__(self):
        super().__init__('example')

        def input_cb(msg):
            self.input = msg.data
        rospy.Subscriber('/input', Float32, input_cb)

        self.output_pub = rospy.Publisher('/output', Float32, queue_size=1)


InitSuccessful = make_outcome('InitSuccessful')
FirstInputReceived = make_outcome('FirstInputReceived', val=0)
StartupTimedOut = make_outcome('StartupTimedOut')
WaitingForInput = make_outcome('WaitingForInput')


class Initialize(StateHandler):
    def iteration(self, gbl):
        return InitSuccessful()

class WaitForInput(StateHandler):
    def __init__(self, _prev_outcome, _gbl):
        self.begin_time = rospy.time()

    def iteration(self, gbl):
        if rospy.time() - self.begin_time > gbl.startup_wait_time:
            return StartupTimedOut()
        elif not gbl.input is None:
            return FirstInputReceived(val=gbl.input)
        else:
            return WaitingForInput()

