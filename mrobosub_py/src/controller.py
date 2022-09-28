from types import Any, FunctionType, List, Optional, TypedDict

import rospy


class RosSubscriber(TypedDict):
    topic: str
    data_type: Any
    callback: FunctionType


class RosPublisher(TypedDict):
    topic: str
    data_type: Any
    name: str


class Controller(Node):
    def __init__(
        self,
        node: str,
        iteration_rate: int,
        subscribers: List[RosSubscriber],
        publishers: List[RosPublisher],
    ):
        super().__init__(node, iteration_rate)

        subscribe(subscribers)
        publish(publishers)

    def subscribe(self, subscribers: List[RosTopic]):
        for subscriber in subscribers:
            rospy.Subscriber(
                subscriber["topic"], subscriber["data_type"], subscriber["callback"]
            )

    def publish(self, publishers: List[RosTopic]):
        self.publishers = {}
        for publisher in publishers:
            publishers[publisher["name"]] = rospy.Publisher(
                publisher["topic"], publisher["data_type"]
            )
