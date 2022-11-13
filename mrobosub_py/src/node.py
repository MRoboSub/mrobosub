import rospy


class Node:
    def __init__(self, node: str, iteration_rate: int):
        rospy.init_node(node, anonymous=False)

        self.rate = rospy.Rate(iteration_rate)

    def run(self):
        while not rospy.is_shutdown():
            self()

            self.rate.sleep()

    def __call__():
        raise NotImplementedError
