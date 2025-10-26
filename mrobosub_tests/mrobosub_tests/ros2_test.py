# import threading

# import rclpy
# from mrobosub_lib import Node
# import time

# class RateTest(Node):
#     def __init__(self):
#         super().__init__("rate_test")
#         self.iteration_rate = 10
#         self.rate = self.create_rate(self.iteration_rate)
#         self.count = 0
#         self.start_time = time.time()

#     def event_loop(self):
#         while rclpy.ok():
#             rclpy.spin_once(self)
#             self.get_logger().info(f"Loop {self.count}")
#             self.count += 1
#             time.sleep(0.1)
#             elapsed = time.time() - self.start_time
#             self.get_logger().info(f"Time elapsed: {elapsed:.2f} seconds")
#             self.rate.sleep()

# def main():
#     rclpy.init()
#     node = RateTest()
#     node.event_loop()

# # class RateTest(Node):
# #     def __init__(self):
# #         super().__init__("rate_test")
# #         self.iteration_rate = 10
# #         self.rate = self.create_rate(self.iteration_rate)
# #         self.count = 0
# #         self.thread = threading.Thread(target=self.event_loop)
# #         self.thread.start()
# #         self.start_time = time.time()

# #     def event_loop(self):
# #         while rclpy.ok():
# #             self.get_logger().info(f"Loop {self.count}")
# #             self.count += 1
# #             time.sleep(0.1)
# #             elapsed = time.time() - self.start_time
# #             print(f"Time elapsed: {elapsed:.2f} seconds")
# #             self.rate.sleep()

# # def main():
# #     rclpy.init()
# #     node = RateTest()
# #     try:
# #         rclpy.spin(node)
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()

# if __name__ == "__main__":
#     main()