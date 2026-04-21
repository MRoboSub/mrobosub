import socket
import rclpy
import numpy as np
from mrobosub_lib import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from mrobosub_msgs.msg import Dvl

UDP_IP =  "0.0.0.0"
HOST_IP = b"192.168.2.3"
UDP_PORT = 27000
POS_UP = True

ALPHA = 20 # deg from vertical

class DVLPublisher (Node):
    """
    Provides /dvl/raw_dvl topic
    """

    def __init__(self):
        super().__init__('dvl_publisher')

        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, "/dvl/twist", qos_profile=1)
        self.local_pose_pub = self.create_publisher(Dvl, "/dvl/local", qos_profile=1)
        self.T_beams_xyz = self.construct_transformation_matrix()

        # connect to socket containing the DVL information
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(True)  # False
        self.sock.settimeout(0.2)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.timer = self.create_timer(1.0/50, self.loop)

        # set the host address and port on the dvl
        message = b"HOST-ADDRESS %b:%b" % (HOST_IP, str(UDP_PORT).encode('utf-8'))
        self.sock.sendto(message, (UDP_IP, UDP_PORT))

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()

    def loop(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            data_str = data.decode()

            if data_str.startswith("$DVKFC"):
                self.publish_twist(data)         
            elif data_str.startswith("$DVPDL"):
                self.publish_local_pose(data)         

        except socket.timeout:
            self.get_logger().info("DVL UDP connection timing out, no data recieved from DVL")

    def publish_twist(self, data):
            # parse according to spec here https://docs.ceruleansonar.com/c/dvl-50/communicating-with-the-tracker-650/outgoing-message-formats-tracker-650-to-host/usddvkfc-kalman-filter-raw-data-support-message
            data_list = data.split(b",")
            va, vb, vc = map(float, data_list[10 : 25 : 7])

            # The DVL provides confidence values in each of the velocity axes
            # Take the biggest (worst) confidence value as variance.
            # Technically, values greater than 0.5 should be chucked out.
            va_c, vb_c, vc_c = map(float, data_list[11 : 26 : 7])
            max_var = max(va_c, vb_c, vc_c)
            if max_var > 0.5:
                return
            
            vx, vy, vz = self.calculate_robot_velocity_from_beams(va, vb, vc)
           
            # Create the message
            msg = TwistWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "dvl_link"

            # Write in the velocities
            msg.twist.twist.linear.x = vx
            msg.twist.twist.linear.y = vy
            msg.twist.twist.linear.z = vz

            if POS_UP:
                msg.twist.twist.linear.z *= 1 # Flip from down-positive to up-positive

            # The covariance matrix is a flattened 6x6 matrix. We only need to place the 0th, 7th, and 14th values.
            cov = [0.0] * 36
            cov[0]  = max_var
            cov[7]  = max_var
            cov[14] = max_var
            msg.twist.covariance = cov
            
            # Publish the msg
            self.twist_pub.publish(msg)
    
    def publish_local_pose(self, data):
            # parse according to spec here https://docs.ceruleansonar.com/c/tracker-650/communicating-with-the-tracker-650/outgoing-message-formats-tracker-650-to-host/usddvpdl-and-usddvpdx-dvl-position-and-angle-deltas-messages
            data_list = data.split(b",")
            # tu=1, dtu=2, adr=3, adp=4, ady=5, pdx=6, pdy=7, pdz=8, c=9
            try:
                confidence = float(data_list[9])
                if confidence <= 0: return # don't trust data with negative confidence

                local_msg = Dvl()
                local_msg.header.stamp = self.get_clock().now().to_msg()

                local_msg.ad[0] = float(data_list[3]) # roll
                local_msg.ad[1] = float(data_list[4]) # pitch
                local_msg.ad[2] = float(data_list[5]) # yaw

                local_msg.pd[0] = float(data_list[6]) # x
                local_msg.pd[1] = float(data_list[7]) # y
                local_msg.pd[2] = float(data_list[8]) # z
            
                if POS_UP:
                    local_msg.pd[2] *= -1 # Flip from down-positive to up-positive

                local_msg.confidence = confidence

                self.local_pose_pub.publish(local_msg) 
            except (IndexError, ValueError):
                 pass

    def calculate_robot_velocity_from_beams(self, va, vb, vc):
        '''
        Returns the velocity of the robot in the Forward-Left-Down frame from the respective beam velocities 
        '''
        beams = np.array([va, vb, vc])
        v_robot = self.T_beams_xyz @ beams
        return v_robot

    def construct_transformation_matrix(self):
        # Since the Tracker 650 is a 3-beam DVL, this means that each beam is aligned at a 120deg offset
        # from each other. Additionally, there is a 70def beam angle from the horizontal
        # We need to apply a transformation matrix to the beam measurements to recover the robot frame velocity
        alpha = np.radians(ALPHA)

        s_a = np.sin(alpha)
        c_a = np.cos(alpha)

        T_beam_xyz = np.array([
            [(2/3)/s_a,         (-1/3)/s_a,          (-1/3)/s_a], # Vx (Forward)
            [        0, (1/np.sqrt(3))/s_a, (-1/np.sqrt(3))/s_a], # Vy (Left)
            [(1/3)/c_a,          (1/3)/c_a,           (1/3)/c_a], # Vz (Down-positive)
        ])

        return T_beam_xyz

def main():
    rclpy.init()
    node = DVLPublisher()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
