from utils import ObjectPositionServiceMock, TargetReader, PoseMock
from std_msgs.msg import Float64, String
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse  # type: ignore
from time import sleep
import rospy

def main():
    rospy.init_node('test_sim')
    started = False
    sub = None
    def set_started(msg: String):
        nonlocal started, sub
        if started:
            return
        print(msg.data)
        started = True
        if sub is not None:
            sub.unregister()
        del sub
    sub = rospy.Subscriber('/captain/current_state', String, set_started)

    while not started:
        sleep(0.1)
    print('Started!')
    sleep(0.5)

    buoy_pos_mock = ObjectPositionServiceMock()
    reader = TargetReader()
    pose = PoseMock()

    pose.heave = 0.

    while pose.heave < reader.target_pose_heave:
        pose.heave += 0.005
        #print(f"{pose.heave=}")
        sleep(0.05)
        pose.publish_update()

    for i in range(20*3):
        sleep(0.05)

    buoy_pos_instance = ObjectPositionResponse()
    buoy_pos_instance.found = True
    buoy_pos_instance.x_theta = 20
    buoy_pos_instance.y_theta = 0

    buoy_pos_mock.set_position(buoy_pos_instance)


    for i in range(20*30):
        
        sleep(0.05)
        
        #print(f"{reader=}")
        print(f"{reader.target_twist_surge=}")
        print(f"{reader.target_twist_heave=}")
        print(f"{reader.target_pose_yaw=}")

    # bin_cam_instance.x *= -1
    # bin_cam_mock.set_position(bin_cam_instance)

    # for i in range(20*3):
    #     sleep(0.05)
    #     #print(f"{reader=}")
    #     print(f"{reader.target_pose_yaw=}")




if __name__ == '__main__':
    main()