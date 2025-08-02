from enum import Enum
from dataclasses import dataclass, field
from struct import Struct
from typing_extensions import Union, Self
import sys
import warnings
from queue import SimpleQueue
from threading import Thread, Lock, Condition

import rospy
from mrobosub_msgs.msg import Dvl, MotorState, Imu_INS, Imu_PIMU
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from mrobosub_lib.lib import Node
from cv_bridge import CvBridge
import numpy as np

import net


class MessageKind(Enum):
    SENSORS = 1
    BOTCAM_IMAGE = 2
    ZED_IMAGE = 3
    MOTORS = 4
    BOTCAM_ON = 5
    ZED_ON = 6


@dataclass
class SensorData:
    depth: Float32
    dvl: Dvl
    imu_ins: Imu_INS
    imu_pimu: Imu_PIMU

    @staticmethod
    def unpack(data: bytes) -> "SensorData":
        FORMAT = Struct("! f 3f 3f 3f3ff")
        vals = FORMAT.unpack(data)
        return SensorData(
            depth=Float32(vals[0]),
            dvl=Dvl(vals[1], vals[2], vals[3]),
            imu_ins=Imu_INS(vals[4:6]),
            imu_pimu=Imu_PIMU(vals[6:9], vals[9:12], vals[12]),
        )

    @property
    def kind(self) -> MessageKind:
        return MessageKind.SENSORS


@dataclass
class ImageData:
    image: np.ndarray

    @classmethod
    def unpack(cls, data: bytes) -> Self:
        FORMAT = Struct("! 2LQ")
        PIXEL_DEPTH = 4  # RGBA
        (w, h, l) = FORMAT.unpack(data[: FORMAT.size])
        data = data[FORMAT.size :]
        assert w * h * PIXEL_DEPTH == l
        assert len(data) == l
        return cls(
            image=np.array(data).reshape((w, h, PIXEL_DEPTH)),
        )

    @property
    def encoding(self) -> str:
        return "bgra8"


@dataclass
class BotcamImage(ImageData):
    @property
    def kind(self) -> MessageKind:
        return MessageKind.BOTCAM_IMAGE


@dataclass
class ZedImage(ImageData):
    @property
    def kind(self) -> MessageKind:
        return MessageKind.ZED_IMAGE


@dataclass
class MotorData:
    state: MotorState

    def pack(self) -> bytes:
        FORMAT = Struct("! 8f")
        data = [getattr(self.state, f"motor{i}") for i in range(8)]
        return FORMAT.pack(*data)

    @property
    def kind(self) -> MessageKind:
        return MessageKind.MOTORS


@dataclass
class BotcamOnData:
    botcam_on: bool

    def pack(self) -> bytes:
        FORMAT = Struct("! b")
        return FORMAT.pack(self.botcam_on)

    @property
    def kind(self) -> MessageKind:
        return MessageKind.BOTCAM_ON


@dataclass
class ZedOnData:
    zed_on: bool

    def pack(self) -> bytes:
        FORMAT = Struct("! b")
        return FORMAT.pack(self.zed_on)

    @property
    def kind(self) -> MessageKind:
        return MessageKind.ZED_ON


MessageReceiveData = Union[SensorData, ImageData]
MessageSendData = Union[MotorData, BotcamOnData, ZedOnData]
MessageData = Union[MessageReceiveData, MessageSendData]


class SimDepth:
    def __init__(self) -> None:
        self.depth_pub = rospy.Publisher("/depth/raw_depth", Float32, queue_size=1)

    def handle_sensors(self, data: SensorData):
        self.depth_pub.publish(data.depth.value)


class SimDvl:
    def __init__(self) -> None:
        self.dvl_pub = rospy.Publisher("/dvl/raw_dvl", Dvl, queue_size=1)

    def handle_sensors(self, data: SensorData):
        self.dvl_pub.publish(data.dvl)


class SimImu:
    def __init__(self) -> None:
        self.ins_pub = rospy.Publisher("/imu_INS", Imu_INS, queue_size=1)
        self.pimu_pub = rospy.Publisher("/imu_PIMU", Imu_PIMU, queue_size=1)

    def handle_sensors(self, data: SensorData):
        self.ins_pub.publish(data.imu_ins)
        self.pimu_pub.publish(data.imu_pimu)


class SimBotcam:
    def __init__(self, hal: "SimHal") -> None:
        self.hal = hal
        self.botcam_pub = rospy.Publisher("/bot_cam", Image, queue_size=1)
        self.br = CvBridge()
        rospy.Service("/bot_cam/on", SetBool, self.handle_on_service)

    def handle_on_service(self, req: SetBoolRequest) -> SetBoolResponse:
        self.hal.send(BotcamOnData(req.data))
        return SetBoolResponse(success=True)

    def handle_images(self, data: BotcamImage):
        image = self.br.cv2_to_imgmsg(data.image, encoding="bgr8")
        self.botcam_pub.publish(image)


class SimZed:
    def __init__(self, hal: "SimHal") -> None:
        self.hal = hal
        self.zed_raw_pub = rospy.Publisher("/zed/raw", Image, queue_size=1)
        self.zed_crop_pub = rospy.Publisher(
            "/zed2/zed_node/rgb/image_rect_color", Image, queue_size=1
        )
        self.br = CvBridge()
        rospy.Service("/zed/on", SetBool, self.handle_on_service)

    def handle_on_service(self, req: SetBoolRequest) -> SetBoolResponse:
        self.hal.send(ZedOnData(req.data))
        return SetBoolResponse(success=True)

    def handle_images(self, data: ZedImage):
        self.zed_raw_pub.publish(
            self.br.cv2_to_imgmsg(data.image, encoding=data.encoding)
        )
        frame_chopped = self.chop(data.image)
        frame_cropped = self.crop(frame_chopped)
        img = self.br.cv2_to_imgmsg(frame_cropped, encoding=data.encoding)
        self.zed_crop_pub.publish(img)

    def chop(self, frame):
        width = frame.shape[1]
        return frame[:, : (width // 2), :]

    def crop(self, frame):
        left, right, top, bottom = 130, 50, 40, 60
        frame[:, :left] = frame[:, -right:] = frame[:top, :] = frame[-bottom:, :] = [
            255,
            0,
            0,
        ]
        return frame


class SimThrusterController:
    def __init__(self, hal: "SimHal") -> None:
        self.hal = hal
        self.motor_state_sub = rospy.Subscriber(
            "/motor_output", MotorState, self.callback
        )

    def callback(self, data: MotorState):
        self.hal.send(MotorData(data))


MSG_HEADER = Struct("!b")


class SimHal(Node):
    @dataclass
    class Client:
        client: net.socket
        queue: SimpleQueue = field(default_factory=SimpleQueue)
        thread: Thread = field(init=False)
        live: bool = True

        def __post_init__(self):
            self.thread = Thread(target=self.main)

        def main(self):
            while self.live:
                data = self.queue.get()
                self.live = net.send(self.client, data)

    def __init__(self, incoming_port: int, outgoing_port: int):
        super().__init__("sim_hal")
        self.clients: list[SimHal.Client] = []
        self.outgoing_server = net.Server(
            "0.0.0.0", outgoing_port, self.outgoing_callback
        )
        self.outgoing_server_thread = Thread(target=self.outgoing_server.run)
        self.incoming_server = net.Server(
            "0.0.0.0", incoming_port, self.incoming_callback
        )
        self.incoming_server_thread = Thread(target=self.incoming_server.run)

        self.depth = SimDepth()
        self.dvl = SimDvl()
        self.imu = SimImu()
        self.botcam = SimBotcam(self)
        self.zed = SimZed(self)
        self.thruster_controller = SimThrusterController(self)

    def send(self, data: MessageSendData):
        byte_data = MSG_HEADER.pack(data.kind.value) + data.pack()
        i = 0
        while i < len(self.clients):
            client = self.clients[i]
            if not client.live:
                client = self.clients.pop(i)
                client.thread.join()
                continue
            client.queue.put(byte_data)
            i += 1

    def outgoing_callback(self, client: net.socket):
        new_client = self.Client(client)
        new_client.thread.start()
        self.clients.append(new_client)

    def incoming_callback(self, client: net.socket):
        data = net.recv(client)
        if data is None:
            warnings.warn("Failed to fully receive message")
            return
        kind = MSG_HEADER.unpack(data[: MSG_HEADER.size])
        data = data[MSG_HEADER.size :]
        if kind == MessageKind.SENSORS:
            data = SensorData.unpack(data)
            self.depth.handle_sensors(data)
            self.dvl.handle_sensors(data)
            self.imu.handle_sensors(data)
        elif kind == MessageKind.BOTCAM_IMAGE:
            data = BotcamImage.unpack(data)
            self.botcam.handle_images(data)
        elif kind == MessageKind.ZED_IMAGE:
            data = ZedImage.unpack(data)
            self.zed.handle_images(data)
        else:
            warnings.warn(f"Unknown message {kind=}")

    def run(self):
        self.outgoing_server_thread.start()
        self.incoming_server_thread.start()
        rospy.spin()
        self.outgoing_server.stop()
        self.incoming_server.stop()
        self.outgoing_server_thread.join()
        self.incoming_server_thread.join()


if __name__ == "__main__":
    SimHal(int(sys.argv[1]), int(sys.argv[2])).run()
