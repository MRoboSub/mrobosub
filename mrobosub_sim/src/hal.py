#!/usr/bin/env python

from enum import Enum
from dataclasses import dataclass, field
from struct import Struct
from typing_extensions import Union, Self, List
import sys
import warnings
from queue import Empty, SimpleQueue
from threading import Thread

import rospy
from mrobosub_lib.lib import Node
from cv_bridge import CvBridge
import cv2
import numpy as np

from mrobosub_msgs.msg import Dvl, MotorState, Imu_INS, Imu_PIMU, Detections, Detection
from mrobosub_msgs.srv import ObjectPosition, ObjectPositionResponse
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

import net


class MessageKind(Enum):
    SENSORS = 1
    BOTCAM_IMAGE = 2
    ZED_IMAGE = 3
    ML_TARGET = 4
    MOTORS = 5
    BOTCAM_ON = 6
    ZED_ON = 7


class Targets(Enum):
    GATE_RED = 0
    GATE_BLUE = 1


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
            imu_ins=Imu_INS(vals[4:7]),
            imu_pimu=Imu_PIMU(vals[6:9], vals[9:12], vals[12]),
        )

    @property
    def kind(self) -> MessageKind:
        return MessageKind.SENSORS


@dataclass
class ImageData:
    time: float
    image: np.ndarray

    @classmethod
    def unpack(cls, data: bytes) -> Self:
        FORMAT = Struct("! d 2L Q")
        PIXEL_DEPTH = 4  # BGRA
        (time, w, h, l) = FORMAT.unpack(data[: FORMAT.size])
        data = data[FORMAT.size :]
        assert (
            w * h * PIXEL_DEPTH == l
        ), f"width={w}, height={h}, len={l} (PIXEL_DEPTH={PIXEL_DEPTH})"
        assert len(data) == l
        buf = np.frombuffer(data, count=l, dtype=np.uint8).reshape((h, w, PIXEL_DEPTH))
        return cls(
            time=time,
            image=buf,
        )

    @property
    def encoding(self) -> str:
        return "rgba8"


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


ML_TARGET_FORMAT = Struct("! b 4f")


@dataclass
class MLTargetData:
    target_kind: Targets
    left: float
    top: float
    right: float
    bottom: float

    @classmethod
    def unpack(cls, data: bytes) -> Self:
        kind, tlx, tly, brx, bry = ML_TARGET_FORMAT.unpack(data)
        return cls(Targets(kind), tlx, tly, brx, bry)

    @property
    def width(self) -> float:
        return self.right - self.left

    @property
    def height(self) -> float:
        return self.bottom - self.top

    @property
    def x_position(self) -> float:
        return (self.right + self.left) / 2

    @property
    def y_position(self) -> float:
        return (self.bottom + self.top) / 2


@dataclass
class MLTargetsData:
    targets: List[MLTargetData]
    width: float
    height: float

    @classmethod
    def unpack(cls, data: bytes) -> Self:
        FORMAT = Struct("! b f f")
        count, width, height = FORMAT.unpack(data[: FORMAT.size])
        data = data[FORMAT.size :]
        targets = []
        for _ in range(count):
            targets.append(MLTargetData.unpack(data[: ML_TARGET_FORMAT.size]))
            data = data[ML_TARGET_FORMAT.size :]
        return cls(targets, width, height)

    @property
    def kind(self) -> MessageKind:
        return MessageKind.ML_TARGET


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


MessageReceiveData = Union[SensorData, ZedImage, BotcamImage, MLTargetData]
MessageSendData = Union[MotorData, BotcamOnData, ZedOnData]
MessageData = Union[MessageReceiveData, MessageSendData]


class SimDepth:
    def __init__(self) -> None:
        self.depth_pub = rospy.Publisher("/depth/raw_depth", Float32, queue_size=1)

    def handle_sensors(self, data: SensorData):
        self.depth_pub.publish(data.depth)


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
        self.last_image_time = 0
        rospy.Service("/bot_cam/on", SetBool, self.handle_on_service)

    def handle_on_service(self, req: SetBoolRequest) -> SetBoolResponse:
        self.hal.send(BotcamOnData(req.data))
        return SetBoolResponse(success=True)

    def handle_images(self, data: BotcamImage):
        if data.time < self.last_image_time:
            return
        self.last_image_time = data.time
        image = self.br.cv2_to_imgmsg(data.image, encoding=data.encoding)
        self.botcam_pub.publish(image)


class SimZed:
    def __init__(self, hal: "SimHal") -> None:
        self.hal = hal
        self.zed_raw_pub = rospy.Publisher("/zed/raw", Image, queue_size=1)
        self.zed_crop_pub = rospy.Publisher(
            "/zed2/zed_node/rgb/image_rect_color", Image, queue_size=1
        )
        self.br = CvBridge()
        self.last_image_time = 0
        rospy.Service("/zed/on", SetBool, self.handle_on_service)

    def handle_on_service(self, req: SetBoolRequest) -> SetBoolResponse:
        self.hal.send(ZedOnData(req.data))
        return SetBoolResponse(success=True)

    def handle_images(self, data: ZedImage):
        if data.time < self.last_image_time:
            return
        self.last_image_time = data.time
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
        frame = np.copy(frame)
        left, right, top, bottom = 130, 50, 40, 60
        frame[:, :left] = frame[:, -right:] = frame[:top, :] = frame[-bottom:, :] = [
            0,
            0,
            255,
            255,
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


class SimML:
    def __init__(self) -> None:
        self.detections_pub = rospy.Publisher(
            "/ml/detections", Detections, queue_size=1
        )

    def handle_targets(self, data: MLTargetsData):
        message = Detections(
            detections=tuple(
                Detection(
                    left=target.left,
                    top=target.top,
                    right=target.right,
                    bottom=target.bottom,
                    confidence=1.0,
                    classification=target.target_kind.value,
                )
                for target in data.targets
            ),
            width=data.width,
            height=data.height,
        )
        self.detections_pub.publish(message)


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
                try:
                    data = self.queue.get(timeout=0.5)
                except Empty:
                    continue
                self.live = self.live and net.send(self.client, data)

        def stop(self):
            self.live = False

    def __init__(self, incoming_port: int, outgoing_port: int):
        super().__init__("sim_hal")
        self.clients: List[SimHal.Client] = []
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
        self.ml = SimML()

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
        (kind,) = MSG_HEADER.unpack(data[: MSG_HEADER.size])
        data = data[MSG_HEADER.size :]
        if kind == MessageKind.SENSORS.value:
            data = SensorData.unpack(data)
            self.depth.handle_sensors(data)
            self.dvl.handle_sensors(data)
            self.imu.handle_sensors(data)
        elif kind == MessageKind.BOTCAM_IMAGE.value:
            data = BotcamImage.unpack(data)
            self.botcam.handle_images(data)
        elif kind == MessageKind.ZED_IMAGE.value:
            data = ZedImage.unpack(data)
            self.zed.handle_images(data)
        elif kind == MessageKind.ML_TARGET.value:
            data = MLTargetsData.unpack(data)
            self.ml.handle_targets(data)
        else:
            warnings.warn(f"Unknown message {kind=}")

    def run(self):
        self.outgoing_server_thread.start()
        self.incoming_server_thread.start()
        rospy.spin()
        self.outgoing_server.stop()
        self.incoming_server.stop()
        for client in self.clients:
            client.stop()
        self.outgoing_server_thread.join()
        self.incoming_server_thread.join()
        for client in self.clients:
            client.thread.join()


if __name__ == "__main__":
    SimHal(int(sys.argv[1]), int(sys.argv[2])).run()
