from socket import (
    SO_REUSEADDR,
    SOL_SOCKET,
    socket,
    AF_INET,
    SOCK_STREAM,
    timeout,
    gethostname,
)
from threading import Thread
import struct
from typing import Callable, Optional

FORMAT = ">Q"


def send(c: socket, data: bytes) -> bool:
    try:
        length = struct.pack(FORMAT, len(data))
        msg = length + data
        total_sent = 0
        while total_sent < len(msg):
            sent = c.send(msg[total_sent:])
            if sent == 0:
                return False
            total_sent += sent
        return True
    except ConnectionResetError as _:
        return False


def recv(c: socket) -> Optional[bytes]:
    try:
        MAX_CHUNK_SIZE = 2**15
        len_data = b""
        LEN_LEN = 8
        len_received = 0
        while len_received < LEN_LEN:
            chunk = c.recv(min(LEN_LEN - len_received, MAX_CHUNK_SIZE))
            if chunk == b"":
                return None
            len_data += chunk
            len_received += len(chunk)
        full_data_len: int
        (full_data_len,) = struct.unpack(FORMAT, len_data)
        chunks = []
        len_received = 0
        while len_received < full_data_len:
            chunk = c.recv(min(full_data_len - len_received, MAX_CHUNK_SIZE))
            if chunk == b"":
                return None
            chunks.append(chunk)
            len_received += len(chunk)
        return b"".join(chunks)
    except (ConnectionResetError, BrokenPipeError) as _:
        return None


def connect(host: str, port: int) -> socket:
    s = socket(AF_INET, SOCK_STREAM)
    s.connect((host, port))
    return s


class Server:
    def __init__(self, host: str, port: int, client_callback: Callable[[socket], None]):
        self.s = socket(AF_INET, SOCK_STREAM)
        self.s.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.s.settimeout(0.2)
        self.s.bind((host, port))
        self.s.listen(5)
        self.client_callback = client_callback
        self.shutdown = False

    def run(self):
        while not self.shutdown:
            try:
                (c, _) = self.s.accept()
                t = Thread(target=lambda: self.client_callback(c))
                t.run()
            except timeout:
                pass

    def stop(self):
        self.shutdown = True
