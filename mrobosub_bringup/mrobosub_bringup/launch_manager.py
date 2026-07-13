from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from multiprocessing import Process, Queue
import threading


class LaunchManager:
    """
    This class manages a launch service so that it can stop and start a launch file
    an arbitrary number of times, safely.

    This function takes in the path to the launch file and will start it on `start()`
    and stop it gracefully on `stop()`

    Args:
        launch_file_path (str): The path to the launch file to run.
    """

    def __init__(self, launch_file_path: str) -> None:
        self.launch_file_path = launch_file_path
        self.process: Process | None = None
        self.shutdown_queue: Queue = Queue()
    
    @staticmethod
    def run_launch_in_process(
        launch_file_path: str, shutdown_queue: Queue
    ) -> None:
        launch_service = LaunchService()
        launch_description = LaunchDescription(
            [IncludeLaunchDescription(AnyLaunchDescriptionSource(launch_file_path))]
        )
        launch_service.include_launch_description(launch_description)

        def check_for_shutdown() -> None:
            shutdown_queue.get()
            launch_service.shutdown()

        shutdown_checker = threading.Thread(target=check_for_shutdown, daemon=True)
        shutdown_checker.start()

        launch_service.run()
        # shutdown_checker.join()

    def start(self) -> None:
        # Don't "start" already running thread.
        if self.process is not None and self.process.is_alive():
            return

        self.process = Process(
            target=LaunchManager.run_launch_in_process,
            args=(self.launch_file_path, self.shutdown_queue),
        )

        self.process.start()

    def stop(self) -> None:
        if self.process is not None and self.process.is_alive():
            self.shutdown_queue.put(True)
            self.process.join()
