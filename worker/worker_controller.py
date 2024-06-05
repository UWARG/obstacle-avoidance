"""
To control multiproccesing workers.
"""

import multiprocessing as mp
import time


class WorkerController:
    """
    Allows main file to control multiprocessing workers via exit and pause requests.
    """

    __QUEUE_DELAY = 0.1

    def __init__(self) -> None:
        """
        Initializes semaphore and queue.
        """
        self.__process_limiter = mp.BoundedSemaphore(1)
        self.__is_paused = False
        self.__exit_queue = mp.Queue(1)

    def request_pause(self) -> None:
        """
        Request a worker to pause its process.
        """
        if not self.__is_paused:
            self.__process_limiter.acquire()
            self.__is_paused = True

    def request_resume(self) -> None:
        """
        Request a worker to resume its process.
        """
        if self.__is_paused:
            self.__process_limiter.release()
            self.__is_paused = False

    def check_pause(self) -> None:
        """
        If pause requested by main, worker is blocked, otherwise worker continues.
        """
        self.__process_limiter.acquire()
        self.__process_limiter.release()

    def request_exit(self) -> None:
        """
        Requests worker to exit its process.
        """
        time.sleep(self.__QUEUE_DELAY)
        if not self.__exit_queue.empty():
            self.__exit_queue.put(None)

    def clear_exit(self) -> None:
        """
        Clears exit request.
        """
        time.sleep(self.__QUEUE_DELAY)
        if not self.__exit_queue.empty():
            _ = self.__exit_queue.get()

    def is_exit_requested(self) -> bool:
        """
        Returns whether main has requested a worker to exit.
        """
        return not self.__exit_queue.empty()
