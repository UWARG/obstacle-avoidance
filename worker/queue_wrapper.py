"""
Worker Queue.
"""

import multiprocessing.managers
import time
import queue


class QueueWrapper:
    """
    Wrapper for an underlying queue proxy which also stores max size.
    """

    __QUEUE_TIMEOUT = 0.1  # seconds
    __QUEUE_DELAY = 0.1  # seconds

    def __init__(self, mp_manager: multiprocessing.managers.SyncManager, max_size: int = 0) -> None:
        self.queue = mp_manager.Queue(max_size)
        self.max_size = max_size

    def fill_queue_with_sentinel(self, timeout: float = 0.0) -> None:
        """
        Fills the queue with sentinel (None).
        """
        if timeout <= 0.0:
            timeout = self.__QUEUE_TIMEOUT

        try:
            for _ in range(0, self.max_size):
                self.queue.put(None, timeout=timeout)
        except queue.Full:
            return

    def drain_queue(self, timeout: float = 0.0) -> None:
        """
        Drains the queue.
        """
        if timeout <= 0.0:
            timeout = self.__QUEUE_TIMEOUT

        try:
            for _ in range(0, self.max_size):
                self.queue.get(timeout=timeout)
        except queue.Empty:
            return

    def fill_and_drain_queue(self) -> None:
        """
        Fills the queue with sentinel and then drains it.
        """
        self.fill_queue_with_sentinel()
        time.sleep(self.__QUEUE_DELAY)
        self.drain_queue()
