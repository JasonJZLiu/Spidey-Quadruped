"""
@author     Mayank Mittal
@email      mittalma@ethz.ch

@brief      Defines timer class for performance measurements..
"""

# python
import time
from typing import Any
from contextlib import ContextDecorator
# mpulator
from spidey_python.utils.errors import TimerError


class Timer(ContextDecorator):
    """
    A class to keep track of time for performance measurement.
    It allows timing via context managers and decorators as well.

    Reference: https://gist.github.com/sumeet/1123871
    """

    def __init__(self):
        """
        Initializes the class variables
        """
        self._start_time = None
        self._stop_time = None
        self._elapsed_time = None

    def __str__(self) -> str:
        """
        :return: String representation of the class object.
        """
        return "{:0.4f} seconds".format(self.time_elapsed)

    """
    Properties
    """

    @property
    def time_elapsed(self) -> float:
        """
        @note This is used for checking how much time has elapsed while the timer is
              still running.

        :return The number of seconds that have elapsed since this timer started timing.
        """
        return time.perf_counter() - self._start_time

    @property
    def total_run_time(self) -> float:
        """
        :return The number of seconds that elapsed from when the timer started to
                when it ended.
        """
        return self._elapsed_time

    """
    Operations
    """

    def start(self):
        """Start timing."""
        if self._start_time is not None:
            raise TimerError(f"Timer is running. Use .stop() to stop it")

        self._start_time = time.perf_counter()

    def stop(self):
        """Stop timing"""
        if self._start_time is None:
            raise TimerError(f"Timer is not running. Use .start() to start it")

        self._stop_time = time.perf_counter()
        self._elapsed_time = self._stop_time - self._start_time
        self._start_time = None

    """
    Context managers
    """

    def __enter__(self) -> "Timer":
        """Start timing and return this `Timer` instance."""
        self.start()
        return self

    def __exit__(self, *exc_info: Any):
        """Stop timing.
        """
        self.stop()

# EOF
