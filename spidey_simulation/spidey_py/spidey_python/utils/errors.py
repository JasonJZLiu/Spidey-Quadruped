"""
@author     Mayank Mittal
@email      mittalma@ethz.ch

@brief      Defines special execptions for mpulator-gym.
"""


class OmniverseError(Exception):
    """Exception raised for errors when running the omniverse simulator."""
    pass


class InvalidHandleError(Exception):
    """Exception raised for errors due to invalid handle discovered from omniverse."""
    pass


class TimerError(Exception):
    """A custom exception used to report errors in use of Timer class"""
    pass

# EOF
