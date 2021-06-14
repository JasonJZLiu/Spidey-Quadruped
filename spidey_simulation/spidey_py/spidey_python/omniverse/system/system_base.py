"""
@author     Mayank Mittal
@email      mittalma@ethz.ch

@brief      Base class for systems in Omniverse worflows.
"""
# python
import abc


class SystemBase(metaclass=abc.ABCMeta):
    """
    Abstract class for implementation of a system. A system is an integrated articulation
    instance which may comprise of interfaces for the robot and the sensors. Sometimes one
    may also want to combine a planners and mapping frameworks as a part of the system.

    The design philosophy is that a system is what one may expect an actual robot to be with
    the essentials for navigation and manipulation running on it.
    """

    """
    Instantiation
    """

    @abc.abstractmethod
    def __init__(self):
        pass

    @abc.abstractmethod
    def __del__(self):
        pass

    """
    Operations
    """

    @abc.abstractmethod
    def create(self, *args, **kwargs):
        """
        Loads the system into the Omniverse stage

        @note This function is kept separate in case one wants to create an instance of the class without launching
              the simulator. Or, if one doesn't want to create a new primitive programmatically but refer to an
              exisiting one in the current USD stage.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def setup(self, *args, **kwargs):
        """
        Registers the assets and configures internal variables of the system.
        """
        raise NotImplementedError

    def reset(self, *args, **kwargs):
        """
        Reset various interfaces in the system.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def advance(self, *args, **kwargs):
        """
        Apply input command to the robot.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def update(self):
        """
        Updates the buffers for dynamics state of the robot.
        """
        raise NotImplementedError

# EOF
