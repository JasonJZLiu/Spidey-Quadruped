"""
@author     Mayank Mittal
@email      mittalma@ethz.ch

@brief      Base class for robots in Omniverse worflows.
"""
# python
import abc
import numpy as np
from typing import Optional
# omniverse
from pxr import Usd


class RobotBase(metaclass=abc.ABCMeta):
    """
    Abstract class for implementation of a robot.
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

    @abc.abstractmethod
    def __str__(self) -> str:
        pass

    """
    Properties
    """

    @property
    def prim(self) -> Usd.Prim:
        """
        :return: The USD primitive instance corresponding to the robot.
        """
        raise NotImplementedError

    @property
    def prim_path(self) -> str:
        """
        :return: The path to the prim the stage.
        """
        raise NotImplementedError

    @property
    def dof_properties(self) -> dict:
        """
        :return: A dictionary containing the DOF properties such as joint limits.
        """
        raise NotImplementedError

    @property
    def q(self) -> np.ndarray:
        """
        :return: The generalized coordinates of the robot.
        """
        raise NotImplementedError

    @property
    def u(self) -> np.ndarray:
        """
        :return: The generalized velocities of the robot.
        """
        raise NotImplementedError

    """
    Helpers
    """

    def toggle_visibility(self, visible: bool):
        """ Toggle visibility of the robot prim in the scene.

        :param visible: Flag to whether make prim visible or invisible.
        """
        raise NotImplementedError

    def set_prim_pose(self, pos: np.ndarray, quat: Optional[np.ndarray] = None):
        """ Set location of the root of the robot in the stage.

        :param pos: (x, y, z) cartesian coordinates for location of root of the robot in the world frame.
        :param quat: (x, y, z, w) quaternion coordinates of orientation of root of the robot in the world frame.
                     Default orientation is (0, 0, 0, 1), i.e. identity w.r.t. world.
        """
        raise NotImplementedError

    def set_state(self, q: np.ndarray, u: np.ndarray, **kwargs):
        """ Set the dof state of the robot.

        :param q: Generalized coordinates for the robot.
        :param u: Generalized velocities for the robot.
        """
        raise NotImplementedError

    """
    Operations
    """

    @abc.abstractmethod
    def create(self, *args, **kwargs):
        """
        Loads the robot into the Omniverse stage

        @note This function is kept separate in case one wants to create an instance of the class without launching
              the simulator. Or, if one doesn't want to create a new primitive programmatically but refer to an
              exisiting one in the current USD stage.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def setup(self, *args, **kwargs):
        """
        Registers the assets and configures internal variables of the robot.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def advance(self, *args, **kwargs):
        """Apply input command to the robot.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def update(self):
        """
        Updates the buffers for dynamics state of the robot.
        """
        raise NotImplementedError

# EOF
