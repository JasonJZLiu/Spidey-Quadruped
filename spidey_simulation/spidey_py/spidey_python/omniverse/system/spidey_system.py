"""
@author     Jingzhou Liu
@email      jingzhou.liu@mail.utoronto.ca

@brief      Implementation of Spidey robot with the pydrake IK Solver.
"""

# python
import os
import numpy as np
from typing import Optional, Tuple, List, Dict, Union
import scipy.spatial.transform as tf
# omniverse
from pxr import Usd, UsdGeom, Gf
import omni.isaac.dynamic_control._dynamic_control as omni_dc

from spidey_python.utils.message import *
from spidey_python.omniverse.system.system_base import SystemBase
from spidey_python.omniverse.robot import SpideyRobot

from spidey_python.ik_solver import IKSolver


# Default dictionary for setting up the spidey system.
SPIDEY_SYSTEM_DEFAULT_CONFIG = {
    'robot_config': None
}


class SpideySystem(SystemBase):
    """
    @brief Implementation of Spidey system.

    This includes the IK-Solver and omniverse interfaces.

    """

    def __init__(self, stage: Usd.Stage, prim_path: Optional[str] = '/spidey', config: Optional[dict] = None,
                 meters_per_unit: Optional[float] = 1.0, dt: Optional[float] = 0.01):
        """
        Defines the variables and constants for the system.

        :param stage: The USD stage to import robot into.
        :param prim_path: The path for the primitive in the stage.
        :param config: A dictionary containing parameters for the class (default: SPIDEY_SYSTEM_DEFAULT_CONFIG).
        :param meters_per_unit: The units of conversion from simulator's scale to meters.
        :param dt: The timestep of simulation.
        """
        super().__init__()
        # copy args to internal variables
        self._dt = dt
        self._stage = stage
        # Update the default dictionary
        self._config = SPIDEY_SYSTEM_DEFAULT_CONFIG
        if config is not None:
            self._config.update(config)

        self._sim_time = 0.0

        # path to the assets directory
        usd_path = os.path.join(os.environ["SPIDEY_ROOT"], "omniverse/resources/usd", "robot/spidey/spidey.usd")
        # Add the USD path to robot config dictionary
        robot_config = {'usd_path': usd_path}
        if self._config["robot_config"] is not None:
            self._config["robot_config"].update(robot_config)
        else:
            self._config["robot_config"] = robot_config
        # Robot instance
        self._robot = SpideyRobot(stage, prim_path, config=self._config["robot_config"],
                                      meters_per_unit=meters_per_unit)

        spidey_path = os.path.join(os.environ["SPIDEY_ROOT"], "spidey_py/spidey_python/ik_solver/spidey_assets/urdf_and_meshes/spidey_v2.urdf")

        # Drake IK Solver instance
        self._spidey_IK_solver = IKSolver(urdf_file = spidey_path, model_name="spidey", root_link_name ="base_link", visualize=False)
        self._spidey_IK_solver.end_effector_frames = ["leg1_link3_tip","leg2_link3_tip","leg3_link3_tip","leg4_link3_tip"]
        self._prev_desired_cmd = {"desired_feet_poses": [], 
                                  "desired_base_link_pose": np.ndarray([])}

        #self._q_initial_guess: [q_w, q_x, q_y, q_z, x_root, y_root, z_root, joint_state_1 ... ]
        self._q_initial_guess = self._spidey_IK_solver.initial_q
        self._reference_spidey_joint_position = None

        self.spidey_joint_cmd = np.array([100]*12)

      

    def __del__(self):
        """
        Cleanup after usage.
        """
        pass

    def __str__(self) -> str:
        """
        :return: A string containing information about the instance's state.
        """
        return self._robot.__str__()

    """
    Properties
    """

    @property
    def robot(self) -> SpideyRobot:
        """
        :return: An instance of the robot interface class.
        """
        return self._robot


    """
    Operations
    """

    def create(self):
        """
        Loads the system into the Omniverse stage.
        """
        self._robot.create()

    def setup(self, dc: omni_dc.DynamicControl):
        """
        Registers the assets and configures internal variables of the system.

        :param dc: Handle to dynamic control plugin instance.
        """
        # robot
        self._robot.set_semantic_label("robot")
        self._robot.setup(dc)
        

    def reset(self, q: Optional[np.ndarray] = None, u: Optional[np.ndarray] = None):
        """
        Reset the robot's state, the internal timer.

        :param q: Generalized coordinates for the robot.
        :param u: Generalized velocities for the robot.
        """
        # check default arguments
        if q is None:
            q = self._robot.default_state["pos"]
        if u is None:
            u = self._robot.default_state["vel"]
        # set state of robot
        self._robot.set_state(q, u)
        # load initial state into buffers
        self._robot.update()
        # reset timer for steps
        self._sim_time = 0.0


    def advance(self, desired_cmd: Dict):
        """
        Apply input command to the robot.

        :param desired_cmd: A dictionary that contains the desired feet poses,  and desired base_link pose.
                            - desired_cmd["desired_feet_poses"] is a list of np.ndarrays that contains the poses of the desired feet poses.
                              [[x, y, z, q_x, q_y, q_z, q_w] ... ]
                            - desired_cmd["desired_base_link_pose"] is an np.ndarray that contains the desired base_link pose.
                              [x, y, z, roll, pitch, yaw]
        @note: It is assumed that only position information is provided in the desired poses; the orientation can be set to arbitrary values.
               If the orientations are also given, the IK solver (ik_solver.py) needs to include orientation constraints, which are not currently implemented.
        """
        new_feet_cmd_flag = True
        for i in range(len(self._prev_desired_cmd["desired_feet_poses"])):
            if np.all(desired_cmd["desired_feet_poses"][i]==self._prev_desired_cmd["desired_feet_poses"][i]):
                new_feet_cmd_flag = False
            else:
                new_feet_cmd_flag = True
                break
                
        if new_feet_cmd_flag or np.array_equal(desired_cmd["desired_base_link_pose"], self._prev_desired_cmd["desired_base_link_pose"]) == False:
            self._prev_desired_cmd = desired_cmd
            #self._reference_spidey_joint_position: np.array([12 joint angles])
            print(desired_cmd)
            self._reference_spidey_joint_position = self._spidey_IK_solver.get_ik_solution(desired_EE_poses = desired_cmd["desired_feet_poses"],
                                                                                  desired_root_pose = desired_cmd["desired_base_link_pose"],
                                                                                  q_initial_guess=None,
                                                                                  verbose=False)

        q_result = self._reference_spidey_joint_position
        spidey_joint_cmd = np.array([q_result[0], q_result[4], q_result[8],
                                     q_result[1], q_result[5], q_result[9],
                                     q_result[2], q_result[6], q_result[10],
                                     q_result[3], q_result[7], q_result[11]])

        self.spidey_joint_cmd = np.copy(spidey_joint_cmd)
        print("\nSPIDEY JOINT COMMAND: ", spidey_joint_cmd)
        # apply actions on robot
        self._robot.advance(spidey_joint_cmd)
        # update internal clock
        self._sim_time = self._sim_time + self._dt

    def update(self):
        """
        Updates the state buffers for the system.
        """
        # update robot state
        self._robot.update()

# EOF
