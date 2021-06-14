"""
@author     Jingzhou Liu
@email      jingzhou.liu@mail.utoronto.ca
@brief      Defines class related to Spidey.
"""
# python
import numpy as np
import scipy.spatial.transform as tf
from typing import Optional, Union
# omniverse
import omni.isaac.dynamic_control._dynamic_control as omni_dc
# spidey
from spidey_python.utils.message import *
from spidey_python.utils.errors import *
from spidey_python.omniverse.robot.spidey.types import SpideyDim


class SpideyBot:
    """
    @brief A thin wrapper around the articulation handle to set, apply and retrieve data related to Spidey.
    @note This file does not create the articulation handle. Rather it expects the handle to be passed during the setup.
    Spidey is a 12-DoF quadraped. In this wrapper, the following controllers are implemented:
        * Position Controller (dims: 16): Inputs are the desired joint positions.
        * Joint Velocity Controller (dims: 16): Inputs are the desired joint velocities. Disable gravity for the links of the
                                    robot to account for gravity compnsation.
        * Effort Controller (dims: 16): Directly set the torque at each joint.
    The class also provides access to various dynamic state of the robot such as the joint positions and velocities, along
    with the end-effector state in the world frame.
    """

    def __init__(self, meters_per_unit: float, ee_handle_names: Optional[list] = None,
                 transform_ext: Union[list, np.ndarray]=None):
        """
        Defines variables for the hand of the robot.
        :param meters_per_unit: Conversion of units in simulator into meters.
        :param ee_handle_names: A list of names of the end-effector links in the robot.
        :param transform_ext: External transform, [x, y, z, quat_x, quat_y, quat_z, quat_w], from EE-handle to
                              another frame (default: None).
        @note The external transform is useful in cases when there is no actual end-effector frame  in the prim.
              Instead one can specify a relative transform from an existing frame to the desired end-effector
              frame indirectly. However, this only works in getting the right end-effector pose. It has not been
              implemented to get higher order terms like velocity.
        """
        # Copy inputs into class variables
        self._meters_per_unit = meters_per_unit
        self._ee_handle_names = ee_handle_names
        self._transform_ext = transform_ext
        # Definitions for internal dimensions
        self._dims = SpideyDim
        self._dof_offset = 0
        # Definition for control type of the hand
        self._ctrl_type = None
        # Handles to various ov-kit plugins
        self._dc_handle = None
        # Handles related to robot
        self._articulation_handle = None
        self._ee_handles = []
        self._base_link_handle = None

        # Joint names for components of the robot
        self._spidey_joint_names = ["leg1_joint1", "leg1_joint2","leg1_joint3",
                                  "leg2_joint1", "leg2_joint2","leg2_joint3", 
                                  "leg3_joint1","leg3_joint2","leg3_joint3",
                                  "leg4_joint1","leg4_joint2","leg4_joint3",]
        # Joint names to handles mapping
        self._spidey_joint_names_to_handle_mapping = dict()
        # Define hand's default state
        self._spidey_default_state = {
            "pos": np.zeros(self._dims.SpideyJointsDim.value),
            "vel": np.zeros(self._dims.SpideyJointsDim.value)
        }
        # Dynamics information of the robot
        self._spidey_state = {
            "pos": np.zeros(self._dims.SpideyJointsDim.value),
            "vel": np.zeros(self._dims.SpideyJointsDim.value)
        }

        self.I_r_IE = []
        self.q_IE = []
        self.C_IE = []
        self.I_v_IE = []
        self.I_omega_IE = []

        for i in range (len(self._ee_handle_names)):
            # The absolute pose of the end effector expressed in the world frame
            self.I_r_IE.append(np.zeros(3))
            self.q_IE.append(np.array([0.0, 0.0, 0.0, 1.0])) #xyzw
            self.C_IE.append(np.eye(3, 3))
            # The absolute velocity of the end-effector expressed wrt world frame
            self.I_v_IE.append(np.zeros(3))
            self.I_omega_IE.append(np.zeros(3))

    def __str__(self) -> str:
        """
        :return: A string containing information about the instance's state.
        """
        msg = "Spidey Joint State:\n" \
              f"  q_j: {self.state['pos']} \n" \
              f"  dq_j: {self.state['vel']} \n" \
              "Spidey Feet Poses:\n" \
              f"    I_r_IE: {self.I_r_IE} \n" \
              f"    q_IE: {self.q_IE} \n" \
              f"    I_v_IE: {self.I_v_IE} \n" \
              f"    I_omega_IE: {self.I_omega_IE} \n"

        return msg

    """
    Configurations
    """

    def set_state(self, q: np.ndarray, u: np.ndarray):
        """ Set the dof state of the robot.
        :param q: Joint coordinates of the robot.
        :param u: Joint velocities for the robot.
        """
        # convert input to numpy array (sanity)
        q = np.asarray(q)
        u = np.asarray(u)
        # check input is of right shape
        assert q.shape == (self._dims.SpideyJointsDim.value,)
        assert u.shape == (self._dims.SpideyJointsDim.value,)
        # for spidey
        robot_dof_states = self._dc_handle.get_articulation_dof_states(self._articulation_handle, omni_dc.STATE_ALL)
        for index in range(len(self._spidey_joint_names)):
            # set initial joint stat
            print ("q is ", q[index])
            print ("u is ", u[index])
            robot_dof_states["pos"][self._dof_offset + index] = q[index]
            robot_dof_states["vel"][self._dof_offset + index] = u[index]
        self._dc_handle.set_articulation_dof_states(self._articulation_handle, robot_dof_states, omni_dc.STATE_ALL)

    """
    Properties
    """

    @property
    def joint_names_to_handle_mapping(self) -> dict:
        """
        :return: A mapping from joint names to the simulation handles.
        """
        return self._spidey_joint_names_to_handle_mapping

    @property
    def dof(self) -> int:
        """
        :return: The degrees of freedom for the system.
        """
        return len(self._spidey_joint_names)

    @property
    def default_state(self) -> dict:
        """
        :return: The default state ("pos": q_j, "vel": u_j) of the robot.
        """
        return self._spidey_default_state

    @property
    def state(self) -> dict:
        """
        :return: The current state ("pos": q_j, "vel": u_j) of the robot.
        """
        return self._spidey_state

    @property
    def base_link_pose(self) -> np.ndarray:
        """
        :return: The current pose of the base_link: [q_w, q_x, q_y, q_z, x_root, y_root, z_root]
        """
        pose = self._dc_handle.get_rigid_body_pose(self._base_link_handle)
        return np.array([pose.r.w, pose.r.x, pose.r.y, pose.r.z, pose.p.x*self._meters_per_unit, pose.p.y*self._meters_per_unit, pose.p.z*self._meters_per_unit])
        
    """
    Operations
    """

    def setup(self, articulation_handle, dc, ctrl: str,
              dof_offset: Optional[int] = 0,
              disable_gravity: Optional[bool] = True):
        """
        Configures internal variables of spidey.
        :param articulation_handle: Handle to the articulation system in the scene.
        :param dc: Handle to dynamic control plugin instance.
        :param ctrl: Type of low-level control for the robot, from ['position', 'velocity', 'effort'].
        :param dof_offset: The degree-of-freedom preceding the robot part's joints in the chain. This number needs to
                           be set manually. To find the information, print the robot's DOF state information.
        :param disable_gravity: Whether to apply gravity forces on the links or not.
        """
        assert ctrl in ['position', 'velocity', 'effort']
        # initialize internal variables
        self._articulation_handle = articulation_handle
        self._dc_handle = dc
        self._ctrl_type = ctrl
        self._dof_offset = dof_offset
        # setup handles for the robot
        self._setup_handles()
        # setup links of the robot
        self._setup_links(disable_gravity)
        # setup controls for the robot
        self._setup_control(ctrl)

    def apply_command(self, cmd: np.ndarray):
        """
        Applies low-level command for spidey.
        :param cmd: The command for the robot.
                    - For a position controller, these are the desired joint positions.
                    - For a velocity controller, these are the desired joint velocities.
                    - For an efforts contrller, these are the joint torques applied.
        """
        # convert input to numpy array (sanity)
        spidey_joint_cmd = np.asarray(cmd)
        # check input is of correct dimensions
        assert spidey_joint_cmd.shape == (self._dims.SpideyInputDim.value,)
        # apply input for spidey
        for index, dof_name in enumerate(self._spidey_joint_names):
            if self._ctrl_type is 'velocity':
                self._dc_handle.set_dof_velocity_target(self._spidey_joint_names_to_handle_mapping[dof_name],
                                                        float(spidey_joint_cmd[index]))
            elif self._ctrl_type is 'effort':
                self._dc_handle.apply_dof_effort(self._spidey_joint_names_to_handle_mapping[dof_name],
                                                 float(spidey_joint_cmd[index]))
            elif self._ctrl_type is 'position':
                self._dc_handle.set_dof_position_target(self._spidey_joint_names_to_handle_mapping[dof_name],
                                                 float(spidey_joint_cmd[index]))

    def update(self):
        """
        Updates the buffers for dynamics state of the robot.
        """
        # fill spidey's joint states to generalized state
        jnt_states = self._dc_handle.get_articulation_dof_states(self._articulation_handle,
                                                                 omni_dc.STATE_ALL)
        for index in range(len(self._spidey_joint_names)):
            dof_handle = self._dc_handle.find_articulation_dof(self._articulation_handle, self._spidey_joint_names[index])
            self._spidey_state["pos"][index] = self._dc_handle.get_dof_position(dof_handle)
            self._spidey_state["vel"][index] = self._dc_handle.get_dof_velocity(dof_handle)

        # fill end-effector state w.r.t. world
        # pose of the end-effectors of the robot
        for i in range (len(self._ee_handle_names)):
            ee_pose = self._dc_handle.get_rigid_body_pose(self._ee_handles[i])
            self.I_r_IE[i] = np.array([ee_pose.p.x, ee_pose.p.y, ee_pose.p.z])
            self.q_IE[i] = np.array([ee_pose.r.x, ee_pose.r.y, ee_pose.r.z, ee_pose.r.w])
            self.C_IE[i] = tf.Rotation.from_quat(self.q_IE[i]).as_matrix()
            # velocity of the end-effector of the robot
            linear_vel = self._dc_handle.get_rigid_body_linear_velocity(self._ee_handles[i])
            angular_vel = self._dc_handle.get_rigid_body_angular_velocity(self._ee_handles[i])
            self.I_v_IE[i] = np.array([linear_vel.x, linear_vel.y, linear_vel.z])
            self.I_omega_IE[i] = np.array([angular_vel.x, angular_vel.y, angular_vel.z])
            # convert from simulator's units to meters
            self.I_r_IE[i] = self.I_r_IE[i] * self._meters_per_unit
            self.I_v_IE[i] = self.I_v_IE[i] * self._meters_per_unit
        # convert to external transform if valid
        if self._transform_ext:
            # local transform
            E_r_EX = self._transform_ext[:3]
            q_EX = tf.Rotation.from_quat(self._transform_ext[3:]).as_quat()
            C_EX = tf.Rotation.from_quat(q_EX).as_matrix()
            # world-> X transform
            for i in range (len(self._ee_handle_names)):
                self.C_IE[i] = np.matmul(self.C_IE[i], C_EX)
                self.q_IE[i] = tf.Rotation.from_matrix(self.C_IE[i]).as_quat()
                self.I_r_IE[i] = self.I_r_IE[i] + np.dot(self.C_IE[i], E_r_EX)

        
    """
    Internals
    """

    def _setup_handles(self):
        """
        Configures the handles of the joints and their mapping.
        """
        # setup end-effector handle
        for i in range (len(self._ee_handle_names)):
            self._ee_handles.append(self._dc_handle.find_articulation_body(self._articulation_handle, self._ee_handle_names[i]))
            # check handles are valid
            if self._ee_handles[i] == omni_dc.INVALID_HANDLE:
                msg = f"*** Failed to load handle at \'{self._ee_handle_names[i]}\'"
                print_error(msg)
                raise InvalidHandleError(msg)

        #setup palm_link handle
        self._base_link_handle = self._dc_handle.find_articulation_body(self._articulation_handle, "base_link")
        if self._ee_handles[i] == omni_dc.INVALID_HANDLE:
            msg = f"*** Failed to load handle at base_link"
            print_error(msg)
            raise InvalidHandleError(msg)

        # setup handles for spidey joints
        for dof_name in self._spidey_joint_names:
            # find handle to dof
            dof_handle = self._dc_handle.find_articulation_dof(self._articulation_handle, dof_name)
            # complain if handle is invalid
            if dof_handle == omni_dc.INVALID_HANDLE:
                msg = f"*** Failed to load handle at \'{dof_name}\'"
                print_error(msg)
                raise InvalidHandleError(msg)
            # add to mapping
            self._spidey_joint_names_to_handle_mapping[dof_name] = dof_handle

    def _setup_links(self, disable_gravity: bool):
        """
        Configures the properties of the links in the robot.
        :param disable_gravity: Whether to apply gravity forces on the links or not.
        """
        # names of the links of the robot
        spidey_link_names = ["base_link", "leg1_link1", "leg1_link2", "leg1_link3","leg1_link3_tip",
                           "leg2_link1", "leg2_link2", "leg2_link3","leg2_link3_tip",
                           "leg3_link1", "leg3_link2", "leg3_link3","leg3_link3_tip",
                           "leg4_link1", "leg4_link2", "leg4_link3","leg4_link3_tip"]

        for link_name in spidey_link_names:
            # get link index
            link_handle = self._dc_handle.find_articulation_body(self._articulation_handle, link_name)
            # complain if handle is invalid
            if link_handle == omni_dc.INVALID_HANDLE:
                msg = f"*** Failed to load handle at \'{link_name}\'"
                print_error(msg)
                raise InvalidHandleError(msg)
            # disable gravity for link
            self._dc_handle.set_rigid_body_disable_gravity(link_handle, disable_gravity)

    def _setup_control(self, ctrl: str):
        """
        Configures the controllers for the robot system.
        :param ctrl: The type of control for spidey's joints. Supports: ['position', 'velocity', 'effort']
        """
        # get joint type efforts
        robot_dof_props = self._dc_handle.get_articulation_dof_properties(self._articulation_handle)
        damping_scaling = 1.0e5
        # hand joints: set control type based on specification
        for index in range(len(self._spidey_joint_names)):
            if ctrl == 'velocity':
                # set drive mode
                robot_dof_props["driveMode"][self._dof_offset + index] = omni_dc.DRIVE_VEL
                robot_dof_props["stiffness"][self._dof_offset + index] = 1.0
                robot_dof_props["damping"][self._dof_offset + index] = 1.0e3 * damping_scaling
            elif ctrl == 'effort':
                # set drive mode
                robot_dof_props["driveMode"][self._dof_offset + index] = omni_dc.DRIVE_NONE
                robot_dof_props["stiffness"][self._dof_offset + index] = 0.0
                robot_dof_props["damping"][self._dof_offset + index] = 0.0
            elif ctrl == 'position':
                # set drive mode
                robot_dof_props["driveMode"][self._dof_offset + index] = omni_dc.DRIVE_POS
                robot_dof_props["stiffness"][self._dof_offset + index] = 1.0e12 #5
                robot_dof_props["damping"][self._dof_offset + index] = 000
            else:
                msg = "The robot only supports the following control types: [\"velocity\", \"effort\", \"position\"]"
                print_error(msg)
                raise NotImplementedError(msg)
        # set dof properties
        self._dc_handle.set_articulation_dof_properties(self._articulation_handle, robot_dof_props)

# EOF


