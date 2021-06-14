"""
@author     Mayank Mittal
@email      mittalma@ethz.ch

@brief      Implementation of an articulated object.
"""

# python
import os
import numpy as np
import scipy.spatial.transform as tf
from typing import Optional, List
# omniverse
from pxr import Usd, UsdGeom, Gf, Semantics
import omni.isaac.dynamic_control._dynamic_control as omni_dc
# mpulator gym
from spidey_python.utils.message import *
from spidey_python.utils.errors import *
from spidey_python.omniverse.robot.robot_base import RobotBase


class ArticulatedObject(RobotBase):
    """
    @brief Implementation of an articulated object.

    Articulated object differs from a "articulated object" in the sense that these are passive instances in the
    environment, i.e. the joints are not actuated. However, since their interface resembles that of
    a articulated object, they derive from the base class `RobotBase`.
    """

    """
    Instantiation
    """

    def __init__(self, stage: Usd.Stage, prim_path: str, usd_path: Optional[str] = None,
                 frame_names: List[str] = None, meters_per_unit: Optional[float] = 1.0):
        """
        Defines the variables and constants for the articulated object.

        :param stage: The USD stage to import articulated object into.
        :param prim_path: The path for the primitive in the stage.
        :param usd_path: The path to the USD file to load.
        :param frame_names: A list of frame names whose pose to store.
        :param meters_per_unit: The units of conversion from simulator's scale to meters.
        """
        super().__init__()
        # Check that input is correct
        assert os.path.isabs(prim_path)
        assert isinstance(meters_per_unit, float)
        # Copy args to internal variables
        self._prim_path = prim_path
        self._meters_per_unit = meters_per_unit
        self._usd_path = usd_path
        # Check if any frames specified whose pose to store
        if frame_names is None:
            self._frame_handle_names = list()
        else:
            self._frame_handle_names = frame_names
        # Persistent Scene-graph related in Universal Scene Description
        self._stage = stage
        self._prim = None
        # Handles to various ov-kit plugins
        self._dc_handle = None
        # Handles related to articulated object
        self._articulation_handle = None
        # Count of number of DOF in object
        self._num_dofs = 0
        # Store DOF properties
        self._dof_properties = {
            "lower_limits": np.array([]),
            "upper_limits": np.array([]),
            "max_velocity": np.array([]),
            "max_efforts": np.array([]),
        }
        # Store frame handles and poses
        self._frames_info = dict()
        for frame_name in self._frame_handle_names:
            self._frames_info[frame_name] = {
                'handle': None,
                'pos': np.empty(3),
                'quat': np.empty(4)
            }
        # Default state of the articulated object
        self._default_state = {
            "pos": np.array([]),
            "vel": np.array([])
        }
        # Dynamics information of the articulated object
        self._state = {
            # Generalized coordinates
            "pos": np.array([]),
            # Generalized velocities
            "vel": np.array([])
        }

    def __del__(self):
        """
        Cleanup after exiting
        """
        pass

    def __str__(self) -> str:
        """
        :return: A string containing information about the instance's state.
        """
        # set print options for numpy
        np.set_printoptions(precision=4)
        # print message
        msg = f"Articulated Object @ \'{self._prim_path}\'\n" \
              "  State:\n" \
              f"    q: {self.q} \n" \
              f"    u: {self.u} \n"

        return msg

    """
    Properties
    """

    @property
    def prim(self) -> Usd.Prim:
        """
        :return: The USD primitive instance corresponding to the articulated object.
        """
        return self._prim

    @property
    def prim_path(self) -> str:
        """
        :return: The path to the prim the stage.
        """
        return self._prim_path

    @property
    def dof_properties(self) -> dict:
        """
        :return: A dictionary containing the DOF properties such as joint limits.
        """
        return self._dof_properties

    @property
    def q(self) -> np.ndarray:
        """
        :return: The generalized coordinates of the articulated object.
        """
        return self._state["pos"]

    @property
    def u(self) -> np.ndarray:
        """
        :return: The generalized velocities of the articulated object.
        """
        return self._state["vel"]

    @property
    def frames_info(self) -> dict:
        """
        :return: A nested dictionary with key as the frame names and values as the information
                 about the frame such as position and orientation in world frame.
        """
        return self._frames_info

    @property
    def default_state(self) -> dict:
        """
        :return: The default state of the articulated object.
        """
        return self._default_state

    @property
    def state(self) -> dict:
        """
        :return: The current state of the articulated object.
        """
        return self._state

    """
    Helpers
    """

    def toggle_visibility(self, visible: bool):
        """ Toggle visibility of the articulated object prim in the scene.

        :param visible: Flag to whether make prim visible or invisible.
        """
        # get imageable object
        imageable = UsdGeom.Imageable(self._prim)
        # toggle visibility
        if visible:
            imageable.MakeVisible()
        else:
            imageable.MakeInvisible()

    def set_semantic_label(self, label: str):
        """
        Set the semantic label corresponding to the prim.

        :param label: Name of the semantic label.
        """
        # create semantics api if not exists
        if not self._prim.HasAPI(Semantics.SemanticsAPI):
            sem = Semantics.SemanticsAPI.Apply(self._prim, "Semantics")
            sem.CreateSemanticTypeAttr()
            sem.CreateSemanticDataAttr()
        else:
            sem = Semantics.SemanticsAPI.Get(self._prim, "Semantics")
        # set attributes
        sem.GetSemanticTypeAttr().Set("class")
        sem.GetSemanticDataAttr().Set(label)

    def set_prim_pose(self, pos: np.ndarray, quat: Optional[np.ndarray] = None):
        """ Set location of the root of the object in the stage.

        :param pos: (x, y, z) cartesian coordinates for location of root of the articulated object in the world frame.
        :param quat: (x, y, z, w) quaternion coordinates of orientation of root of the articulated object in the world frame.
                     Default orientation is (0, 0, 0, 1), i.e. identity w.r.t. world.
        """
        if self._prim is None:
            print_warn(f"Prim not found at \'{self._prim_path}\'. Please ensure that the USD stage has the prim.")
            return
        # convert to datatypes accepted by simulator
        if not isinstance(pos, Gf.Vec3d):
            pos = pos / self._meters_per_unit
            pos = Gf.Vec3d(*pos)
        # if orientation not provided, default to identity
        if quat is not None:
            rotm = tf.Rotation.from_quat(quat).as_matrix()
            rotm = Gf.Matrix3d(*rotm.ravel())
        else:
            rotm = Gf.Matrix3d().SetIdentity()
        # set attribute properties for the transform on the primitive
        properties = self._prim.GetPropertyNames()
        if "xformOp:transform" in properties:
            transform_attr = self._prim.GetAttribute("xformOp:transform")
            matrix = self._prim.GetAttribute("xformOp:transform").Get()
            matrix.SetTranslateOnly(pos).SetRotateOnly(rotm)
            transform_attr.Set(matrix)
        else:
            xform = UsdGeom.Xformable(self._prim)
            xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
            xform_op.Set(Gf.Matrix4d().SetTranslate(pos).SetRotate(rotm))

    def set_state(self, q: np.ndarray, u: np.ndarray, **kwargs):
        """ Set the dof state of the articulated object.

        :param q: Generalized coordinates for the object.
        :param u: Generalized velocities for the object.
        """
        # convert input to numpy array (sanity)
        q = np.asarray(q)
        u = np.asarray(u)
        # check input is of right shape
        assert q.shape == (self._num_dofs,)
        assert u.shape == (self._num_dofs,)
        # assign
        # for arm
        dof_states = self._dc_handle.get_articulation_dof_states(self._articulation_handle, omni_dc.STATE_ALL)
        for index in range(self._num_dofs):
            # set initial joint stat
            dof_states["pos"][index] = q[index]
            dof_states["vel"][index] = u[index]
        self._dc_handle.set_articulation_dof_states(self._articulation_handle, dof_states, omni_dc.STATE_ALL)

    """
    Operations
    """

    def create(self):
        """
        Loads the articulated object into the Omniverse stage.

        @note This function is kept separate in case one wants to create an instance of the class without launching
              the simulator. Or, if one doesn't want to create a new primitive programmatically but refer to an
              exisiting one in the current USD stage.
        """
        # Extract USD path from configuration
        usd_path = self._usd_path
        # check that path exists
        if not os.path.exists(usd_path):
            msg = f"File not found: {usd_path}"
            print_error(msg)
            raise FileNotFoundError(msg)
        else:
            print_info(f"Loading from: {usd_path}.")
        # define persistent scene graph geometry for the articulated object
        self._prim = self._stage.DefinePrim(self._prim_path, "Xform")
        # add reference to the USD in the current stage
        self._prim.GetReferences().AddReference(usd_path)
        # check that the path to articulation in scene-graph is correct
        assert self._prim_path == self._prim.GetPath().pathString

    def setup(self, dc: omni_dc.DynamicControl):
        """
        Registers the assets and configures internal variables of the articulated object.

        :param dc: Handle to dynamic control plugin instance.
        """
        # get prim if it doesn't exist yet
        # this is to deal with the scenario when the stage already has the prim so user does not create one.
        if self._prim is None:
            self._prim = self._stage.GetPrimAtPath(self._prim_path)
            # check that prim exists. (GetPrimPath returns invalid prim if one doesn't exist)
            if not self._prim.IsValid():
                msg = f"Prim not found at \'{self._prim_path}\'. Please ensure that the USD stage has the prim."
                print_error(msg)
                raise OmniverseError(msg)
        # initialize dynamic control handle
        self._dc_handle = dc
        # initialize handle to the articulation for articulated object through dynamic control toolbox
        self._articulation_handle = self._dc_handle.get_articulation(self._prim_path)
        if self._articulation_handle == omni_dc.INVALID_HANDLE:
            raise InvalidHandleError(f"Failed to obtain articulated object at \'{self._prim_path}\'")
        # get number of degrees of freedom of articulated object
        self._num_dofs = self._dc_handle.get_articulation_dof_count(self._articulation_handle)
        # setup corresponding frame handle
        self._setup_handles()
        # setup links of the robot
        self._setup_links()
        # setup controls for the robot
        self._setup_control()
        # record initial state of the object in the scene as default state
        dof_states = self._dc_handle.get_articulation_dof_states(self._articulation_handle, omni_dc.STATE_ALL)
        self._default_state["pos"] = np.asarray(dof_states["pos"])
        self._default_state["vel"] = np.zeros_like(self._default_state["pos"])
        # root spawned position
        self.set_prim_pose(pos=np.array([0.0, 0.0, 0.0]), quat=None)
        # set default initial state of the articulated object
        self.set_state(q=self._default_state["pos"],
                       u=self._default_state["vel"])
        # update the internal buffers
        self.update()
        # print status
        print_notify(f"Setup complete for articulated object \'{self._prim_path}\'.")

    def advance(self):
        """Apply input command to the articulated object.

        @note Passive object in the scene with no joint commands.
        """
        pass

    def update(self):
        """
        Updates the buffers for dynamics state of the articulated object.
        """
        # get frame poses
        for frame_name in self._frames_info:
            frame_handle = self._frames_info[frame_name]['handle']
            # pose of the base of the robot
            frame_pose = self._dc_handle.get_rigid_body_pose(frame_handle)
            pos = np.array([frame_pose.p.x, frame_pose.p.y, frame_pose.p.z])
            quat = np.array([frame_pose.r.x, frame_pose.r.y, frame_pose.r.z, frame_pose.r.w])
            # convert from simulator's units to meters
            pos = pos * self._meters_per_unit
            # store into the dictionary
            self._frames_info[frame_name]['pos'] = pos
            self._frames_info[frame_name]['quat'] = quat
        # fill joint state of the object
        dof_states = self._dc_handle.get_articulation_dof_states(self._articulation_handle, omni_dc.STATE_ALL)
        self._default_state["pos"] = np.asarray(dof_states["pos"])
        self._default_state["vel"] = np.asarray(dof_states["vel"])

    def display(self):
        """
        Display the configuration of the articulated object.
        """
        print(f"Articulation handle: {self._articulation_handle}")
        # Print information about kinematic chain
        root_link_index = self._dc_handle.get_articulation_root_body(self._articulation_handle)
        print("--- Hierarchy:\n"
              f"{self._convert_kinematic_hierarchy_to_string(root_link_index)}")
        # Information about the body states of the articulated object
        body_states = self._dc_handle.get_articulation_body_states(self._articulation_handle, omni_dc.STATE_ALL)
        print_info("--- Body states:\n"
                   f"{body_states}")
        # Information about the DOF states of the articulated object.
        dof_states = self._dc_handle.get_articulation_dof_states(self._articulation_handle, omni_dc.STATE_ALL)
        print_info("--- DOF states:\n"
                   f"{dof_states}")
        # Information about the DOF properties of the articulated object.
        dof_props = self._dc_handle.get_articulation_dof_properties(self._articulation_handle)
        print_info("--- DOF properties:\n"
                   "[type] [has-limits] [lower] [upper] [drive-mode] [max-vel] [max-effort] [stiffness] [damping]\n"
                   f"{dof_props}")

    """
    Internals
    """

    def _setup_handles(self):
        """
        Configures the handles of the frames.
        """
        for frame_name in self._frame_handle_names:
            # get frame handle
            frame_handle = self._dc_handle.find_articulation_body(self._articulation_handle, frame_name)
            # check handles are valid
            if frame_handle == omni_dc.INVALID_HANDLE:
                msg = f"*** Failed to load handle at \'{frame_name}\'"
                print_error(msg)
                raise InvalidHandleError(msg)
            # store information into information dictionary
            self._frames_info[frame_name]['handle'] = frame_handle

    def _setup_links(self):
        """
        Configures the properties of the links in the object.
        """
        pass

    def _setup_control(self):
        """
        Configures the controllers for the robot system. Since passive system, we set the DOF
        type to None for all joints.
        """
        # get joint poperties
        dof_props = self._dc_handle.get_articulation_dof_properties(self._articulation_handle)
        # store essential dof properties internally
        self._dof_properties["lower_limits"] = np.asarray(dof_props["lower"])
        self._dof_properties["upper_limits"] = np.asarray(dof_props["upper"])
        self._dof_properties["max_velocity"] = np.asarray(dof_props["maxVelocity"])
        self._dof_properties["max_effort"] = np.asarray(dof_props["maxEffort"])
        # joints: set control type based on specification
        for index in range(self._num_dofs):
            # set drive mode
            dof_props["driveMode"][index] = omni_dc.DRIVE_NONE
            dof_props["stiffness"][index] = 0.0
            dof_props["damping"][index] = 0.0
        # set dof properties
        self._dc_handle.set_articulation_dof_properties(self._articulation_handle, dof_props)

    def _convert_kinematic_hierarchy_to_string(self, body_index, indent_level=0) -> str:
        """ Reads the articulation handle and converts kinematic tree into a string.

        :param body_index: Index of the body to start iteration with.
        :param indent_level: Indentation level in the converted message
        :return: A string message containing the kinematic tree.
        """
        # define current indentation
        indent = "|" + "-" * indent_level
        # get name of the body
        body_name = self._dc_handle.get_rigid_body_name(body_index)
        # add body name to string
        str_output = f"{indent}Body: {body_name}\n"
        # iterate over children of the body
        for i in range(self._dc_handle.get_rigid_body_child_joint_count(body_index)):
            # get joint name
            joint = self._dc_handle.get_rigid_body_child_joint(body_index, i)
            joint_name = self._dc_handle.get_joint_name(joint)
            # get child link name
            child = self._dc_handle.get_joint_child_body(joint)
            child_name = self._dc_handle.get_rigid_body_name(child)
            # add information to string output
            str_output += f"{indent}>>Joint: {joint_name} -> {child_name}\n"
            # iterate recrusively for depth-first-search
            str_output += self._convert_kinematic_hierarchy_to_string(child, indent_level + 4)

        # return result
        return str_output

# EOF
