"""
@author     Mayank Mittal, Jingzhou Liu
@email      mittalma@ethz.ch, jingzhou.liu@mail.utoronto.ca
@brief      Implementation of Spidey robot in Isaac Sim.
"""

# python
import os
import numpy as np
import scipy.spatial.transform as tf
from typing import Optional
# omniverse
from pxr import Usd, UsdGeom, Gf, Semantics
import omni.isaac.dynamic_control._dynamic_control as omni_dc
# spidey
from spidey_python.utils.message import *
from spidey_python.utils.errors import *
from spidey_python.omniverse.robot.robot_base import RobotBase
from spidey_python.omniverse.robot.spidey import SpideyDim, SpideyBot

# Default dictionary for setting up the spidey robot.
SPIDEY_ROBOT_DEFAULT_CONFIG = {
    # Type of controller for spidey ["position", "velocity", "effort"]
    'spidey_ctrl': 'position',
    # Disable gravity for spidey
    'spidey_disable_gravity': False,
    # path to the USD file for the robot
    'usd_path': os.path.join(os.environ["SPIDEY_ROOT"], "omniverse/resources/usd", "robot/spidey/spidey.usd")
}


class SpideyRobot(RobotBase):
    """
    @brief Implementation of Spidey robot.

    State: q_j, dq_j
    Input: dq_j
    """

    """
    Instantiation
    """

    def __init__(self, stage: Usd.Stage, prim_path: Optional[str] = '/spidey',
                 config: Optional[dict] = None, meters_per_unit: Optional[float] = 1.0):
        """
        Defines the variables and constants for the system.

        :param stage: The USD stage to import robot into.
        :param prim_path: The path for the primitive in the stage.
        :param config: A dictionary containing parameters for the class (default: SPIDEY_ROBOT_DEFAULT_CONFIG).
        :param meters_per_unit: The units of conversion from simulator's scale to meters.
        """
        super().__init__()
        # Check that input is correct
        assert os.path.isabs(prim_path)
        assert isinstance(meters_per_unit, float)
        # Copy args to internal variables
        self._prim_path = prim_path
        self._meters_per_unit = meters_per_unit
        # Update the default dictionary
        self._config = SPIDEY_ROBOT_DEFAULT_CONFIG
        if config is not None:
            self._config.update(config)
        # Persistent Scene-graph related in Universal Scene Description
        self._stage = stage
        self._prim = None
        # Handles to various ov-kit plugins
        self._dc_handle = None
        # Handles related to robot
        self._articulation_handle = None
        # Definitions for various internal dimensions
        self._dims = SpideyDim
        # Parts of the robot are defined separately since each are controlled differently.
        self._parts = {
            "spidey": SpideyBot(meters_per_unit, ee_handle_names=["leg1_link3_tip","leg2_link3_tip","leg3_link3_tip","leg4_link3_tip"], transform_ext=None)
        }
        # Store DOF properties
        self._dof_properties = {
            "lower_limits": np.array([]),
            "upper_limits": np.array([]),
            "max_velocity": np.array([]),
            "max_efforts": np.array([]),
        }
        # Default state of the robot
        self._robot_default_state = {
            "pos": np.concatenate([self._parts["spidey"].default_state["pos"]]),
            "vel": np.concatenate([self._parts["spidey"].default_state["vel"]])
        }
        # Dynamics information of the robot
        self._robot_state = {
            # Generalized coordinates: spidey joints (12)
            "pos": np.zeros(self._dims.GeneralizedCoordinatesDim.value),
            # Generalized velocities: spidey joints (12)
            "vel": np.zeros(self._dims.GeneralizedVelocitiesDim.value)
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
        msg = f"Spidey @ \'{self._prim_path}\'\n" \
              "  Spidey Feet Poses:\n" \
              f"    I_r_IE: {self._parts['spidey'].I_r_IE} \n" \
              f"    q_IE: {self._parts['spidey'].q_IE} \n" \
              "  Spidey State:\n" \
              f"    q_j: {self.q[0:]} \n" \
              f"    dq_j: {self.u[0:]} \n" \

        return msg

    """
    Properties
    """

    @property
    def prim(self) -> Usd.Prim:
        """
        :return: The USD primitive instance corresponding to the robot.
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
        :return: The generalized coordinates of the robot.
        """
        return self._robot_state["pos"] 

    @property
    def u(self) -> np.ndarray:
        """
        :return: The generalized velocities of the robot.
        """
        return self._robot_state["vel"]

    @property
    def config(self) -> dict:
        """
        :return: A dictionary containing parameters for the class.
        """
        return self._config

    @property
    def parts(self) -> dict:
        """
        :return: A dictionory providing direct access to various parts of the robot.
        """
        return self._parts

    @property
    def default_state(self) -> dict:
        """
        :return: The default state of the robot.
        """
        return self._robot_default_state

    @property
    def state(self) -> dict:
        """
        :return: The current state of the robot.
        """
        return self._robot_state
    
    @property
    def base_link_pose(self)-> np.ndarray:
        """
        :return: The current pose of the base_link: [q_w, q_x, q_y, q_z, x_root, y_root, z_root]
        """
        return self._parts["spidey"].base_link_pose

    """
    Helpers
    """

    def toggle_visibility(self, visible: bool):
        """ Toggle visibility of the robot prim in the scene.

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
        """ Set location of the root of the robot in the stage.

        :param pos: (x, y, z) cartesian coordinates for location of root of the robot in the world frame.
        :param quat: (x, y, z, w) quaternion coordinates of orientation of root of the robot in the world frame.
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
            # xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
            # xform_op.Set(Gf.Matrix4d().SetTranslate(pos).SetRotate(rotm))
            matrix = Gf.Matrix4d().SetTranslateOnly(pos).SetRotateOnly(rotm)
            xform.AddTransformOp().Set(matrix)


    def set_state(self, q: np.ndarray, u: np.ndarray):
        """ Set the dof state of the robot.

        :param q: Generalized coordinates for the robot.
        :param u: Generalized velocities for the robot.
        """
        # convert input to numpy array (sanity)
        q = np.asarray(q)
        u = np.asarray(u)
        # check input is of right shape
        assert q.shape == (self._dims.GeneralizedCoordinatesDim.value,)
        assert u.shape == (self._dims.GeneralizedVelocitiesDim.value,)
        # for spidey
        self._parts["spidey"].set_state(q[:], u[:])
 

    """
    Operations
    """

    def create(self):
        """
        Loads the robot into the Omniverse stage.

        @note This function is kept separate in case one wants to create an instance of the class without launching
              the simulator. Or, if one doesn't want to create a new primitive programmatically but refer to an
              exisiting one in the current USD stage.
        """
        # Extract USD path from configuration
        usd_path = self._config["usd_path"]
        # check that path exists
        if not os.path.exists(usd_path):
            msg = f"File not found: {usd_path}"
            print_error(msg)
            raise FileNotFoundError(msg)
        else:
            print_info(f"Loading from: {usd_path}.")
        # define persistent scene graph geometry for the robot
        self._prim = self._stage.DefinePrim(self._prim_path, "Xform")
        # add reference to the USD in the current stage
        self._prim.GetReferences().AddReference(usd_path)
        # check that the path to articulation in scene-graph is correct
        assert self._prim_path == self._prim.GetPath().pathString

    def setup(self, dc: omni_dc.DynamicControl):
        """
        Registers the assets and configures internal variables of the robot.

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
        # initialize handle to the articulation for robot through dynamic control toolbox
        self._articulation_handle = self._dc_handle.get_articulation(self._prim_path)
        if self._articulation_handle == omni_dc.INVALID_HANDLE:
            raise InvalidHandleError(f"Failed to obtain robot at \'{self._prim_path}\'")

        # get number of degrees of freedom of robot
        num_dofs = self._dc_handle.get_articulation_dof_count(self._articulation_handle)
        num_dofs_expected = self._parts["spidey"].dof
        # check that number of DOFs are correct
        if num_dofs != num_dofs_expected:
            raise OmniverseError(f"Incorrect number of degrees of freedom. "
                                 f"Expected {num_dofs_expected} but received {num_dofs}.")

        # setup handles for the robot
        # For spidey
        self._parts["spidey"].setup(self._articulation_handle, self._dc_handle, ctrl=self._config["spidey_ctrl"],
                                 dof_offset=0,
                                 disable_gravity=self._config["spidey_disable_gravity"])

        # get joint poperties
        dof_props = self._dc_handle.get_articulation_dof_properties(self._articulation_handle)
        # store essential dof properties internally
        self._dof_properties["lower_limits"] = np.asarray(dof_props["lower"])
        self._dof_properties["upper_limits"] = np.asarray(dof_props["upper"])
        self._dof_properties["max_velocity"] = np.asarray(dof_props["maxVelocity"])
        self._dof_properties["max_effort"] = np.asarray(dof_props["maxEffort"])
        # root spawned position
        self.set_prim_pose(pos=np.array([0.0, 0.0, 0.0]), quat=None)
        # set default initial state of the robot
        self.set_state(q=self._robot_default_state["pos"], u=self._robot_default_state["vel"])
        # update the internal buffers
        self.update()
        # print status
        print_notify(f"Setup complete for spidey robot: \'{self._prim_path}\'.")

    def advance(self, spidey_cmd: np.ndarray = None):
        """Apply input command to the robot.
        :param spidey_cmd: The joint command for spidey.
        """
        if spidey_cmd is None:
            spidey_cmd = self.q[:]
        # apply command to the robot

        self._parts["spidey"].apply_command(spidey_cmd)

    def update(self):
        """
        Updates the buffers for dynamics state of the robot.
        """
        # update the wrappers
        self._parts["spidey"].update()

        # fill base pose to generalized coordinates
        self._robot_state["pos"][:] = self._parts["spidey"].state["pos"]

        # fill base velocity to generalized velocities
        self._robot_state["vel"][:] = self._parts["spidey"].state["vel"]

    def display(self):
        """
        Display the configuration of the robot.
        """
        print(f"Articulation handle: {self._articulation_handle}")
        # Print information about kinematic chain
        root_link_index = self._dc_handle.get_articulation_root_body(self._articulation_handle)
        print("--- Hierarchy:\n"
              f"{self._convert_kinematic_hierarchy_to_string(root_link_index)}")
        # Information about the body states of the robot
        body_states = self._dc_handle.get_articulation_body_states(self._articulation_handle, omni_dc.STATE_ALL)
        print_info("--- Body states:\n"
                   f"{body_states}")
        # Information about the DOF states of the robot.
        dof_states = self._dc_handle.get_articulation_dof_states(self._articulation_handle, omni_dc.STATE_ALL)
        print_info("--- DOF states:\n"
                   f"{dof_states}")
        # Information about the DOF properties of the robot.
        dof_props = self._dc_handle.get_articulation_dof_properties(self._articulation_handle)
        print_info("--- DOF properties:\n"
                   "[type] [has-limits] [lower] [upper] [drive-mode] [max-vel] [max-effort] [stiffness] [damping]\n"
                   f"{dof_props}")

    """
    Internals
    """

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
