"""
@author     Jingzhou Liu, Ritvik Singh
@email      jingzhou.liu@mail.utoronto.ca, ritviks.singh@mail.utoronto.ca

@brief      Inverse kinematics solver for kinematic trees implemented using pyDrake.
"""

import importlib
import sys

from meshcat.servers.zmqserver import start_zmq_server_as_subprocess

import numpy as np
from functools import partial

from pydrake.common.jupyter import process_ipywidget_events
from pydrake.systems.framework import BasicVector, VectorSystem

from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve

from pydrake.all import (
    AddMultibodyPlantSceneGraph, ConnectMeshcatVisualizer, DiagramBuilder, 
    FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
    MultibodyPlant, Parser, Simulator)

from pydrake.math import RigidTransform, RotationMatrix, RollPitchYaw
from pydrake.all import RigidTransform, RotationMatrix
from pydrake.geometry import Cylinder
from pydrake.multibody.tree import UnitInertia, SpatialInertia, JointIndex
from pydrake.multibody.plant import CoulombFriction
from pydrake.multibody.inverse_kinematics import InverseKinematics
import pydrake.multibody as multibody

from typing import Optional, Tuple, List


class IKSolver:
  """
  @brief Inverse Kinematics Solver Implemented using pyDrake.

  This class solves the inverse kinematics problem given a kinematic tree via a URDF file, desired
  end-effector positions (or poses), and the root link position (or pose). 
  """
  def __init__ (self, urdf_file: Optional[str] = "", 
                model_name: Optional[str] = "", 
                root_link_name: Optional[str] = "",
                visualize: bool = False,
                position_tolerance: Optional[float] = 0.001):
    """
    Defines the variables and constants for the solver.

    :param urdf_file: The URDF file path.
    :param model_name: The robot name in the URDF. 
    :param root_link_name: The root link of the robot described in the URDF.
    :param visualize: A flag that determines whether the meshcat visualizer is enabled.
    :param position_tolerance: The tolerance that the optimizer uses to compute the solution.
    """         
    self._position_tolerance = position_tolerance
    self._builder = DiagramBuilder()
    self._plant, self._scene_graph = AddMultibodyPlantSceneGraph(self._builder, time_step=1e-4)
    self._model_name = model_name
    self._world_frame = self._plant.world_frame()
    self._root_link_name = root_link_name
    self._robot_model = Parser(self._plant, self._scene_graph).AddModelFromFile(urdf_file, self._model_name)
    self._plant.Finalize()

    self._visualize_state = visualize
    if self._visualize_state:
      proc, zmq_url, web_url = start_zmq_server_as_subprocess(server_args=[])
      self._meshcat = ConnectMeshcatVisualizer(self._builder, self._scene_graph, zmq_url=zmq_url)
      self._meshcat.load()

    self._diagram = self._builder.Build()
    self._context = self._diagram.CreateDefaultContext()
    self._plant_context = self._plant.GetMyContextFromRoot(self._context)
    self._desired_EE_poses = None
    self._EE_frame_names = []
    self._end_effector_frames = []

    self.initial_q = self._plant.GetPositions(self._plant_context)



  @property
  def end_effector_frames(self) -> List[multibody.tree.Frame_[float]]:
    """
    :return: A list of end-effector frames that the solver can reference. 
    """
    return self._end_effector_frames


  @end_effector_frames.setter
  def end_effector_frames(self, EE_frame_names: List[str]):
    """
    :param EE_frame_names: A list of the names of end-effector frames given in the URDF.
    :return: A list of end-effector frames that the solver can reference. 
    """
    self._EE_frame_names = EE_frame_names
    for EE_frame in EE_frame_names:
      self._end_effector_frames.append(self._plant.GetFrameByName(EE_frame, self._robot_model))


  def get_ik_solution(self, desired_EE_poses: List[np.ndarray], 
                      desired_root_pose: np.ndarray, 
                      q_initial_guess: np.ndarray, 
                      verbose: bool = False) -> np.ndarray:
    """
    Solves the inverse kinematics problem.

    :param desired_EE_poses: The desired poses of the end-effectors. [np.array(x_ee1, y_ee1, z_ee1, roll1 pitch1 yaw1), ... ]
    :param desired_root_pose: The pose of the root link. [x_root, y_root, z_root, roll, pitch, yaw]
    :param q_initial_guess: The states that the solver is initialized with. [w, x, y, z, x_root, y_root, z_root, robot_state_1, robot_state_2, ..., robot_state_n]
    :param verbose: A flag that prints out more results when set to True. 
    :return: The states of the robot that yield the desired end-effector positions (poses). 
    """     
    self._plant_context = self._plant.GetMyContextFromRoot(self._context)
    ik = multibody.inverse_kinematics.InverseKinematics(self._plant, self._plant_context)

    root_frame = self._plant.GetFrameByName(self._root_link_name, self._robot_model)
    ik.AddPointToPointDistanceConstraint(root_frame, np.array([0,0,0]), self._world_frame, desired_root_pose[0:3], 0, self._position_tolerance)
    # If desired_root_pose contains the orientations of the end-effectors and the orientations need to be solved, uncomment the following two lines:
    rot = RigidTransform(RollPitchYaw(desired_root_pose[3:6]), desired_root_pose[0:3]).rotation()
    ik.AddOrientationConstraint(root_frame, RotationMatrix(), self._world_frame, rot, 0)

    for i in range(len(desired_EE_poses)):
      desired_pose = desired_EE_poses[i]
      ik.AddPointToPointDistanceConstraint(self._end_effector_frames[i], np.array([0,0,0]), self._world_frame, desired_pose[0:3], 0, self._position_tolerance)
      # If desired_EE_poses contains the orientations of the end-effectors and the orientations need to be solved, uncomment the following two lines:
      # rot = RigidTransform(RollPitchYaw(desired_pose[3:6]), desired_pose[0:3]).rotation()
      # ik.AddOrientationConstraint(self._end_effector_frames[i], RotationMatrix(), self._world_frame, rot, 0)

    prog = ik.get_mutable_prog()
    q = ik.q()

    q_initial_guess = self._plant.GetPositions(self._plant_context)

    prog.SetInitialGuess(q, q_initial_guess)
    result = Solve(prog)
    q_result = result.GetSolution() 
    success_flag = result.is_success()

    q_corrected = self._from_user_to_sim (q_result[7:])

    if verbose:
      self._print_frame_poses()
      print("\nStates Solution: \n", np.array(q_corrected))
      print ("\nSuccess Flag: ", success_flag, "\n")

    return np.array(q_corrected)


  def _print_frame_poses(self):
    """
    Prints out the end-effector poses.
    """     
    print("\nFrame Poses: ")
    for EE_frame_name in self._EE_frame_names[:]+[self._root_link_name]:
      ee = self._plant.GetBodyByName(EE_frame_name) 
      print(EE_frame_name, ": ")
      pose = self._plant.EvalBodyPoseInWorld(self._plant_context, ee) 
      print(np.array2string(pose.translation(), formatter={'float': lambda x: "{:4.3f}".format(x)})) 
      print(np.array2string(RollPitchYaw(pose.rotation()).vector(), formatter={'float': lambda x: "{:4.3f}".format(x)}))
    

  def update_meshcat(self):
    """
    Updates the meshcat visualizer. Only works if visualize = True when initializing the IKSolver.
    """    
    if self._visualize_state:
      publishing_context = self._meshcat.GetMyContextFromRoot(self._context)
      self._meshcat.Publish(publishing_context)
    else:
      print ("--- Visualization is disabled when the IKSolver was initialized. ---")


  def get_base_and_EE_poses(self):
    frames = self._EE_frame_names[:]+[self._root_link_name]
    pose_dict = {}
    for EE_frame_name in frames:
      ee = self._plant.GetBodyByName(EE_frame_name) 
      #print(EE_frame_name, ": ")
      pose = self._plant.EvalBodyPoseInWorld(self._plant_context, ee)
      #print(np.array2string(pose.translation(), formatter={'float': lambda x: "{:4.3f}".format(x)})) 
      #print(np.array2string(RollPitchYaw(pose.rotation()).vector(), formatter={'float': lambda x: "{:4.3f}".format(x)}))
      pose_dict[EE_frame_name] = list(np.array(pose.translation())) + list(RollPitchYaw(pose.rotation()).vector())
    return pose_dict


  def _from_user_to_sim(self, servo_cmd):
    # for i in [2,6,9,10]:
    #   servo_cmd[i] *= 1
    return servo_cmd


  def set_angular_positions(self, servo_cmd):
    base_pose = list(self._plant.GetPositions(self._plant_context)[:7])
    
    self._plant.SetPositions(self._plant_context, 
                       self._plant.GetModelInstanceByName("spidey"),
                       base_pose + self._from_user_to_sim(servo_cmd))

  
