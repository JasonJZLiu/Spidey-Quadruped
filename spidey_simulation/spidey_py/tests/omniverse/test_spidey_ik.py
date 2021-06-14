
from spidey_python.command_generator.spidey_walking import SpideyWalkingCommandGenerator
from spidey_python.ik_solver import IKSolver, interpolate_between_waypoints, interpolate_trajectory
import numpy as np
import os, time

#plant.SetFreeBodyPose(plant_context, B_Ggrasp, X_WGgrasp)


if __name__ == "__main__":
  spidey_path = os.path.join(os.environ["SPIDEY_ROOT"], "spidey_py/spidey_python/ik_solver/spidey_assets/urdf_and_meshes/spidey_v2.urdf")
  SpideyIKSolver = IKSolver(urdf_file = spidey_path, 
                              model_name = "spidey", 
                              root_link_name = "base_link",
                              visualize = True,
                              position_tolerance = 0.001)

  SpideyIKSolver.end_effector_frames = ['leg1_link3_tip', 'leg2_link3_tip', 'leg3_link3_tip', 'leg4_link3_tip']
  # SpideyIKSolver.set_angular_positions([0,0,0,0,
  #                                       0,0,0,0,
  #                                       -0.174533,-0.174533,-0.174533,-0.174533])
  SpideyIKSolver.update_meshcat()
  #print(SpideyIKSolver.get_base_and_EE_poses())

  init_waypoints = []

  CommandGenerator = SpideyWalkingCommandGenerator(desired_EE_poses=None, desired_root_pose=None)



  input("wait")
  init_waypoints += CommandGenerator.default_state()
  init_waypoints += CommandGenerator.move_3_to_IN_init()
  init_waypoints += CommandGenerator.move_1_to_IN_init()
  
  init_trajectory = interpolate_trajectory(init_waypoints)

  for desired_cmd in init_trajectory:
    reference_spidey_joint_position = SpideyIKSolver.get_ik_solution(desired_EE_poses = desired_cmd["desired_feet_poses"],
                                                                    desired_root_pose = desired_cmd["desired_base_link_pose"],
                                                                    q_initial_guess=None,
                                                                    verbose=True)
    SpideyIKSolver.update_meshcat()
    time.sleep(0.025)
  

  input("wait")
  while True:
    forward_waypoints = []
    forward_waypoints += [CommandGenerator.desired_cmd]
    forward_waypoints += CommandGenerator.move_1_to_OUT()
    forward_waypoints += CommandGenerator.move_body_forward()
    forward_waypoints += CommandGenerator.move_4_to_IN()
    forward_waypoints += CommandGenerator.move_2_to_OUT()
    forward_waypoints += CommandGenerator.move_body_forward()
    forward_waypoints += CommandGenerator.move_3_to_IN()

    forward_trajectory = interpolate_trajectory(forward_waypoints)

    for desired_cmd in forward_trajectory:
      reference_spidey_joint_position = SpideyIKSolver.get_ik_solution(desired_EE_poses = desired_cmd["desired_feet_poses"],
                                                                      desired_root_pose = desired_cmd["desired_base_link_pose"],
                                                                      q_initial_guess=None,
                                                                      verbose=True)
      SpideyIKSolver.update_meshcat()
      time.sleep(0.025)



#EOF