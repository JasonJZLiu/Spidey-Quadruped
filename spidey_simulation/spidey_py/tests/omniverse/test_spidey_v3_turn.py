
from spidey_python.command_generator.spidey_turning import SpideyTurningCommandGenerator
from spidey_python.ik_solver import IKSolver, interpolate_between_waypoints, interpolate_trajectory
import numpy as np
import os, time

#plant.SetFreeBodyPose(plant_context, B_Ggrasp, X_WGgrasp)


if __name__ == "__main__":
  spidey_path = os.path.join(os.environ["SPIDEY_ROOT"], "spidey_py/spidey_python/ik_solver/spidey_assets/urdf_and_meshes_v3/spidey_v3.urdf")
  SpideyIKSolver = IKSolver(urdf_file = spidey_path, 
                              model_name = "spidey", 
                              root_link_name = "base_link",
                              visualize = True,
                              position_tolerance = 0.001)

  SpideyIKSolver.end_effector_frames = ['leg1_link3_tip', 'leg2_link3_tip', 'leg3_link3_tip', 'leg4_link3_tip']
  # SpideyIKSolver.set_angular_positions([0,0,0,0,
  #                                       0,0,0,0,
  #                                       0,0,0,0])
  SpideyIKSolver.update_meshcat()
  #print(SpideyIKSolver.get_base_and_EE_poses())

  init_waypoints = []

  CommandGenerator = SpideyTurningCommandGenerator(desired_EE_poses=None, desired_root_pose=None)



  desired_cmd = CommandGenerator.default_right_ready_state()[0]
  reference_spidey_joint_position = SpideyIKSolver.get_ik_solution(desired_EE_poses = desired_cmd["desired_feet_poses"],
                                                                    desired_root_pose = desired_cmd["desired_base_link_pose"],
                                                                    q_initial_guess=None,
                                                                    verbose=True)
  SpideyIKSolver.update_meshcat()
  input("wait")


  num = 0
  for i in range (1):
    num += 1
    # if num == 3:
    #   CommandGenerator.delta_angle *= -1
    trajectory = []
    trajectory += CommandGenerator.turn_CCW_from_right_move_1()
    trajectory += CommandGenerator.turn_CCW_from_right_move_2()
    trajectory += CommandGenerator.turn_CCW_from_right_move_3()
    trajectory += CommandGenerator.turn_CCW_from_right_move_4()
    trajectory += CommandGenerator.turn_CCW_from_right_move_5()
    trajectory = interpolate_trajectory(trajectory)

    for desired_cmd in trajectory:
      reference_spidey_joint_position = SpideyIKSolver.get_ik_solution(desired_EE_poses = desired_cmd["desired_feet_poses"],
                                                                      desired_root_pose = desired_cmd["desired_base_link_pose"],
                                                                      q_initial_guess=None,
                                                                      verbose=True)
      SpideyIKSolver.update_meshcat()
      time.sleep(0.025)
      # input("wait")




  