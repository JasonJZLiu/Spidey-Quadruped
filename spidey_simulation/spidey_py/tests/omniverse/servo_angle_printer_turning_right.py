from spidey_python.command_generator.spidey_turning import SpideyTurningCommandGenerator
from spidey_python.ik_solver import IKSolver, interpolate_between_waypoints, interpolate_trajectory

import numpy as np
import os, pickle



if __name__ == "__main__":

    spidey_path = os.path.join(os.environ["SPIDEY_ROOT"], "spidey_py/spidey_python/ik_solver/spidey_assets/urdf_and_meshes_v3/spidey_v3.urdf")
    spidey_IK_solver = IKSolver(urdf_file = spidey_path, model_name="spidey", root_link_name ="base_link", visualize=False)
    spidey_IK_solver.end_effector_frames = ["leg1_link3_tip","leg2_link3_tip","leg3_link3_tip","leg4_link3_tip"]

    CommandGenerator = SpideyTurningCommandGenerator(desired_EE_poses=None, desired_root_pose=None)

    turn_right_waypoints = []

    turn_right_waypoints += CommandGenerator.default_right_ready_state()
    turn_right_waypoints += CommandGenerator.turn_CCW_from_right_move_1()
    turn_right_waypoints += CommandGenerator.turn_CCW_from_right_move_2()
    turn_right_waypoints += CommandGenerator.turn_CCW_from_right_move_3()
    turn_right_waypoints += CommandGenerator.turn_CCW_from_right_move_4()
    turn_right_waypoints += CommandGenerator.turn_CCW_from_right_move_5()

    turn_right_trajectory = interpolate_trajectory(turn_right_waypoints)


    turn_right_joint_cmd = []

    for desired_cmd in turn_right_trajectory:
        reference_spidey_joint_position = spidey_IK_solver.get_ik_solution(desired_EE_poses = desired_cmd["desired_feet_poses"],
                                                                        desired_root_pose = desired_cmd["desired_base_link_pose"],
                                                                        q_initial_guess=None,
                                                                        verbose=False)
        q_result = reference_spidey_joint_position
        spidey_joint_cmd = [q_result[0], q_result[4], q_result[8],
                            q_result[1], q_result[5], q_result[9],
                            q_result[2], q_result[6], q_result[10],
                            q_result[3], q_result[7], q_result[11]]

        spidey_joint_cmd_angle = [element * 180/np.pi for element in spidey_joint_cmd]

        #print(spidey_joint_cmd_angle)
        turn_right_joint_cmd.append(spidey_joint_cmd_angle)


    print("\n-------------------------------------------------------\n")
    print(turn_right_joint_cmd)



    filename = 'joint_cmds/turn_right_joint_cmd'
    outfile = open(filename,'wb')
    pickle.dump(turn_right_joint_cmd, outfile)
    outfile.close()