from spidey_python.command_generator.spidey_walking import SpideyWalkingCommandGenerator, SpideyTurningCommandGenerator
from spidey_python.ik_solver import IKSolver, interpolate_between_waypoints, interpolate_trajectory

import numpy as np
import os, pickle



if __name__ == "__main__":

    spidey_path = os.path.join(os.environ["SPIDEY_ROOT"], "spidey_py/spidey_python/ik_solver/spidey_assets/urdf_and_meshes_v3/spidey_v3.urdf")
    spidey_IK_solver = IKSolver(urdf_file = spidey_path, model_name="spidey", root_link_name ="base_link", visualize=False)
    spidey_IK_solver.end_effector_frames = ["leg1_link3_tip","leg2_link3_tip","leg3_link3_tip","leg4_link3_tip"]

    CommandGenerator = SpideyWalkingCommandGenerator(desired_EE_poses=None, desired_root_pose=None)


    init_waypoints = []
    forward_waypoints = []

    init_waypoints += CommandGenerator.default_state()
    init_waypoints += CommandGenerator.move_3_to_IN_init()
    init_waypoints += CommandGenerator.move_1_to_IN_init()

    init_trajectory = interpolate_trajectory(init_waypoints)



    forward_waypoints += [CommandGenerator.desired_cmd]
    forward_waypoints += CommandGenerator.move_1_to_OUT()
    forward_waypoints += CommandGenerator.move_body_forward()
    forward_waypoints += CommandGenerator.move_4_to_IN()
    forward_waypoints += CommandGenerator.move_2_to_OUT()
    forward_waypoints += CommandGenerator.move_body_forward()
    forward_waypoints += CommandGenerator.move_3_to_IN()

    forward_trajectory = interpolate_trajectory(forward_waypoints)



    init_joint_cmd = []
    walking_joint_cmd = []

    for desired_cmd in init_trajectory:
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
        init_joint_cmd.append(spidey_joint_cmd_angle)

    print("\n-------------------------------------------------------\n")


    for desired_cmd in forward_trajectory:
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
        walking_joint_cmd.append(spidey_joint_cmd_angle)


    print("\n-------------------------------------------------------\n")
    print(init_joint_cmd)
    print("\n-------------------------------------------------------\n")
    print(walking_joint_cmd)


    filename = 'joint_cmds/init_joint_cmd'
    outfile = open(filename,'wb')
    pickle.dump(init_joint_cmd,outfile)
    outfile.close()

    filename = 'joint_cmds/walking_joint_cmd'
    outfile = open(filename,'wb')
    pickle.dump(walking_joint_cmd,outfile)
    outfile.close()