import numpy as np
import os

from spidey_python.command_generator.spidey_walking import SpideyWalkingCommandGenerator
from spidey_python.ik_solver import IKSolver, interpolate_between_waypoints, interpolate_trajectory





if __name__ == "__main__":
    spidey_path = os.path.join(os.environ["SPIDEY_ROOT"], "spidey_py/spidey_python/ik_solver/spidey_assets/urdf_and_meshes/spidey_v2.urdf")
    spidey_IK_solver = IKSolver(urdf_file = spidey_path, model_name="spidey", root_link_name ="base_link", visualize=False)
    spidey_IK_solver.end_effector_frames = ["leg1_link3_tip","leg2_link3_tip","leg3_link3_tip","leg4_link3_tip"]



    init_waypoints = []
    forward_waypoints = []

    CommandGenerator = SpideyWalkingCommandGenerator(desired_EE_poses=None, desired_root_pose=None)

    init_waypoints += CommandGenerator.default_state()
    init_waypoints += CommandGenerator.move_3_to_IN_init()
    init_waypoints += CommandGenerator.move_1_to_IN_init()
    
    init_trajectory = interpolate_trajectory(init_waypoints)
    

    forward_waypoints += CommandGenerator.move_1_to_OUT()
    forward_waypoints += CommandGenerator.move_body_forward()
    forward_waypoints += CommandGenerator.move_4_to_IN()
    forward_waypoints += CommandGenerator.move_2_to_OUT()
    forward_waypoints += CommandGenerator.move_body_forward()
    forward_waypoints += CommandGenerator.move_3_to_IN()

    forward_trajectory = interpolate_trajectory(forward_waypoints)
