#!/usr/bin/python
"""
Tests the spidey robot in Omniverse-Kit.
"""

# python
import numpy as np
import os

from spidey_python.omniverse import OmniKitHelper
from spidey_python.utils.message import *
# Omniverse
from pxr import UsdGeom, Gf, PhysicsSchemaTools


def setup_kit_and_robot():
    """
    Launches the simulator and creates the robot into the scene.

    :return: Tuple containing the kit instance and the robot interface object.
    """
    # Load kit helper
    kit_config = {
        'height': 480,
        'width': 640,
        'headless': False
    }
    kit = OmniKitHelper(kit_config)
    # Acquire stage units conversions
    
    meters_per_unit = kit.meters_per_unit
    # Acquire current stage of simulation
    stage = kit.get_stage()
    print_warn(stage)
    # Acquire dynamic control tooblox interface
    dc_interface = kit.get_dynamic_control_interface()
    # Acquire PhysX interface
    import omni.physx._physx as omni_physx
    physx_interface = omni_physx.acquire_physx_interface()
    physx_interface.overwrite_gpu_setting(0)

    kit.stop()
    # Force load the USD scene
    physx_interface.force_load_physics_from_usd()
    physx_interface.release_physics_objects()
    # Create scene
    # setup the physics
    kit.setup_physics("/World/physics/scene")
    # create ground plane
    PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 5000, Gf.Vec3f(0, 0, -0.001/meters_per_unit), Gf.Vec3f(1.0))

    # import omniverse wrappers
    from spidey_python.omniverse.robot import SpideyRobot
    # Define robot interface
    robot_config = None
    robot = SpideyRobot(stage, prim_path='/spidey', config=robot_config, meters_per_unit=meters_per_unit)

    # Create the prims into the scene
    robot.create()
    physx_interface.release_physics_objects()
    physx_interface.force_load_physics_from_usd()

    for _ in range(500):
        kit.update(dt=0.01)

    # Set things up after hitting play
    kit.play()
    # robot
    robot.setup(dc=dc_interface)
    # return the robot and kit
    return kit, robot


def test_basic():
    """
    The test launches the simulator, loads the robot and prints the kinematic chain and state.
    """
    # Setup simulator
    kit, robot = setup_kit_and_robot()
    # display information
    robot.display()

def test_advance():
    """
    The test lauches the simulator, creates the robot, and tries random base commands.
    """
    # Specify the timestep for physics
    dt = 1 / 60.0
    # Setup simulator
    kit, robot = setup_kit_and_robot()
    # run the simulator
    for idx in range(100000000):
        # commands for the robot
        if idx < 500:
            robot.advance(spidey_cmd=np.array([1,    0,    0,
                                               1,    0,    0,
                                               1,    0,    0,
                                               1,    0,    0]))
        elif idx < 100000:
            robot.advance(spidey_cmd=np.array([-1,    0,    0,
                                               -1,    0,    0,
                                               -1,    0,    0,
                                               -1,    0,    0]))


        # Step through physics
        kit.update(dt=dt)
        # Update state of robot
        robot.update()
        # print robot stats
        #print_info(robot)

    kit.close()

def test_set_state():
    """
    Tests setting random initial pose of the prim.
    """
    # Setup simulator
    kit, robot = setup_kit_and_robot()
    # Get default state
    def_state = robot.default_state
    # run the simulator
    # for idx in range(1000):
    #     # commands for the robot
    #     if idx % 100 == 0:
    #         state = def_state
    #         state["pos"][0:7] = (np.random.random(7))*2-1
    #         robot.set_state(state["pos"], state["vel"])
    #     # Step through physics
    #     kit.update(dt=0.0)
    #     # Update state of robot
    #     robot.update()
    #     # print robot stats
    #     print_info(robot)


    for idx in range(1000):
        # commands for the robot
       
        state = def_state
        state["pos"][0:7] = np.array([0.0000, 0.1710, 0.1144, -1.5700 , 0.0500, 1.5700, 0.4690])
        print("ASLFKJSAFKLSAKDJFSAKFLSAJKLF",state)
        #[0.0, 0.171, 0.1144, -1.57, 0.05, 1.57, 0.469])

        #[0, 0.13226619, 0.0993359, -0.55023599, 1.50935026, 0.74207276, -1.42979813])


        robot.set_state(state["pos"], state["vel"])
        # Step through physics
        kit.update(dt=0.0)
        # Update state of robot
        robot.update()
        # print robot stats
        print_info(robot)

    kit.close()



if __name__ == "__main__":
    #setup_kit_and_robot()
    #test_basic()
    test_advance()
    # test_set_prim_pose()
    #test_set_state()
    # test_toggle_visibility()
    #test_gripper_action()

# EOF
