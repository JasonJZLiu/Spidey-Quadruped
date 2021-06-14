#!/usr/bin/python
"""
Tests the spidey system in Omniverse-Kit.
"""

# python
import numpy as np
import os
import pickle
import random

from spidey_python.omniverse import OmniKitHelper
from spidey_python.utils.message import *
from spidey_python.command_generator.spidey_walking import SpideyWalkingCommandGenerator
from spidey_python.ik_solver import interpolate_between_waypoints, interpolate_trajectory

# Omniverse
from pxr import UsdGeom, Gf, PhysicsSchemaTools


def setup_kit_and_system():
    """
    Launches the simulator and creates the system into the scene.

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
    PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 5000, Gf.Vec3f(0, 0, -0.01), Gf.Vec3f(1.0))

    # import omniverse wrappers
    from spidey_python.omniverse.system import SpideySystem
    # Define system interface
    system_config = None
    system = SpideySystem(stage, prim_path='/spidey', config=system_config, meters_per_unit=meters_per_unit)

    # Create the prims into the scene

    system.create()
    physx_interface.release_physics_objects()
    physx_interface.force_load_physics_from_usd()

    for _ in range(200):
        kit.update(dt=0.01)

    # Set things up after hitting play
    kit.play()
    # robot
    system.setup(dc=dc_interface)
    # return the robot and kit
    return kit, system


def test_basic():
    """
    The test launches the simulator, loads the system and prints the kinematic chain and state of the robot.
    """
    # Setup simulator
    kit, system = setup_kit_and_system()
    # display information
    system.robot.display()

def test_direct_advance():
    """
    The test lauches the simulator, creates the system, and tries random arm commands.
    """
    # Specify the timestep for physics
    dt = 1 / 60.0
    # Setup simulator
    kit, system = setup_kit_and_system()
    # run the simulator
    for idx in range(50000000):
        # commands for the robot
        if idx < 500:
            system.robot.advance(spidey_cmd=np.array([0,    0,    0,
                                                      0,    0,    0,
                                                      0,    0,    0,
                                                      0,    0,    0]))
        # Step through physics
        kit.update(dt=dt)
        # Update state of robot
        system.update()
        # print robot stats
        #print_info(system.robot)
    print("Feet Poses: ", [system.robot.parts['spidey'].I_r_IE, system.robot.parts['spidey'].q_IE])
    print ("Base Link Pose: ", system.robot.parts['spidey'].base_link_pose)
    kit.close()


def test_system_advance():
    """
    The test lauches the simulator, creates the system, and follows a specified trajectory.
    """
    # Specify the timestep for physics
    dt = 0.1 / 60.0
    # Setup simulator
    kit, system = setup_kit_and_system()
    # run the simulator
    error_tol = 0.002



    CommandGenerator = SpideyWalkingCommandGenerator(desired_EE_poses = None, desired_root_pose = None)
    
    init_waypoints = []
    init_waypoints += CommandGenerator.default_state()
    init_waypoints += CommandGenerator.move_3_to_IN_init()
    init_waypoints += CommandGenerator.move_1_to_IN_init()
  
    init_trajectory = interpolate_trajectory(init_waypoints)

    forward_waypoints = []
    forward_waypoints += [CommandGenerator.desired_cmd]
    forward_waypoints += CommandGenerator.move_1_to_OUT()
    forward_waypoints += CommandGenerator.move_body_forward()
    forward_waypoints += CommandGenerator.move_4_to_IN()
    forward_waypoints += CommandGenerator.move_2_to_OUT()
    forward_waypoints += CommandGenerator.move_body_forward()
    forward_waypoints += CommandGenerator.move_3_to_IN()

    forward_trajectory = interpolate_trajectory(forward_waypoints)

    for desired_cmd in init_trajectory:
        error = float('inf')
        time = 0
        while error > error_tol and time < 150:
            time += 1
            system.advance(desired_cmd=desired_cmd)
            kit.update(dt=dt)
            system.update()
            error = np.sum(np.abs(system.spidey_joint_cmd-system.robot.q))
            print("\n----------------------------------------------------------------------")
            print("System Desired Joint Positions: ", system.spidey_joint_cmd)
            print("Robot Positions in Sim: ", system.robot.q)
            print("ERROR: ", np.sum(np.abs(system.spidey_joint_cmd-system.robot.q)))

            print ("\n------Base Link Pose in Sim------")
            print(system.robot.parts['spidey'].base_link_pose[4:])
            print("------Feet Poses in Sim------")
            print(system.robot.parts['spidey'].I_r_IE)
            print("------Feet Poses Commanded------")
            print([desired_cmd["desired_feet_poses"][i][:3] for i in range(4)])
            print("----------------------------------------------------------------------\n \n")
        

    while True:
        for desired_cmd in forward_trajectory:
            error = float('inf')
            time = 0
            while error > error_tol and time < 150:
                time += 1
                system.advance(desired_cmd=desired_cmd)
                kit.update(dt=dt)
                system.update()
                error = np.sum(np.abs(system.spidey_joint_cmd-system.robot.q))
                print("\n----------------------------------------------------------------------")
                print("System Desired Joint Positions: ", system.spidey_joint_cmd)
                print("Robot Positions in Sim: ", system.robot.q)
                print("ERROR: ", np.sum(np.abs(system.spidey_joint_cmd-system.robot.q)))

                print ("\n------Base Link Pose in Sim------")
                print(system.robot.parts['spidey'].base_link_pose[4:])
                print("------Feet Poses in Sim------")
                print(system.robot.parts['spidey'].I_r_IE)
                print("------Feet Poses Commanded------")
                print([desired_cmd["desired_feet_poses"][i][:3] for i in range(4)])
                print("----------------------------------------------------------------------\n \n")




    while True:
        system.advance(desired_cmd=trajectory[-1])
        kit.update(dt=dt)
        system.update()
    
    kit.close()



if __name__ == "__main__":

    #test_direct_advance()
    test_system_advance()



# EOF


