"""
@author     Mayank Mittal
@email      mittalma@ethz.ch

@brief      Defines base class for creating environments using omniverse simulator.
"""

# python
from collections import OrderedDict
import os
import numpy as np
from typing import Optional, List
# gym
from gym import error, spaces
from gym.utils import seeding
import gym
# omniverse
from pxr import Usd, UsdGeom, Gf, PhysicsSchema, PhysicsSchemaTools, PhysxSchema
# mpulator
from teleop_python.omniverse.kit import OmniKitHelper


def convert_observation_to_space(observation):
    if isinstance(observation, dict):
        space = spaces.Dict(OrderedDict([
            (key, convert_observation_to_space(value))
            for key, value in observation.items()
        ]))
    elif isinstance(observation, np.ndarray):
        low = np.full(observation.shape, -float('inf'), dtype=np.float32)
        high = np.full(observation.shape, float('inf'), dtype=np.float32)
        space = spaces.Box(low, high, dtype=observation.dtype)
    else:
        raise NotImplementedError(type(observation), observation)

    return space


class OmniverseEnv(gym.Env):
    """Superclass for all Omniverse environments.
    """

    def __init__(self, dt):
        # 
        self._dt = dt
        # create kit instance
        config = None
        self._kit = OmniKitHelper(config=config)
        # setup physics of the simulator
        self._meters_per_unit = self._setup_physics()
        # set seed of environment
        self.seed()

    """
    Internals
    """

    def _setup_physics(self) -> float:
        """
        Setups the physics configurations for the simulator.

        :return: The conversion of meters into simulator units.
        """
        # Setup the meters per unit parameter
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(self.stage)
        # Specify gravity
        gravityScale = 9.81 / metersPerUnit
        gravity = Gf.Vec3f(0.0, 0.0, -gravityScale)
        # Create the physics scene
        scene = PhysicsSchema.PhysicsScene.Define(self.stage, "/World/physics/scene")
        scene.CreateGravityAttr().Set(gravity)
        # Apply the physics to the stage
        PhysxSchema.PhysxSceneAPI.Apply(self.stage.GetPrimAtPath("/World/physics/scene"))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(self.stage, "/World/physics/scene")
        physxSceneAPI.CreatePhysxSceneEnableCCDAttr(True)
        physxSceneAPI.CreatePhysxSceneEnableStabilizationAttr(True)
        physxSceneAPI.CreatePhysxSceneEnableGPUDynamicsAttr(False)
        physxSceneAPI.CreatePhysxSceneBroadphaseTypeAttr("MBP")
        physxSceneAPI.CreatePhysxSceneSolverTypeAttr("TGS")

        return metersPerUnit

    """
    Properties
    """

    @property
    def kit(self) -> OmniKitHelper:
        """
        :return: The omniverse kit helper class.
        """
        return self._kit

    @property
    def stage(self) -> Usd.Stage:
        """
        :return: The USD stage for the scene.
        """
        return self._kit.get_stage()

    """
    Operations
    """

    def seed(self, seed: Optional[int] = None) -> List[int]:
        """ Sets seed for the environment.

        :param seed: The seed number for the environment. If none, a random seed is created.
        :return: List containing the seed value.
        """
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def close(self):
        """
        Closes the simulator application.
        """
        self._kit.stop()
        self._kit.close()

# EOF
