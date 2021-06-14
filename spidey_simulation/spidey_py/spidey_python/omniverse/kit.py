"""
Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto.  Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.

---

Modified by Mayank Mittal (2020, Robotics Systmes Lab, ETH Zurich)

@brief Defines helper class for launching OmniKit from a Python environment.
"""

# python
import os
import importlib
import atexit
import asyncio
from typing import Union, Tuple
# ovKit
import carb
import omni.usd
import omni.kit.app
import omni.kit.editor
import omni.kit.viewport
import omni.kit.pipapi
import omni.kit.asyncapi
from pxr import Usd, UsdGeom, Gf, PhysicsSchema, PhysicsSchemaTools, PhysxSchema
# spidey 
from spidey_python.utils.message import *
from spidey_python.utils.errors import *


def find_environ_variable(name: str) -> Union[str, int, float]:
    """Checks if environment variable exists or not.

    :param name The name of the environment variable to search for.
    :return The value of the environment variable.
    """
    try:
        value = os.environ[name]
    except KeyError:
        msg = f"Environment variable \'{name}\' is not set. Please check project setup."
        print_error(msg)
        raise KeyError(msg)

    return value


# default configurations to load the simulator
OVKIT_DEFAULT_CONFIG = {
    # Applet window parameters
    "width": 1024,
    "height": 800,
    "headless": False,
    # Force PhysX to run on cpu
    "physx_cpu": False,
    "physx_threads": 8,
    # Configuration file for extensions to load
    "config_path": os.path.join(find_environ_variable('ISAAC_PATH'),"experiences/isaac-sim.json")
}






# advanced configuration for the renderer
RENDERER_ADVANCED_CONFIG = {
    "renderer": "PathTracing",
    "samples_per_pixel_per_frame": 64,
    "denoiser": True,
    "subdiv_refinement_level": 2,
    "max_bounces": 4,
    "max_specular_transmission_bounces": 6,
    "max_volume_bounces": 4,
}

# advacned configuration for the phyx
# TODO: enable gpu dynamics should be read from main settings.
PHYSX_ADVANCED_CONFIG = {
    "enable_ccd": True,
    "enable_stablization": True,
    "enable_gpu_dynamics": False,
    "broadphase_type": "MBP",
    "solver_type": "TGS"
}


class OmniKitHelper:
    """Helper class for launching OmniKit from a Python environment.

    Launches and configures OmniKit and exposes useful functions.

        Typical usage example:
        ```
        config = {'width': 800, 'height': 600, 'renderer': 'PathTracing'}
        # Start omniverse kit
        kit = OmniKitHelper(config)
        # <Code to generate or load a scene>
        kit.update()    # Render a single frame
        ```
    """

    def __init__(self, config: dict = None):
        # Load the configuration for kit
        self.config = OVKIT_DEFAULT_CONFIG
        if config is not None:
            self.config.update(config)
        # Check that config file has valid parameters
        if not os.path.exists(self.config['config_path']):
            msg = f"Configuration file: \'{self.config['config_path']}\' does not exist."
            print_error(msg)
            raise FileNotFoundError(msg)

        # Load app plugin
        carb.get_framework().load_plugins(
            loaded_file_wildcards=["omni.kit.app.plugin"], search_paths=["${CARB_APP_PATH}/plugins"]
        )

        # store current time
        self._last_update_t = time.time()
        # acquire app interface
        self._app = omni.kit.app.get_app_interface()
        # register launch applet function
        setup_future = self._launch_kit()
        # register exit function
        self._is_active = False
        atexit.register(self._cleanup)

        # start application
        self._start_app()
        
        # keep running the app while rendering sets up
        while self._app.is_running() and not setup_future.done():
            time.sleep(0.001)  # This sleep prevents a deadlock in certain cases
            self.update()




        # load settings for robotics applications
        self._carb_settings = None
        self._omni_settings = None

        # aquire plugins/extensions interfaces
        self._editor = omni.kit.editor.get_editor_interface()
        self._viewport_interface = omni.kit.viewport.get_viewport_interface()
        # get default camera prim
        self._default_cam_path = self._editor.get_active_camera()
        # get stage conversions to units

        stage = self.get_stage()
        self._meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)

    """
    Internals
    """

    def _start_app(self):
        """
        Start the omniverse-kit application.
        """
        print_info("Starting Application...")
        print_info(f"Loading config file: {self.config['config_path']}")
        # find that environment variables exist
        kit_path = find_environ_variable("KIT_PATH")
        isaac_path = find_environ_variable("ISAAC_PATH")
        carb_app_path = find_environ_variable("CARB_APP_PATH")
        # define arguments to the applet
        args = [
            os.path.abspath(__file__),
            # Configuration JSON file with settings.
            f'--merge-config={self.config["config_path"]}',
            # Set window properties
            '--/app/window/title="Omniverse Kit for Robotics"',
            f'--/app/renderer/resolution/width={self.config["width"]}',
            f'--/app/renderer/resolution/height={self.config["height"]}',
            # This is required due to a infinite loop but results in errors on launch
            '--/app/content/emptyStageOnStart=False',
            # Set path to extensions (adding to json doesn't work)
            f'--/app/extensions/folders2/0="{kit_path}/exts"',
            f'--/app/extensions/folders2/1="{kit_path}/extsPhysics"',
            f'--/app/extensions/folders2/2="{isaac_path}/exts"',
        ]
        # append configuration for running headless
        if self.config.get("headless"):
            args.append("--/app/window/hideUi=true")
            args.append("--no-window")
        
        # run omniverse applet
        self._is_active = True
        self._app.startup("omniverse-kit", carb_app_path, args)


    def _launch_kit(self):
        """ Launches the kit using asynchronous call. It waits while co-routine

        :return: An aysncio Future-like object that runs a Python coroutine. Not thread-safe.
        """

        # Set up the renderer
        async def __setup():
            await omni.kit.asyncapi.new_stage()

            # Acquire settings for simulator
            self._carb_settings = carb.settings.acquire_settings_interface()
            self._omni_settings = omni.kit.settings.get_settings_interface()
            # Set the common settings for the simulator
            self._setup_settings()

        return asyncio.ensure_future(__setup())

    def _setup_settings(self):
        """
        Sets common settings useful for robotic applications
        """
        # world
        self._omni_settings.set("/persistent/app/stage/upAxis", "Z")
        # physics
        self._omni_settings.set("/persistent/physics/updateToUsd", False)
        self._omni_settings.set("/persistent/physics/useFastCache", True)
        # force physx on CPU
        self._omni_settings.set("/persistent/physics/overrideGPUSettings", self.config["physx_cpu"])
        self._omni_settings.set("/persistent/physics/numThreads", self.config['physx_threads'])
        # renderer
        self._omni_settings.set("/app/renderer/gpuSynchronization", False)
        self._omni_settings.set("/rtx/reflections/halfRes", True)
        self._omni_settings.set("/rtx/shadows/denoiser/quarterRes", True)
        self._omni_settings.set("/rtx/translucency/reflectionCutoff", 0.1)

    def _cleanup(self):
        """
        Performs cleanup operations once the application closes.
        """
        if self._is_active:
            self.update()
            self._app.post_quit()
            self._app.shutdown()
            self._is_active = False

    """
    Configurations
    """

    def set_setting(self, setting: str, value: Union[str, bool, int, float]):
        """Convenience function to set settings.

            Example:
            ```
            self.set_setting("/rtx/pathtracing/optixDenoiser/enabled", True)
            self.set_setting("/rtx/rendermode", "PathTracing")
            ```
        :param setting: The name of th setting in the simulator.
        :param value: The value of the setting.
        """
        if isinstance(value, str):
            self._carb_settings.set_string(setting, value)
        elif isinstance(value, bool):
            self._carb_settings.set_bool(setting, value)
        elif isinstance(value, int):
            self._carb_settings.set_int(setting, value)
        elif isinstance(value, float):
            self._carb_settings.set_float(setting, value)
        else:
            msg = f"Value of type {type(value)} is not supported."
            print_error(msg)
            raise OmniverseError(msg)

    def set_active_camera(self, cam_path: str):
        """
        Configure the active viewport to current camera prim.

        @note: The function automatically calls kit.update(). This ensures that the active camera is actually set.

        :param cam_path: The path to the camera prim.
        """
        # change current viewport to current camera
        self._viewport_interface.get_viewport_window().set_active_camera(cam_path)
        # apply viewport change
        for _ in range(2):
            self.update()

    """
    Properties
    """

    @property
    def meters_per_unit(self) -> float:
        """
        :return: The conversion of simulation units to meters
        """
        return self._meters_per_unit

    @property
    def default_camera_path(self) -> str:
        """
        :return: The prim path to the default camera in omniverse kit.
        """
        return self._default_cam_path

    def get_editor(self):
        """
        :return: The editor interface.
        """
        return self._editor

    @staticmethod
    def get_stage() -> Usd.Stage:
        """Returns the current stage.

        :return: The current stage.
        """
        return omni.usd.get_context().get_stage()

    @staticmethod
    def get_dynamic_control_interface():  # -> omni.isaac.dynamic_control._dynamic_control.DynamicControl
        """Returns interface to dynamic control toolbox.

        :return: The dynamic control toolbox interface.
        """
        omni_dc = importlib.import_module(name='omni.isaac.dynamic_control._dynamic_control')
        return omni_dc.acquire_dynamic_control_interface()

    def get_status(self) -> Tuple[float, str, bool, bool]:
        """Get the status of the renderer to see if anything is loading.
        
        :return: The current rendering status of simulator. The status comprises of a tuple which
                 contains (time, message, loaded, loading) status.
        """
        return self._editor.get_current_renderer_status()

    """
    Operations: Setup renderer and phyiscs 
    """

    def setup_advanced_renderer(self, cfg: dict = None):
        """Set advanced settings for renderer (useful for computer vision tasks).

        @note Advancing physics with rendere set to path-tracing causes slowdown.

        :param cfg: Dictionary containing the parameters for the renderer (default: RENDERER_ADVANCED_CONFIG).
        """
        # load default configuration if input is None
        config = RENDERER_ADVANCED_CONFIG
        if cfg is not None:
            config.update(cfg)
        # set settings into simulator
        self.set_setting("/rtx/pathtracing/spp", config["samples_per_pixel_per_frame"])
        self.set_setting("/rtx/pathtracing/totalSpp", config["samples_per_pixel_per_frame"])
        self.set_setting("/rtx/pathtracing/clampSpp", config["samples_per_pixel_per_frame"])
        self.set_setting("/rtx/pathtracing/maxBounces", config["max_bounces"])
        self.set_setting("/rtx/pathtracing/maxSpecularAndTransmissionBounces",
                         config["max_specular_transmission_bounces"])
        self.set_setting("/rtx/pathtracing/maxVolumeBounces", config["max_volume_bounces"])
        self.set_setting("/rtx/pathtracing/optixDenoiser/enabled", config["denoiser"])
        self.set_setting("/rtx/hydra/subdivision/refinementLevel", config["subdiv_refinement_level"])

    def setup_physics(self, prim_path: str = "/World/physics/scene", cfg: dict = None):
        """Creates the physics scene prim in the stage and configures the settings.

        :param prim_path: Path to create physics scene prim at.
        :param cfg: Dictionary containing the parameters for the physics.
        """
        # check valid prim path
        if not os.path.abspath(prim_path):
            msg = f"Input scene physics prim path \'{prim_path}\' is not absolute."
            print_error(msg)
            raise OmniverseError(msg)
        # load default configuration if input is None
        config = PHYSX_ADVANCED_CONFIG
        if cfg is not None:
            config.update(cfg)
        # get stage for the simulator
        stage = self.get_stage()
        # create physics scene
        scene = PhysicsSchema.PhysicsScene.Define(stage, prim_path)
        # set gravity settings
        gravity = Gf.Vec3f(0.0, 0.0, -9.81 / self._meters_per_unit)
        scene.CreateGravityAttr().Set(gravity)
        # apply phyx scene to the stage
        PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(prim_path))
        # get api interface to the physx schema
        physx_scene_api = PhysxSchema.PhysxSceneAPI.Get(stage, prim_path)
        # set settings into physx solver
        physx_scene_api.CreatePhysxSceneEnableCCDAttr(config["enable_ccd"])
        physx_scene_api.CreatePhysxSceneEnableStabilizationAttr(config["enable_stablization"])
        physx_scene_api.CreatePhysxSceneEnableGPUDynamicsAttr(config["enable_gpu_dynamics"])
        physx_scene_api.CreatePhysxSceneBroadphaseTypeAttr(config["broadphase_type"])
        physx_scene_api.CreatePhysxSceneSolverTypeAttr(config["solver_type"])

    """
    Operations: Run the simulator 
    """

    def play(self):
        """Starts the editor physics simulation.
        """
        self.update()
        self._editor.play()
        self.update()

    def update(self, dt: float = 0.0):
        """Render one frame. Optionally specify dt in seconds, specify None to use wallclock.
        
        :param dt: The forward simulation time of physics in the simulator.
        """
        if dt is not None:
            self._app.update(dt)
        else:
            time_now = time.time()
            dt = time_now - self._last_update_t
            self._last_update_t = time_now
            self._app.update(dt)

    def pause(self):
        """Pauses the editor physics simulation.
        """
        self.update()
        self._editor.pause()
        self.update()

    def stop(self):
        """Stops the editor physics simulation"""
        self.update()
        self._editor.stop()
        self.update()

    def close(self):
        """Close the simulator application."""
        self._cleanup()

    @staticmethod
    def execute(*args, **kwargs):
        """Allows usage of kit command interface."""
        omni.kit.commands.execute(*args, **kwargs)

# EOF
