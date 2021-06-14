#!/usr/bin/python
"""
Tests Omniverse-Kit helper kit.
"""



from spidey_python.omniverse import OmniKitHelper, create_prim
from spidey_python.utils.message import *
# Omniverse
from pxr import UsdGeom


def test_basic():
    """
    Basic example usage of the omniverse application toolkit. The test lauches the simulator, creates a cube
    primitive into the world and then closes the simulator.
    """
    # Example usage
    print_notify("Running app ...")
    # Load kit helper
    kit = OmniKitHelper()
    # Acquire current stage of simulation
    stage = kit.get_stage()
    # Assert that stage exists
    assert stage is not None
    # Add cube geometry
    cube = create_prim(stage, "/World/Cube", "Cube", scale=(100, 100, 100))
    # Check if properties can be set externally
    UsdGeom.XformCommonAPI(cube).SetScale([25, 25, 25])
    # Simulate physics
    for _ in range(2000):
        kit.update(dt=0.01)
    # Close the application
    kit.close()
    # Shutdown
    print_notify("Exiting the program...")    


if __name__ == "__main__":
    # basic test for kit
    test_basic()

# EOF
