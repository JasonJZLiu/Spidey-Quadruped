"""
@author     Mayank Mittal
@email      mittalma@ethz.ch

@brief      Utility functions for transformations of geometric quantities SE(3).

Reference: https://github.com/StanfordVL/robosuite/blob/master/robosuite/utils/transform_utils.py
"""

import numpy as np

from teleop_python.utils.message import *


def to_camel_case(snake_str: str, to: str = 'cC') -> str:
    """Converts a string from snake case to camel case.

    :param snake_str: A string in snake case (i.e. with '_')
    :param to: A string, either 'CC' or 'cc' determining which convention to convert to.
    :return: A string in camel-case
    """
    # check input is correct
    if to not in ["cC", "CC"]:
        msg = "to_camel_case(): Choose a valid `to` argument (CC or cC)"
        print_error(msg)
        raise Exception(msg)
    # convert string to lower case and split
    components = snake_str.lower().split('_')
    if to == 'cC':
        # We capitalize the first letter of each component except the first one
        # with the 'title' method and join them together.
        return components[0] + ''.join(x.title() for x in components[1:])
    else:
        # Capitalize first letter in all the components
        return ''.join(x.title() for x in components)


def convert_quat(quat: np.ndarray, to: str = "xyzw") -> np.ndarray:
    """
    Converts quaternion from one convention to another.

    The convention to convert TO is specified as an optional argument. If to == 'xyzw', then the input is in
    'wxyz' format, and vice-versa.

    :param quat: A numpy array corresponding to a quaternion of shape (4,) or (4, 1)
    :param to: A string, either 'xyzw' or 'wxyz', determining which convention to convert to.
    :return A numpy array corresponding to the converted quaternion.
    """
    # check input is correct
    if to not in ["xyzw", "wxyz"]:
        msg = "convert_quat(): Choose a valid `to` argument (xyzw or wxyz)"
        print_error(msg)
        raise Exception(msg)
    if quat.shape != (4,) and quat.shape != (4, 1):
        msg = f"convert_quat(): Expected input quaternion shape mismatch: {quat.shape} != (4,) or (4, 1)"
        print_error(msg)
        raise Exception(msg)
    # squeeze input to remove excess dimensions
    quat_shape = quat.shape
    quat = quat.squeeze()
    # convert to specified quaternion type
    if to == "xyzw":
        return quat[[1, 2, 3, 0]].reshape(quat_shape)
    if to == "wxyz":
        return quat[[3, 0, 1, 2]].reshape(quat_shape)

# EOF
