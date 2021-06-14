"""
@author     Mayank Mittal
@email      mittalma@ethz.ch

@brief      Defines methods to read/write point cloud data as PLY format.

"""

import os
from typing import Optional, Tuple
import numpy as np
import plyfile as ply


def read_ply(filename: str) -> Tuple[np.ndarray, np.ndarray]:
    """Read a PLY file 
    
    :param filename: Path to the file to read PLY from.
    :return: A tuple of numpy arrays comprising of two point cloud of shape (N, 3)
             with location and color of each point. Here `N`: number of points.
    """
    # check input is valid
    assert os.path.exists(filename)
    # Read the file
    plydata = ply.PlyData.read(filename)
    pc = plydata['vertex'].data
    pc_array = np.asarray(pc)

    return pc_array[:, :3], pc_array[:, 3:]


def write_ply(filename: str, points_xyz: np.ndarray, points_rgb: np.ndarray, text: Optional[bool] = True):
    """ Writes an input pointcloud into file of PLY format.

    :param filename: The name of the file to save data to.
    :param points_xyz: A numpy array of shape (N, 3) containing the pointcloud
    :param points_rgb: A numpy array of shape (N, 3) containing the RGB color corresponding to point cloud.
    :param text: Whether to save file in ASCII format or not.
    """
    # append ply specification
    if not filename.endswith('ply'):
        filename += '.ply'
    # create directory to the file if they don't exist
    if os.path.exists(os.path.dirname(filename)):
        os.makedirs(os.path.dirname(filename), exist_ok=True)
    # check valid input
    assert points_xyz.shape == points_rgb.shape
    assert len(points_xyz.shape) == 2
    # concatenate points and color
    points = np.concatenate([points_xyz, points_rgb], axis=1)
    # custom dtype for PLY format ('name', 'field')
    ply_dtype = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                          ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    # convert into PLY vertex format
    vertex = np.array(list(map(tuple, points)), dtype=ply_dtype)
    el = ply.PlyElement.describe(vertex, 'vertex', comments=['vertices'])
    # write into PLY file
    ply.PlyData([el], text=text).write(filename)

# EOF
