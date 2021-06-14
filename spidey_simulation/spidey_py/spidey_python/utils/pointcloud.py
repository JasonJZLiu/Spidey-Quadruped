import numpy as np
import torch
from typing import Optional, Union, Tuple


def create_pointcloud_from_rgbd(intrinsic_matrix: np.ndarray, depth: np.ndarray,
                                rgb: Optional[Union[np.ndarray, Tuple[float, float, float]]] = None) \
        -> Tuple[np.ndarray, np.ndarray]:
    """Creates pointcloud from input depth image and camera transformation matrix.

    :param intrinsic_matrix: A (3, 3) numpy array providing camera's calibration matrix.
    :param depth: A numpy array of shape (H, W) with values encoding the depth measurement.
    :param rgb: Color corresponsing to the generated point cloud.
                - If a numpy array of shape (H, W, 3) then the corresponding channels encode RGB values.
                - If a tuple then the point cloud has a single color specified by the values (r, g, b).
                - If None, then default color is white, i.e. (0, 0, 0).
    :return: A tuple comprising of two numpy array of shape (N, 3) comprising of 3D coordinates of points and
             their corresponding RGB color.
    """
    # device for carrying out conversion
    device = 'cpu'
    # check shape of inputs
    assert np.asarray(intrinsic_matrix).shape == (3, 3)
    assert len(np.asarray(depth).shape) == 2
    # convert inputs to numpy arrays
    intrinsic_matrix = torch.from_numpy(intrinsic_matrix).to(device)
    depth = torch.from_numpy(depth).to(device)
    # get image height and width
    im_width, im_height = depth.shape
    # total number of points
    num_points = im_height * im_width
    # convert image points into list of shape (3, H x W)
    img_indices = np.indices((im_width, im_height))
    pixels = np.stack([img_indices[0].ravel(), img_indices[1].ravel(), np.ones(im_height * im_width)], axis=0)
    pixels = torch.tensor(pixels, device=device)
    # convert into 3D points
    points = torch.matmul(torch.inverse(intrinsic_matrix), pixels)
    points = points / points[-1, :]
    points_xyz = points * depth.view(-1)
    # convert it to (H x W , 3)
    points_xyz = torch.transpose(points_xyz, dim0=0, dim1=1)
    points_xyz = points_xyz.detach().cpu().numpy()

    # extract color value
    if isinstance(rgb, np.ndarray):
        assert rgb.shape == (im_width, im_height, 3)
        # get rgb values
        points_rgb = np.asarray(rgb, dtype=np.uint8).reshape(-1, 3)
    elif isinstance(rgb, Tuple):
        points_rgb = np.asarray((rgb,) * num_points, dtype=np.uint8)
    else:
        points_rgb = np.asarray(((0, 0, 0),) * num_points, dtype=np.uint8)

    return points_xyz, points_rgb

# EOF
