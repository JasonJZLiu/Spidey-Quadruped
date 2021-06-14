"""
@author     Mayank Mittal
@email      mittalma@ethz.ch

@brief      Defines methods to read/write images as PFM format.

Reference: https://github.com/microsoft/AirSim/blob/master/PythonClient/airsim/utils.py
"""

# python
import re
import sys
import numpy as np
from typing import Tuple


def read_pfm(filename: str) -> Tuple[np.ndarray, float]:
    """ Read a pfm file

    :param filename: Path to the file to read PFM from.
    :return: Tuple comprising of the read image and the scaling used.
    """
    file = open(filename, 'rb')

    # define properties of the image
    color = None
    width = None
    height = None
    scale = None
    endian = None

    header = file.readline().rstrip()
    header = str(bytes.decode(header, encoding='utf-8'))
    if header == 'PF':
        color = True
    elif header == 'Pf':
        color = False
    else:
        raise Exception('Not a PFM file.')

    pattern = r'^(\d+)\s(\d+)\s$'
    temp_str = str(bytes.decode(file.readline(), encoding='utf-8'))
    dim_match = re.match(pattern, temp_str)
    if dim_match:
        width, height = map(int, dim_match.groups())
    else:
        temp_str += str(bytes.decode(file.readline(), encoding='utf-8'))
        dim_match = re.match(pattern, temp_str)
        if dim_match:
            width, height = map(int, dim_match.groups())
        else:
            raise Exception('Malformed PFM header: width, height cannot be found')

    scale = float(file.readline().rstrip())
    if scale < 0:  # little-endian
        endian = '<'
        scale = -scale
    else:
        endian = '>'  # big-endian

    data = np.fromfile(file, endian + 'f')
    shape = (height, width, 3) if color else (height, width)

    data = np.reshape(data, shape)
    # DEY: I don't know why this was there.
    file.close()

    return data, scale


def write_pfm(filename: str, image: np.ndarray, scale: float = 1):
    """ Write a pfm file.

    :param filename: Path to the file to save PFM at.
    :param image: Input numpy array of dtype float32 and shape H x W x channels
    :param scale: Scaling for the saved image.
    """
    file = open(filename, 'wb')

    color = None

    if image.dtype.name != 'float32':
        raise Exception('Image dtype must be float32.')

    if len(image.shape) == 3 and image.shape[2] == 3:  # color image
        color = True
    elif len(image.shape) == 2 or len(image.shape) == 3 and image.shape[2] == 1:  # greyscale
        color = False
    else:
        raise Exception('Image must have H x W x 3, H x W x 1 or H x W dimensions.')

    file.write(bytes('PF\n', 'UTF-8') if color else bytes('Pf\n', 'UTF-8'))
    temp_str = '%d %d\n' % (image.shape[1], image.shape[0])
    file.write(bytes(temp_str, 'UTF-8'))

    endian = image.dtype.byteorder

    if endian == '<' or endian == '=' and sys.byteorder == 'little':
        scale = -scale

    temp_str = '%f\n' % scale
    file.write(bytes(temp_str, 'UTF-8'))

    image.tofile(file)


def write_png(filename: str, image: np.ndarray):
    """ Write a png file.

    :param filename: Path to the file to save PFM at.
    :param image: Input numpy array of shape H x W x channels
    """
    import zlib
    import struct

    buf = image.flatten().tobytes()
    width = image.shape[1]
    height = image.shape[0]

    # reverse the vertical line order and add null bytes at the start
    width_byte_3 = width * 3
    raw_data = b''.join(b'\x00' + buf[span:span + width_byte_3]
                        for span in range((height - 1) * width_byte_3, -1, - width_byte_3))

    def png_pack(png_tag, data):
        """Helper to write data in png format."""
        chunk_head = png_tag + data
        return (struct.pack("!I", len(data)) +
                chunk_head +
                struct.pack("!I", 0xFFFFFFFF & zlib.crc32(chunk_head)))

    png_bytes = b''.join([
        b'\x89PNG\r\n\x1a\n',
        png_pack(b'IHDR', struct.pack("!2I5B", width, height, 8, 6, 0, 0, 0)),
        png_pack(b'IDAT', zlib.compress(raw_data, 9)),
        png_pack(b'IEND', b'')])

    with open(filename, 'wb') as afile:
        afile.write(png_bytes)

# EOF
