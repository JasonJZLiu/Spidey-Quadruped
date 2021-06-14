"""
@author     Mayank Mittal, Jingzhou Liu
@email      mittalma@ethz.ch, jingzhou.liu@mail.utoronto.ca
"""
from enum import Enum

class SpideyDim(Enum):
	"""
    @brief Defines the dimensions of the states for the robot.
    """
	
    # Dynamics state
	SpideyJointsDim = 12
	GeneralizedCoordinatesDim = SpideyJointsDim
	GeneralizedVelocitiesDim = SpideyJointsDim

 	# dq_j_spidey
	SpideyInputDim = 12


#EOF
