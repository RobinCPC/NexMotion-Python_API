"""NexMotionControl module"""

# import modules
from .api import Control
from .struct import Pos_T
from .constants import *
from .errors import *

__all__ = ['Control', 'Pos_T']
