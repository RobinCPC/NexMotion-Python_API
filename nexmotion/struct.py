"""Structure Type related to motion"""
from nexmotion.constants import MAX_POS_SIZE, MAX_POSE_DATA_SIZE, MAX_XYZ_SIZE

from sys import platform
if any([platform.startswith(os_name) for os_name in ['linux', 'darwin', 'freebsd']]):
    try:
        import zugbruecke as ctypes
    except ImportError:
        raise ImportError("Need zugbruecke to use nexmtion dll lib in Linux/Darwin os system!")
elif platform.startswith('win'):
    import ctypes
else:
    # Handle unsupported plafroms
    print('NexMotion lib is used in unsupported platform.\n Necessary package may not be installed!')
#from ctypes import c_double, Structure

SIZE_ARRAY_DOUBLE = ctypes.c_double * MAX_POS_SIZE
SIZE_XYZ_DOUBLE = ctypes.c_double * MAX_XYZ_SIZE
SIZE_COORD_DOUBLE = ctypes.c_double * MAX_POSE_DATA_SIZE

class Pos_T(ctypes.Structure):
    _fields_ = [("pos", SIZE_ARRAY_DOUBLE)]

class Xyz_T(ctypes.Structure):
    _fields_ = [("pos", SIZE_XYZ_DOUBLE)]

class CoordTrans_T(ctypes.Structure):
    _fields_ = [("pose", SIZE_COORD_DOUBLE)]

