"""Structure Type related to motion"""
from nexmotion.constants import MAX_POS_SIZE, MAX_POSE_DATA_SIZE, MAX_XYZ_SIZE
from ctypes import c_double, Structure

SIZE_ARRAY_DOUBLE = c_double * MAX_POS_SIZE
SIZE_XYZ_DOUBLE = c_double * MAX_XYZ_SIZE
SIZE_COORD_DOUBLE = c_double * MAX_POSE_DATA_SIZE

class Pos_T(Structure):
    _fields_ = [("pos", SIZE_ARRAY_DOUBLE)]

class Xyz_T(Structure):
    _fields_ = [("pos", SIZE_XYZ_DOUBLE)]

class CoordTrans_T(Structure):
    _fields_ = [("pose", SIZE_COORD_DOUBLE)]

