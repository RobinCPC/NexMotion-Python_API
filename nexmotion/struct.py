"""Structure Type related to motion"""
from nexmotion.constants import MAX_POS_SIZE
from ctypes import c_double, Structure

SIZE_ARRAY_DOUBLE = c_double * MAX_POS_SIZE

class Pos_T(Structure):
    _fields_ = [("pos", SIZE_ARRAY_DOUBLE)]
