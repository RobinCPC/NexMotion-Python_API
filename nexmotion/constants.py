"""The constants related to NexMotion Library"""

# Device Type Selection
DEVICE_TYPE_SIMULATOR = 0
DEVICE_TYPE_ETHERCAT  = 1

# Device State
DEVICE_STATE_INIT = 1
DEVICE_STATE_READY = 2
DEVICE_STATE_ERROR = 3
DEVICE_STATE_OPERATION = 4

# Group State
NMC_GROUP_STATE_DISABLE     = 0 # Disable: The group servo is OFF.
NMC_GROUP_STATE_STAND_STILL = 1 # Enable: The group servo is stand still.
NMC_GROUP_STATE_STOPPED     = 2 # Receiving the Stop command and stopped
NMC_GROUP_STATE_STOPPING    = 3 # Receiving the Stop command and slowing down to stop.
NMC_GROUP_STATE_MOVING      = 4 # Executing the move command
NMC_GROUP_STATE_HOMING      = 5 # Executing the Homing motion.
NMC_GROUP_STATE_ERROR       = 6 # Stop for error.

# Group Coordinate Number Mask
GROUP_AXIS_MASK_X = 0x00000001 # The mask for X axis of the coordinate system
GROUP_AXIS_MASK_Y = 0x00000002 # The mask for Y axis of the coordinate system
GROUP_AXIS_MASK_Z = 0x00000004 # The mask for Z axis of the coordinate system
GROUP_AXIS_MASK_A = 0x00000008 # The mask for A axis of the coordinate system
GROUP_AXIS_MASK_B = 0x00000010 # The mask for B axis of the coordinate system
GROUP_AXIS_MASK_C = 0x00000020 # The mask for C axis of the coordinate system
GROUP_AXIS_MASK_U = 0x00000040 # The mask for U axis of the coordinate system
GROUP_AXIS_MASK_V = 0x00000080 # The mask for V axis of the coordinate system

# Bit Code of Group State
GROUP_STATUS_EMG  = 0  # A latched signal is issued at the external EMG signal (*1).
GROUP_STATUS_ALM  = 1  # A latched signal is issued at a group axis servo Alarm (*1)
GROUP_STATUS_PEL  = 2  # A latched signal is issued at the positive limit signal of a group axis (*1)
GROUP_STATUS_NEL  = 3  # A latched signal is issued at the negative limit signal of a group axis (*1)
GROUP_STATUS_PSEL = 4  # A latched signal is issued at the software positive limit signal a group axis (*1)
GROUP_STATUS_NSEL = 5  # A latched signal is issued at the software negative limit signal a group axis (*1)
GROUP_STATUS_ENA  = 6  # The group is Enable or Disable.
GROUP_STATUS_ERR  = 7  # Group (a group axis) error
GROUP_STATUS_CSTP = 9  # No displacement of all group axes (i.e. the displacement is 0).
GROUP_STATUS_ACC  = 10 # Moving in the Cartesian coordinate system (along a line or an arc) and accelerating to the maximum velocity or decelerating. The value is 0 for PTP or JOG motion.
GROUP_STATUS_DEC  = 11 # Moving in the Cartesian coordinate system (along a line or an arc) and decelerating to the target position or STOP. The value is 0 for PTP or JOG motion.
GROUP_STATUS_MV   = 12 # Moving in the Cartesian coordinate system (along a line or an arc) at the maximum velocity. The value is 0 for PTP or JOG motion.
GROUP_STATUS_OP   = 13 # The group is moving. That is the state is GROUP_MOVING, GROUP_HOMING or GROUP_STOPPING.
GROUP_STATUS_STOP = 14 # The group is STOP. That is the state is GROUP_STOPPED.

# Axis State
AXIS_STATE_DISABLE           = 0
AXIS_STATE_STAND_STILL       = 1
AXIS_STATE_HOMING            = 2
AXIS_STATE_DISCRETE_MOTION   = 3
AXIS_STATE_CONTINUOUS_MOTION = 4
AXIS_STATE_STOPPING          = 5
AXIS_STATE_STOPPED           = 6
AXIS_STATE_WAIT_SYNC         = 7
AXIS_STATE_GROUP_MOTION      = 8
AXIS_STATE_ERROR             = 10

# Group axis number definition
MAX_AXES_IN_GROUP = 8

# Pos_T array size definition
MAX_POS_SIZE = 8

# Xyz_T array size definition
MAX_XYZ_SIZE = 3

# APos_T array size definition
MAX_AXIS_POS_SIZE = MAX_AXES_IN_GROUP

# CPos_T array size definition
MAX_CARTESIAN_POS_SIZE = 6

# CoordTrans_T array size definition
MAX_POSE_DATA_SIZE = 6
