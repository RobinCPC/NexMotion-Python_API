# -*- coding: utf-8 -*-
"""The API of NexMotion Library"""
import ctypes
from ctypes import WinDLL, c_int32, c_uint32, c_uint16, c_char_p, c_double, byref
import numpy as np
import csv, time, copy

from nexmotion.constants import *
from nexmotion.struct import Pos_T, CoordTrans_T
from nexmotion.errors import *


class Control(object):
    """
    A class to wrap NexMotion library
    """

    def __init__(self, dll_path="C:\\Windows\\System32\\NexMotion.dll"):
        self.dll_ = WinDLL(dll_path)
        self.id_ = c_int32(0)
        self.index_ = c_int32(0)
        self.type_ = c_int32(DEVICE_TYPE_SIMULATOR)
        self.devState_ = c_int32(0)
        self.groupState_ = c_int32(0)
        self.groupVel_ = c_double(0)
        self.numGroup_ = c_int32(0)
        self.numGroupAxis_ = c_int32(0)
        self.actPos_ = Pos_T()
        self.desPos_ = Pos_T()
        self.refBasePntArr_ = [Pos_T() for i in range(3)]
        self.refBaseCoordTrans_ = CoordTrans_T()
        self.offsetByte_ = c_uint32(0)
        self.sizeByte_ = c_uint32(2)     # total two bytes (16 bits)
        self.doValue_ = c_uint16(0)
        self.diValue_ = c_uint16(0)
        self.pnt_list = []              # a list to store poses as list of 12 joint
        # Get DLL version
        major = c_int32(0)
        minor = c_int32(0)
        stage = c_int32(0)
        build = c_int32(0)
        self.version_ = self.dll_.NMC_GetLibVersion(byref(major), byref(minor), byref(stage), byref(build))
        print( "Dynamics library version =", self.version_, "(", major.value, ",", minor.value, \
            ",", stage.value, ",", build.value, ")")

    def deviceOpenup(self, type_=None, idx_=None):
        """
        Open up the device (Blocking call).

        :param type_: The specified type. 0: Simulation, 1: EtherCAT
        :type  type_: int
        :param idx_:  The specified index of device which is set to 0.
        :type  idx_:  int
        :return: error code
        :rtype: int
        """
        if type_ is not None:
            self.type_ = c_int32(type_)

        if idx_ is not None:
            self.index_ = c_int32(idx_)
        ret = self.dll_.NMC_DeviceOpenUp(self.type_, self.index_, byref(self.id_))
        if ret == SUCCESS:
            print( "device id =", self.id_.value)
        return ret

    def deviceShutdown(self, id_=None):
        """
        Shutdown the device (Blocking call).

        :param id_: Device ID (DevID)
        :type id_: int (option)
        :return: error code
        :rtype: int
        """
        if id_ is not None:
            self.id_ = c_int32(id_)
        return self.dll_.NMC_DeviceShutdown(self.id_)

    def deviceGetState(self):
        """
        Get device state

        :return: error code
        :type: int
        """
        return self.dll_.NMC_DeviceGetState(self.id_, byref(self.devState_))

    def deviceGetGroupCount(self):
        """
        Get the quantity of group

        :return: error code
        :rtype: int
        """
        return self.dll_.NMC_DeviceGetGroupCount(self.id_, byref(self.numGroup_))

    def deviceGetGroupAxisCount(self):
        """
        Get the quantity of group axis

        :return: error code
        :rtype: int
        """
        return self.dll_.NMC_DeviceGetGroupAxisCount(self.id_, self.index_, byref(self.numGroupAxis_))

    def setIniPath(self, ini_path):
        """
        Set the path of ini (configuration) file.

        :param ini_path: The ini file path to be set. Can be set to NULL(0) to reset to the default path.
        :type ini_path: str
        :return: error code
        :rtype: int
        """
        self.ini_path_ptr_ = c_char_p(ini_path)
        return self.dll_.NMC_SetIniPath(self.ini_path_ptr_)

    def writeOutputMemory(self, DO_list):
        """
        Write the mapped Output memory
        TODO: Need modify to remove second list element.

        :param DO_list: a List of two array show which DO should on or off [ [], [] ]
        :type DO_list: List( List, List)
        :return: error code
        :rtype: int
        """
        if len(DO_list) != 2:
            print( "DO_list should be a list of two list")
            return
        tmp_value = 0
        for i in DO_list[0]:
            tmp_value |= (1 << i)
        for i in DO_list[1]:
            tmp_value &= (~(1 << i))
        self.doValue_.value = tmp_value
        return self.dll_.NMC_WriteOutputMemory(self.id_, self.offsetByte_, self.sizeByte_, byref(self.doValue_))

    def readOutputMemory(self):
        """
        Read the mapped Output memory

        :return: error code
        :rtype: int
        """
        return self.dll_.NMC_ReadOutputMemory(self.id_, self.offsetByte_, self.sizeByte_, byref(self.doValue_))

    def readInputMemory(self):
        """
        Read the mapped Input memory

        :return: error code
        :rtype: int
        """
        return self.dll_.NMC_ReadInputMemory(self.id_, self.offsetByte_, self.sizeByte_, byref(self.diValue_))

    def groupResetDriveAlmAll(self):
        """
        Rest all group servo alarms.

        :return: error code
        :rtype: int
        """
        return self.dll_.NMC_GroupResetDriveAlmAll(self.id_, self.index_)

    def groupGetState(self):
        """
        Get the state of group

        :return: error code
        :rtype: int
        """
        return self.dll_.NMC_GroupGetState(self.id_, self.index_, byref(self.groupState_))

    def groupResetState(self):
        """
        Reset the group state . It can reset the group state from GROUP_STOPPED to GROUP_STAND_STILL . If
        the group state is GROUP_ERROR_STOP and the API is called, all drive alarms will be reset automatically.
        Then the group state will transfer to GROUP_STAND_STILL after all drive alarms are reset.

        :return: error code
        :rtype: int
        """
        return self.dll_.NMC_GroupResetState(self.id_, self.index_)

    def groupEnable(self):
        """
        Enable all group axes (Servo On).

        :return: error code
        :rtype: int
        """
        return self.dll_.NMC_GroupEnable(self.id_, self.index_)

    def groupDisable(self):
        """
        Disable all group axes (Servo off).

        :return: error code
        :rtype: int
        """
        return self.dll_.NMC_GroupDisable(self.id_, self.index_)

    def groupSetVelRatio(self, ratio):
        """
        Set the velocity percentage of a group from 0.0 to 100.0%.

        :param ratio: The velocity percentage will be set.
        :type pos: float
        :return: error code
        :rtype: int
        """
        ret = self.dll_.NMC_GroupSetVelRatio(self.id_, self.index_, c_double(ratio))
        if ret == SUCCESS:
            self.groupVel_.value = ratio
        else:
            print( "Set group velocity failed!")
        return ret

    def groupGetVelRatio(self, ratio):
        """
        Get the velocity percentage of a group.
        TODO: put ratio to return argument.

        :param ratio: variable to storage the current velocity percentage.
        :type pos: float
        :return: error code
        :rtype: int
        """
        ret = self.dll_.NMC_GroupGetVelRatio(self.id_, self.index_, byref(self.groupVel_))
        if ret == SUCCESS:
            print( self.groupVel_.value)
            ratio = self.groupVel_.value
        else:
            print( "Get group velocity failed!")
        return ret

    def groupGetActualPosAcs(self, pos):
        """
        Get the actual position of a group in the axis coordinate system (ACS).

        :param pos: a list of 6 element. use it (as return param) to get joint values
        :type pos: list
        :return: error code
        :rtype: int
        """
        ret = self.dll_.NMC_GroupGetActualPosAcs(self.id_, self.index_, byref(self.actPos_))
        for i in range(len(pos)):
            pos[i] = self.actPos_.pos[i]
        return ret

    def groupGetActualPosPcs(self, pos):
        """
        Get the actual position of a group in Cartesian Coordinate System (PCS).

        :param pos: a list of 6 element. use it (as return param) to get Cartesian values
        :type pos: list
        :return: error code
        :rtype: int
        """
        ret = self.dll_.NMC_GroupGetActualPosPcs(self.id_, self.index_, byref(self.actPos_))
        for i in range(len(pos)):
            pos[i] = self.actPos_.pos[i]
        return ret

    def groupAxesHomeDrive(self, mask=None):
        """
        Do Homing by Driver.
        Note: this command only work for DEVICE_TYPE_ETHERCAT (control real robot)

        :param mask: a list/set of joint number (0 - 5) ex. mask = [0,2,4] # will do homing for joint 1, 3, 5
        :type mask: List
        :return: error code
        :rtype: int
        """
        if mask is None:
            mask = []
        mask_set = [GROUP_AXIS_MASK_X, GROUP_AXIS_MASK_Y, GROUP_AXIS_MASK_Z,
                     GROUP_AXIS_MASK_A, GROUP_AXIS_MASK_B, GROUP_AXIS_MASK_C]
        mask_sum = 0
        for m in mask:
            mask_sum += mask_set[m]
        return self.dll_.NMC_GroupAxesHomeDrive(self.id_, self.index_, c_int32(mask_sum))

    def groupSetHomePose(self, mask=None, setPose=None):
        """
        Do Homing Manually.
        Note: this command may record joint offset value.

        :param mask: a list/set of joint number (0 - 5) ex. mask = [0,2,4] # will do homing for joint 1, 3, 5
        :type mask: List
        :param setPose: a list/set of joint value that assign to joints according to mask ex. mask = [0,2,4], setPose = [0, 20, -90]. will set 0 deg to joint 1, 20 deg to joint 3, and -90 deg to joint 5.
        :type mask: List
        :return: error code
        :rtype: int
        """
        if setPose == None or mask == None:
            print( "arg mask and setPose need to provide")
        if not isinstance(setPose, list) or not isinstance(mask, list):
            print( "arg mask and setPose both should be list")
        if len(setPose) != len(mask):
            print( "arg mask and setPose should have the same number of element.")
        homePose = Pos_T()
        mask_set = [GROUP_AXIS_MASK_X, GROUP_AXIS_MASK_Y, GROUP_AXIS_MASK_Z,
                     GROUP_AXIS_MASK_A, GROUP_AXIS_MASK_B, GROUP_AXIS_MASK_C]
        mask_sum = 0
        for m, p in zip(mask, setPose):
            mask_sum += mask_set[m]
            homePose.pos[m] = p
        return self.dll_.NMC_GroupSetHomePose(self.id_, self.index_, c_int32(mask_sum), byref(homePose))

    def groupPtpAcsAll(self, desPos):
        """
        Do PTP motion

        :param desPos: a List of 6 element [j1 - j6]
        :type desPos: List
        :return: error code
        :rtype: int
        """
        for i in range(len(desPos)):
            self.desPos_.pos[i] = desPos[i]
        mask = 0
        mask += GROUP_AXIS_MASK_X
        mask += GROUP_AXIS_MASK_Y
        mask += GROUP_AXIS_MASK_Z
        mask += GROUP_AXIS_MASK_A
        mask += GROUP_AXIS_MASK_B
        mask += GROUP_AXIS_MASK_C
        return self.dll_.NMC_GroupPtpAcsAll(self.id_, self.index_, c_int32(mask), byref(self.desPos_))

    def groupLine(self, desPos, maxVel=0):
        """
        Enable the group line interpolation motion from the current position to the target position in
        the Cartesian space.

        :param desPos: a List of 6 element [x, y, z, roll, pitch, yaw]
        :param maxVel: set the limit of max. velocity. input 0 to ignore the parameter
        :return: error code
        :rtype: int
        """
        mask = 0
        mask += GROUP_AXIS_MASK_X
        mask += GROUP_AXIS_MASK_Y
        mask += GROUP_AXIS_MASK_Z
        mask += GROUP_AXIS_MASK_A
        mask += GROUP_AXIS_MASK_B
        mask += GROUP_AXIS_MASK_C
        for i in range(len(desPos)):
            self.desPos_.pos[i] = desPos[i]
        return self.dll_.NMC_GroupLine(self.id_, self.index_, c_int32(mask), byref(self.desPos_), maxVel)

    def groupHalt(self):
        """
        Halt a group (to Stand still state).

        :return: error code
        :rtype: int
        """
        return self.dll_.NMC_GroupHalt(self.id_, self.index_)

    def group3DShow(self, top_=True):
        """
        Create or display a 3D simulation window

        :param top_:
        :type top_: bool
        :return: error code
        :rtype: int
        """
        ret = self.dll_.NMC_Group3DShow(self.id_, self.index_)
        if top_:
            ret = self.dll_.NMC_Group3DAlwaysTop(self.id_, self.index_, True)
        return ret

    def group3DDrawPath(self, enable=True):
        """
        Draw the path of tcp when robot is moving

        :param enable: if enable is true, show the path of robot tcp.
        :type enable: bool
        :return: error code
        :rtype: int
        """
        return self.dll_.NMC_Group3DDrawPath(self.id_, self.index_, enable)

    def baseCalib_1p(self, baseP1, baseCoordTrans):
        """
        Base teaching - 1 point method

        :param baseP1: The cartesian pose for the first step. a List of 6 element [x, y, z, roll, pitch, yaw]
        :type baseP1: list
        :param baseCoordTrans: Return the relationship respected to reference coordinate convention
        :type baseCoordTrans: list
        :return: error code
        :rtype: int
        """
        for i in range(len(baseP1)):
            self.refBasePntArr_[0].pos[i] = baseP1[i]
        ret = self.dll_.NMC_BaseCalib_1p(byref(self.refBasePntArr_[0]), byref(self.refBaseCoordTrans_))
        if ret != SUCCESS:
            print( "Failed to compute base coordinate convention!")
            return ret
        for i in range(len(baseCoordTrans)):
            baseCoordTrans[i] = self.refBaseCoordTrans_.pose[i]
        return ret

    def baseCalib_2p(self, baseP1, baseP2, baseCoordTrans):
        """
        Base teaching - 2 points method

        :param baseP1: The cartesian pose for the first step. a List of 6 element [x, y, z, roll, pitch, yaw]
        :type baseP1: list
        :param baseP2: The cartesian pose for the second step. a List of 6 element [x, y, z, roll, pitch, yaw]
        :type baseP2: list
        :param baseCoordTrans: Return the relationship respected to reference coordinate convention
        :type baseCoordTrans: list
        :return: error code
        :rtype: int
        """
        for i in range(len(baseP1)):
            self.refBasePntArr_[0].pos[i] = baseP1[i]
            self.refBasePntArr_[1].pos[i] = baseP2[i]
        ret = self.dll_.NMC_BaseCalib_2p(byref(self.refBasePntArr_[0]), byref(self.refBasePntArr_[1]), byref(self.refBaseCoordTrans_))
        if ret != SUCCESS:
            print( "Failed to compute base coordinate convention!")
            return ret
        for i in range(len(baseCoordTrans)):
            baseCoordTrans[i] = self.refBaseCoordTrans_.pose[i]
        return ret

    def baseCalib_3p(self, baseP1, baseP2, baseP3, baseCoordTrans):
        """
        Base teaching - 2 points method

        :param baseP1: The cartesian pose for the first step. a List of 6 element [x, y, z, roll, pitch, yaw]
        :type baseP1: list
        :param baseP2: The cartesian pose for the second step. a List of 6 element [x, y, z, roll, pitch, yaw]
        :type baseP2: list
        :param baseP3: The cartesian pose for the third step. a List of 6 element [x, y, z, roll, pitch, yaw]
        :type baseP3: list
        :param baseCoordTrans: Return the relationship respected to reference coordinate convention
        :type baseCoordTrans: list
        :return: error code
        :rtype: int
        """
        for i in range(len(baseP1)):
            self.refBasePntArr_[0].pos[i] = baseP1[i]
            self.refBasePntArr_[1].pos[i] = baseP2[i]
            self.refBasePntArr_[2].pos[i] = baseP3[i]
        ret = self.dll_.NMC_BaseCalib_3p(byref(self.refBasePntArr_[0]), byref(self.refBasePntArr_[1]), byref(self.refBasePntArr_[2]),
                                         byref(self.refBaseCoordTrans_))
        if ret != SUCCESS:
            print( "Failed to compute base coordinate convention!")
            return ret
        for i in range(len(baseCoordTrans)):
            baseCoordTrans[i] = self.refBaseCoordTrans_.pose[i]
        return ret

    def movePTP(self, index=None):
        """
        Move PTP to the target pose.
        Note: check group state will block function till joints arrive, so groupHalt/groupStop can not interrupt.

        :param index: indicate the position of target pose in the point list
        :return: error code
        :rtype: int
        """
        if index is None:
            return -43       # TODO: find other suitable error code
        ret = -42
        if isinstance(index, int) and index < len(self.pnt_list):
            ret = self.groupPtpAcsAll(self.pnt_list[index][:6])
        if ret != SUCCESS:
            print( "Failed to execute groupPtpAcsAll!\n")
            return ret
        # Check if joints arrive command position
        ret = self.groupGetState()
        while ret == SUCCESS and self.groupState_.value == NMC_GROUP_STATE_MOVING:
            time.sleep(0.1)
            ret = self.groupGetState()
        return ret

    def moveLine(self, index=None):
        """
        Move Line to the target pose.
        Note: check group state will block function till joints arrive, so groupHalt/groupStop can not interrupt.

        :param index: indicate the position of target pose in the point list
        :return: error code
        :rtype: int
        """
        if index is None:
            return -43       # TODO: find other suitable error code
        ret = -42
        if isinstance(index, int) and index < len(self.pnt_list):
            ret = self.groupLine(self.pnt_list[index][6:])
        if ret != SUCCESS:
            print( "Failed to execute groupPtpAcsAll!\n")
            return ret
        # Check if joints arrive command position
        ret = self.groupGetState()
        while ret == SUCCESS and self.groupState_.value == NMC_GROUP_STATE_MOVING:
            time.sleep(0.1)
            ret = self.groupGetState()
        return ret

    def recordPoint(self):
        """
        Store current pose (joint, tcp) as a List of 12 element into pnt_list

        :return: error code
        :rtype: int
        """
        jntPos = [0.] *6
        cartPos = [0.] * 6
        ret = self.groupGetActualPosAcs(jntPos)
        if ret != SUCCESS:
            print( "Failed to get joint pose!")
            return ret
        self.groupGetActualPosPcs(cartPos)
        if ret != SUCCESS:
            print( "Failed to get cartesian pose!")
            return ret
        fullPos = jntPos + cartPos
        self.pnt_list.append(fullPos)
        return 0

    def updatePoint(self, index=None):
        """
        Store current pose (joint, tcp) as a List of 12 element into existed element of pnt_list.

        :param index: the index of existed element that will be replaced by current pose.
        :type index: int
        :return: error code
        :rtype: int
        """
        if not isinstance(index, int) or index >= len(self.pnt_list) or index < 0:
            print( "index is not valid!")
            return -1
        jntPos = [0.] *6
        cartPos = [0.] * 6
        ret = self.groupGetActualPosAcs(jntPos)
        if ret != SUCCESS:
            print( "Failed to get joint pose!")
            return ret
        self.groupGetActualPosPcs(cartPos)
        if ret != SUCCESS:
            print( "Failed to get cartesian pose!")
            return ret
        fullPos = jntPos + cartPos
        self.pnt_list[index] = fullPos
        return 0

    def readPoint(self, fileName):
        """
        Read Points from CSV file.

        :param fileName: the name of file that have points inside.
        :type fileName: string
        :return:
        """
        with open(fileName, 'r') as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            next(readCSV)   # pop out header
            for row in readCSV:
                pnt_arr = [float(el) for el in row[1:]]
                self.pnt_list.append(pnt_arr)

    def savePoint(self, fileName):
        """
        Save Points to CSV file.

        :param fileName: the name of csv file will save points.
        :type filename: string
        :return:
        """
        csvfile = open(fileName, 'wb')
        writeCSV = csv.writer(csvfile, delimiter=',')
        writeCSV.writerow(('', 'j1', 'j2','j3','j4','j5','j6','x','y','z','a','b','c'))
        pnts = copy.deepcopy(self.pnt_list)
        for id, el in enumerate(pnts):
            el.insert(0, id)
            writeCSV.writerow(el)

        csvfile.close()

def pose2matrix(pose):
    rz = pose[3] * np.pi / 180.
    ry = pose[4] * np.pi / 180.
    rx = pose[5] * np.pi / 180.
    rotz = np.mat([[np.cos(rz), -np.sin(rz),  0,  0],
                   [np.sin(rz),  np.cos(rz),  0,  0],
                   [         0,           0,  1,  0],
                   [         0,           0,  0,  1]])

    roty = np.mat([[ np.cos(ry),  0, np.sin(ry),  0],
                   [          0,  1,          0,  0],
                   [-np.sin(ry),  0, np.cos(ry),  0],
                   [         0,   0,          0,  1]])

    rotx = np.mat([[ 1,           0,           0,  0],
                   [ 0,  np.cos(rx), -np.sin(rx),  0],
                   [ 0,  np.sin(rx),  np.cos(rx),  0],
                   [ 0,           0,           0,  1]])
    HTmat = rotz*roty*rotx
    HTmat[0,3] = pose[0]
    HTmat[1,3] = pose[1]
    HTmat[2,3] = pose[2]
    return HTmat

def getTFmatrix(theta, alpha, a, d):
    tht = theta * (np.pi / 180.)
    alp = alpha * (np.pi / 180.)
    ct, st = np.cos(tht), np.sin(tht)
    ca, sa = np.cos(alp), np.sin(alp)
    out_mat = np.mat([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -sa*d ],
        [st*sa, ct*sa,  ca,  ca*d ],
        [0, 0, 0, 1]
    ])
    return out_mat

try:
    import matplotlib.pyplot as plt
    import mpl_toolkits.mplot3d.axes3d as p3
    import matplotlib.animation as animation
except:
    print("Can not import maplotlib!")
    print("MplVisaul class will not work!")


class MplVisual(object):
    def __init__(self, ax, theta=None):
        self.ax = ax
        if theta is None:
            self.theta = [0, 90, 0, 0, -90, 0] # (deg) joint values
        else:
            self.theta = theta
        if len(self.theta) is not 6:
            print("theta should be a array of 6 value!")
            return
        self.alpha = [0, 90, 0, 90, -90, 90] # deg
        self.d = [339, 0, 0, 250, 0, 95]
        self.a = [0, 0, 250, 70, 0, 0]
        self.tf_mat = []
        self.tf_coor_data = []
        self.joints_pos = [[0.], [0.], [0.] ]  # np.mat([[0.], [0.], [0.]])
        self.tf_coor = []     # List of 3 Line3D (x,y,z)
        self.links3d = None   # Line3D object
        self.arrow_len = 60
        self.draw_id = [1, 1, 1, 1, 1, 1]

    def get_tf_data(self):
        for idx, (tht, alp, ai, di) in enumerate(zip(self.theta, self.alpha, self.a, self.d)):
            #print(idx, len(self.tf_mat))
            mat = []
            coor_bar = []
            if idx == 0:
                mat = getTFmatrix(tht, alp, ai, di)
            else:
                mat = self.tf_mat[idx-1] * getTFmatrix(tht, alp, ai, di)
            for i in range(3):  # get 3-axis of each joint
                self.joints_pos[i].append(mat[i, 3])
                coor_bar.append([[ self.arrow_len*mat[0, i]+mat[0, 3], mat[0, 3] ],
                                 [ self.arrow_len*mat[1, i]+mat[1, 3], mat[1, 3] ],
                                 [ self.arrow_len*mat[2, i]+mat[2, 3], mat[2, 3] ]])
            self.tf_mat.append(mat)
            self.tf_coor_data.append(coor_bar)

    def draw_tf_view(self):
        self.links3d = self.ax.plot(self.joints_pos[0], self.joints_pos[1], self.joints_pos[2], color='orange', marker='o', \
                                    linewidth=4, markersize=12, markerfacecolor='purple')[0]

        # add joint coordinates marker
        #draw_id = [1, 1, 1, 1, 1, 1]
        #tf_coor = []
        for dr, data in zip(self.draw_id, self.tf_coor_data):
            #print(dr, coor)
            if dr: # true for drawing coordinate of the i joint.
                x_dir = self.ax.plot(data[0][0], data[0][1], data[0][2], 'r', linewidth=1)[0]
                y_dir = self.ax.plot(data[1][0], data[1][1], data[1][2], 'g', linewidth=1)[0]
                z_dir = self.ax.plot(data[2][0], data[2][1], data[2][2], 'b', linewidth=1)[0]
                self.tf_coor.append([x_dir, y_dir, z_dir])
        self.ax.set_xlim3d([-300, 450])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-300, 450])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([-50, 700])
        self.ax.set_zlabel('Z')
        #self.ax.axis('equal')
        self.ax.set_title('Robot 3D TF View')

    def update_tf_data(self):
        for idx, (tht, alp, ai, di) in enumerate(zip(self.theta, self.alpha, self.a, self.d)):
            #print(idx, len(self.tf_mat))
            mat = []
            coor_bar = []
            if idx == 0:
                mat = getTFmatrix(tht, alp, ai, di)
            else:
                mat = self.tf_mat[idx-1] * getTFmatrix(tht, alp, ai, di)
            for i in range(3):  # get 3-axis of each joint
                self.joints_pos[i][idx+1] = mat[i, 3]
                coor_bar.append([[ self.arrow_len*mat[0, i]+mat[0, 3], mat[0, 3] ],
                                 [ self.arrow_len*mat[1, i]+mat[1, 3], mat[1, 3] ],
                                 [ self.arrow_len*mat[2, i]+mat[2, 3], mat[2, 3] ]])
            self.tf_mat[idx] = mat
            self.tf_coor_data[idx] = coor_bar

    def update_tf_view(self, theta=None):
        if theta is None:
            print("Need provide joints value")
            return
        if not isinstance(theta, list) or not len(theta) == 6:
            print("input is not a list of 6 value")
            return
        # Update tf_data
        self.theta = theta
        self.update_tf_data()

        # Update Line3D data in mpl ax
        self.links3d.set_data_3d(self.joints_pos[0], self.joints_pos[1], self.joints_pos[2])
        for coor, data in zip(self.tf_coor, self.tf_coor_data):
            for i in range(3):
                coor[i].set_data_3d(data[i][0], data[i][1], data[i][2])
