import os, ctypes
import time
import numpy as np

from sawyer_control.configs import base_config

os.sys.path.append('../dynamixel_functions_py')
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24                            # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_PRESENT_VELOCITY    = 38
ADDR_MX_PRESENT_POS_VEL     = 36                            # Trick to get position and velocity at once
ADDR_MX_MAX_VELOCITY        = 32
ADDR_MX_TRQ_LIMIT           = 35
ADDR_MX_PRESENT_LOAD        = 40

# Data Byte Length
LEN_MX_PRESENT_POSITION     = 2
LEN_MX_PRESENT_VELOCITY     = 2
LEN_MX_PRESENT_POS_VEL      = 4
LEN_MX_GOAL_POSITION        = 2
LEN_MX_PRESENT_LOAD         = 2


# torque control mode options
ADDR_MX_TORQUE_CONTROL_MODE = 70
ADDR_MX_GOAL_TORQUE         = 72                            # Lowest byte of goal torque value
DXL_NULL_TORQUE_VALUE       = 0
DXL_MIN_CW_TORQUE_VALUE     = 1024
DXL_MAX_CW_TORQUE_VALUE     = 2047
DXL_MIN_CCW_TORQUE_VALUE    = 0
DXL_MAX_CCW_TORQUE_VALUE    = 1023
LEN_MX_GOAL_TORQUE          = 2

# Protocol version
PROTOCOL_VERSION            = 1                             # See which protocol version is used in the Dynamixel

# Default setting
MX12                        = 1
MX28                        = 2
MX64                        = 3


# Settings for MX28
POS_SCALE                   = 2*np.pi/4096 #(=.088 degrees)
VEL_SCALE                   = 0.11 * 2 * np.pi / 60 #(=0.11rpm)

MAX_LOAD                    = 1023


BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 100                           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000                          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 15                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed



class dxl():
    def __init__(self, motor_id, DEVICENAME=DEVICENAME, config=base_config):

        self.n_motors = len(motor_id)

        # default mode
        self.ctrl_mode = TORQUE_DISABLE

        # Initialize PortHandler Structs
        # Set the port path and Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.port_num = dynamixel.portHandler(DEVICENAME)

        self.config=config
        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

        # Open port
        if dynamixel.openPort(self.port_num):
            print("Succeeded to open the port!")
        else:
            print("Failed to open the port")
            os.system("sudo chmod a+rw /dev/ttyUSB0")
            print("Editing permissions and trying again")
            if dynamixel.openPort(self.port_num):
                print("Succeeded to open the port!")
            else:
                quit("Failed to open the port! Run following command and try again.\nsudo chmod a+rw /dev/ttyUSB0")

        # Set port baudrate
        if dynamixel.setBaudRate(self.port_num, BAUDRATE):
            print("Succeeded to change the baudrate!")
        else:
            quit("Failed to change the baudrate!")

        # Enable Dynamixel Torque
        self.engage_motor(motor_id, True)

        # Initialize Group instance

        # controls
        self.group_desPos = dynamixel.groupSyncWrite(self.port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
        self.group_desTor = dynamixel.groupSyncWrite(self.port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_TORQUE, LEN_MX_GOAL_TORQUE)

        # positions
        self.group_pos = dynamixel.groupBulkRead(self.port_num, PROTOCOL_VERSION)
        for m_id in motor_id:
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupBulkReadAddParam(self.group_pos, m_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupBulkRead addparam_posfailed" % (m_id))
                quit()

        # velocities
        self.group_vel = dynamixel.groupBulkRead(self.port_num, PROTOCOL_VERSION)
        for m_id in motor_id:
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupBulkReadAddParam(self.group_vel, m_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY)).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupBulkRead addparam_vel failed" % (m_id))
                quit()

        # positions and velocities
        self.group_pos_vel = dynamixel.groupBulkRead(self.port_num, PROTOCOL_VERSION)
        for m_id in motor_id:
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupBulkReadAddParam(self.group_pos_vel, m_id, ADDR_MX_PRESENT_POS_VEL, LEN_MX_PRESENT_POS_VEL)).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupBulkRead addparam_posfailed" % (m_id))
                quit()

        # loads
        self.group_load = dynamixel.groupBulkRead(self.port_num, PROTOCOL_VERSION)
        for m_id in motor_id:
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupBulkReadAddParam(self.group_load, m_id, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupBulkRead addparam_loadfailed" % (m_id))
                quit()

        # buffers
        self.dxl_present_position = float('nan')*np.zeros(self.n_motors)
        self.dxl_present_velocity = float('nan')*np.zeros(self.n_motors)
        self.dxl_last_position = float('nan')*np.zeros(self.n_motors)
        self.dxl_last_velocity = float('nan')*np.zeros(self.n_motors)
        print("Dynamixel has been successfully connected")


    # Cheak health
    def okay(self):
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, PROTOCOL_VERSION)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
            return False
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
            return False
        else:
            return True


    # Engage/ Disengae the motors. enable = True/ False
    def engage_motor(self, motor_id, enable):
        for dxl_id in motor_id:

            # fault handelling
            while(True):
                dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_ENABLE, enable)
                if(self.okay()):
                    break
                else:
                    print('dxl%d: Error with ADDR_MX_TORQUE_ENABLE. Retrying ...' %dxl_id, flush=True)
                    time.sleep(0.25)


    # Returns pos in radians and velocity in radian/ sec
    def get_pos_vel_old(self, motor_id):

        dxl_present_position = []
        dxl_present_velocity = []

        # Bulkread present positions
        dynamixel.groupBulkReadTxRxPacket(self.group_pos_vel)

        # Retrieve data
        for i in range(self.n_motors):
            dxl_id = motor_id[i]
            # Get present position value
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupBulkReadIsAvailable(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_POS_VEL, LEN_MX_PRESENT_POS_VEL)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupBulkRead get_pos_vel failed" % (dxl_id))
                dxl_present_position.append(float('nan'))
                dxl_present_velocity.append(float('nan'))
                # quit()
            else:
                dxl_present_position.append(dynamixel.groupBulkReadGetData(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION))
                dxl_present_velocity.append(dynamixel.groupBulkReadGetData(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY))

        dxl_present_velocity = np.array(dxl_present_velocity)
        for i in range(len(dxl_present_velocity)):
            if(dxl_present_velocity[i]>=1024):
                dxl_present_velocity[i] = -1.*(dxl_present_velocity[i] - 1024)
        return POS_SCALE*np.array(dxl_present_position), VEL_SCALE*np.array(dxl_present_velocity)


    # Returns pos in radians and velocity in radian/ sec
    def get_pos_vel(self, motor_id):

        # Bulkread present positions
        dynamixel.groupBulkReadTxRxPacket(self.group_pos_vel)
        if(not self.okay()):
            print("try one more time. If not, we will spoof packets below ====================== ")
            # try one more time. If not, we will spoof packets below.
            dynamixel.groupBulkReadTxRxPacket(self.group_pos_vel)

        dxl_errored = []
        # Retrieve data
        for i in range(self.n_motors):
            dxl_id = motor_id[i]
            # Get present position value
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupBulkReadIsAvailable(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_POS_VEL, LEN_MX_PRESENT_POS_VEL)).value
            if dxl_getdata_result != 1:
                #send last known values
                dxl_errored.append(dxl_id)
                self.dxl_present_position[i] = self.dxl_last_position[i].copy()
                self.dxl_present_velocity[i] = self.dxl_last_velocity[i].copy()
            else:
                self.dxl_last_position[i] = self.dxl_present_position[i].copy()
                self.dxl_last_velocity[i] = self.dxl_present_velocity[i].copy()

                dxl_present_position = dynamixel.groupBulkReadGetData(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
                dxl_present_velocity = dynamixel.groupBulkReadGetData(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY)
                if(dxl_present_velocity>=1024):
                    dxl_present_velocity = -1.*(dxl_present_velocity - 1024)

                self.dxl_present_position[i] = POS_SCALE*dxl_present_position
                self.dxl_present_velocity[i] = VEL_SCALE*dxl_present_velocity

        if len(dxl_errored):
            print("groupBulkRead get_pos_vel failed. Sending last known values for dynamixel ids: " + str(dxl_errored),flush=True)
        return self.dxl_present_position.copy(), self.dxl_present_velocity.copy()


    # Returns pos in radians
    def get_pos(self, motor_id):
        dxl_present_position = []

        # Bulkread present positions
        dynamixel.groupBulkReadTxRxPacket(self.group_pos)

        # Retrieve data
        for dxl_id in motor_id:
            # Get present position value
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupBulkReadIsAvailable(self.group_pos, dxl_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupBulkRead pos_getdata failed" % (dxl_id))
                dxl_present_position.append(0)
                quit()
            else:
                dxl_present_position.append(dynamixel.groupBulkReadGetData(self.group_pos, dxl_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION))

        return POS_SCALE*np.array(dxl_present_position)


    # Returns load as proportion of max load
    def get_load(self, motor_id):
        dxl_present_load = []

        # Bulkread present positions
        dynamixel.groupBulkReadTxRxPacket(self.group_load)

        # Retrieve data
        for dxl_id in motor_id:
            # Get present position value
            # import IPython; IPython.embed()
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupBulkReadIsAvailable(self.group_load, dxl_id, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupBulkRead load_getdata failed" % (dxl_id))
                dxl_present_load.append(0)
                # quit()
            else:
                dxl_present_load.append(dynamixel.groupBulkReadGetData(self.group_load, dxl_id, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD))

        # print(dxl_present_load)
        return np.array([-load / MAX_LOAD if load <= MAX_LOAD else (load - (MAX_LOAD+1)) / MAX_LOAD for load in dxl_present_load])


    # Returns vel in radians/sec
    def get_vel(self, motor_id):
        dxl_present_velocity = []

        dynamixel.groupBulkReadTxRxPacket(self.group_vel)
        for dxl_id in motor_id:
            # Get present velocity value
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupBulkReadIsAvailable(self.group_vel, dxl_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupBulkRead vel_getdata failed" % (dxl_id))
                dxl_present_velocity.append(0)
                quit()
            else:
                dxl_present_velocity.append(dynamixel.groupBulkReadGetData(self.group_vel, dxl_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY))

        dxl_present_velocity = np.array(dxl_present_velocity)
        for i in range(len(dxl_present_velocity)):
            if(dxl_present_velocity[i]>=1024):
                dxl_present_velocity[i] = -1.*(dxl_present_velocity[i] - 1024)

        return VEL_SCALE*dxl_present_velocity


    # Returns pos in radians
    def getIndividual_pos(self, motor_id):
        dxl_present_position = []

        # Read present position and velocity
        for dxl_id in motor_id:
            dxl_present_position.append(dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_PRESENT_POSITION))
        if (not self.okay()):
            self.close(motor_id)
            quit('error getting ADDR_MX_PRESENT_POSITION')
        return POS_SCALE*np.array(dxl_present_position)


    # Returns vel in radians/sec
    def getIndividual_vel(self, motor_id):
        dxl_present_velocity = []
        # Read present position
        for dxl_id in motor_id:
            dxl_present_velocity.append(dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_PRESENT_VELOCITY))
        if (not self.okay()):
            self.close(motor_id)
            quit('error getting ADDR_MX_PRESENT_VELOCITY')

        dxl_present_velocity = np.array(dxl_present_velocity)

        for i in range(len(dxl_present_velocity)):
            if(dxl_present_velocity[i]>=1024):
                dxl_present_velocity[i] = -1.*(dxl_present_velocity[i] - 1024)

        return VEL_SCALE*dxl_present_velocity


    # Expects des_pos in radians
    def setIndividual_des_pos(self, motor_id, des_pos_inRadians):
        # if in torque mode, activate position control mode
        if(self.ctrl_mode == TORQUE_ENABLE):
            for dxl_id in motor_id:
                dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_DISABLE)
            if (not self.okay()):
                self.close(motor_id)
                quit('error disabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_DISABLE

        # Write goal position
        for i in range(len(motor_id)):
            dynamixel.write2ByteTxRx(self.port_num, PROTOCOL_VERSION, motor_id[i], ADDR_MX_GOAL_POSITION, int(des_pos_inRadians[i]/POS_SCALE))
        if (not self.okay()):
            self.close(motor_id)
            quit('error setting ADDR_MX_GOAL_POSITION')


    # Expects des_pos in radians (0-2*pi)
    def set_des_pos(self, motor_id, des_pos_inRadians):

        # des_pos_inRadians = np.clip(des_pos_inRadians, 0.0, 2*np.pi)
        # if in torque mode, activate position control mode
        if(self.ctrl_mode == TORQUE_ENABLE):
            for dxl_id in motor_id:
                dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_DISABLE)
            if (not self.okay()):
                self.close(motor_id)
                quit('error disabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_DISABLE

        # Write goal position
        for i in range(len(motor_id)):
            dxl_addparam_result = ctypes.c_ubyte(
                dynamixel.groupSyncWriteAddParam(
                    self.group_desPos,
                    motor_id[i],
                    int(des_pos_inRadians[i]/POS_SCALE),
                    LEN_MX_GOAL_POSITION)
            ).value
            if dxl_addparam_result != 1:
                print(dxl_addparam_result)
                print("[ID:%03d] groupSyncWrite addparam failed" % (motor_id[i]))
                self.close(motor_id)
                quit()

        # Syncwrite goal position
        dynamixel.groupSyncWriteTxPacket(self.group_desPos)
        if(not self.okay()):
                self.close(motor_id)
                quit('error bulk commanding desired positions')

        # Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(self.group_desPos)

    def set_trq_limit(self, motor_id, max_trq=200):
        dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, motor_id, ADDR_MX_TRQ_LIMIT, max_trq)

    def get_trq_limit(self, motor_id):
        return dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, motor_id, ADDR_MX_TRQ_LIMIT)



    # Set desired torques, only for MX64
    def set_des_torque(self, motor_id, des_tor):
        # If in position mode, activate torque mode
        if(self.ctrl_mode == TORQUE_DISABLE):
            for dxl_id in motor_id:
                dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_ENABLE)
            if (not self.okay()):
                self.close(motor_id)
                quit('error enabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_ENABLE

        # Write goal position
        for i in range(len(motor_id)):
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(self.group_desTor, motor_id[i], int(des_tor[i]), LEN_MX_GOAL_TORQUE)).value
            if dxl_addparam_result != 1:
                print(dxl_addparam_result)
                print("[ID:%03d] groupSyncWrite addparam failed" % (motor_id[i]))
                self.close(motor_id)
                quit()

        # Syncwrite goal position
        dynamixel.groupSyncWriteTxPacket(self.group_desTor)
        if(not self.okay()):
            self.close(motor_id)
            quit('error bulk commanding desired torques')

        # Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(self.group_desTor)


    # Set desired torques, only for MX64
    def setIndividual_des_torque(self, motor_id, des_tor):
        # If in position mode, activate torque mode
        if(self.ctrl_mode == TORQUE_DISABLE):
            for dxl_id in motor_id:
                dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_ENABLE)
            if (not self.okay()):
                self.close(motor_id)
                quit('error enabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_ENABLE

        # Write goal position
        for i in range(len(motor_id)):
            dynamixel.write2ByteTxRx(self.port_num, PROTOCOL_VERSION, motor_id[i], ADDR_MX_GOAL_TORQUE, int(des_tor[i]))
        if (not self.okay()):
            self.close(motor_id)
            quit('error setting ADDR_MX_GOAL_TORQUE')


    # Set maximum velocity
    def set_max_vel(self, motor_id, max_vel):
        for dxl_id in motor_id:
            dynamixel.write2ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_MAX_VELOCITY, max_vel)
            if (not self.okay()):
                self.close(motor_id)
                quit('error setting ADDR_MX_MAX_VELOCITY')

    # Close connection
    def close(self, motor_id):
        # Disengage Dynamixels
        self.engage_motor(motor_id, False)

        # Close port
        dynamixel.closePort(self.port_num)

        return True

    def reset(self, dxl_ids):
        self.set_des_pos_loop(dxl_ids, 20)
        self.set_des_pos_loop(dxl_ids, 10)
        time.sleep(.5)
        return self.get_pos(dxl_ids)

    def set_des_pos_loop(self, dxl_ids, goal_position):
        self.engage_motor(dxl_ids, True)
        for i in range(100):
            load = np.abs(self.get_load(dxl_ids)[0])
            if load > .9:
                break
            self.set_des_pos(dxl_ids, [goal_position])
            self.set_max_vel(dxl_ids, self.config.DYNAMIXEL_SPEED)
        self.engage_motor(dxl_ids, False)


if __name__ == '__main__':
    dxl_ids =  [1]
    dy = dxl(dxl_ids)
    dy.reset(dxl_ids)
    # Close connection and exit
    dy.close(dxl_ids)
    print('successful exit')
