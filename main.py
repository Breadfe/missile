## 윤혁이 포탑 마무리 ##
# 0. 모터 코드로 제어하기
# 1. opcv로 마커 따라가기
# 2. 총 발사하기

import os
import sys
import time
import math
import numpy as np

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * 

# Control table address
ADDR_RX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_RX_GOAL_POSITION      = 30
ADDR_RX_MOVING_SPEED       = 32   
ADDR_RX_PRESENT_POSITION   = 36


# ADDR_MOVING_SPEED       = 32
# ADDR_TORQUE_LIMIT       = 34

# Data Byte Length
LEN_GOAL_POSITION       = 4
LEN_PRESENT_POSITION    = 4
LEN_MOVING_SPEED        = 4
# Protocol version
PROTOCOL_VERSION            = 1.0     

BODY_ID = 0
CAMERA_ID = 1
SHOOT_ID = 2
GUN_ID = 3

BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM10'
#DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20 

#rotate_type
JOINT_TYPE = 0
WHEEL_TYPE = 1

portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
GOAL_POSI_Write = GroupSyncWrite(portHandler, packetHandler, ADDR_RX_GOAL_POSITION, LEN_GOAL_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

class motor():
    def __init__(self, id, rotate_type):
        self.id = id
        self.rotate_type = rotate_type

    def set_position(self, deg):
        rad = deg * math.pi/180
        value = rad * 512 / math.pi
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.id, ADDR_RX_GOAL_POSITION, int(value))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    def torque_on(self):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.id, ADDR_RX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    def shoot(self):# 발사 시작하고 1.515초 딜레이 주면 딱 1사이클 맞음(회전속도 1000 기준)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.id, ADDR_RX_MOVING_SPEED, 1000)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        time.sleep(1.515)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.id, ADDR_RX_MOVING_SPEED, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    #def cease_fire(self):
        

val2deg = 180/512
# initialize
body_joint = motor(BODY_ID, JOINT_TYPE)
body_joint.set_position(180)
camera_joint = motor(CAMERA_ID, JOINT_TYPE)
camera_joint.set_position(425*val2deg)
gun_joint = motor(GUN_ID, JOINT_TYPE)
gun_joint.set_position(536*val2deg)
shoot_sw = motor(SHOOT_ID, WHEEL_TYPE)


# shoot_sw.shoot()



# body_joint.set_position(200)
# time.sleep(1)
# body_joint.set_position(180*math.pi/180)