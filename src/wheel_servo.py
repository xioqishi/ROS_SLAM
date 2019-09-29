#!/usr/bin/env python
import time
import sys
import os
import rospy
from std_msgs.msg import String
from dynamixel_sdk import *
from geometry_msgs.msg import Twist




# Control table address
# Control table address is different in Dynamixel model
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30                # 0 - 4095
ADDR_MX_PRESENT_POSITION = 36
ADDR_CW_ANGLE_LIMIT = 6
ADDR_CCW_ANGLE_LIMIT = 8
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_SPEED = 38
ADDR_Present_Load = 40
ADDR_Present_Temperature = 43
# Data Byte Lengthf
LEN_MX_GOAL_POSITION = 4
LEN_MX_PRESENT_POSITION = 4
LEN_MOVING_SPEED = 2
LEN_PRESENT_SPEED = 2
LEN_PRESENT_LAOD = 2
LEN_PRESENT_TEMPERATURE = 1
# Protocol version
# See which protocol version is used in the Dynamixel
PROTOCOL_VERSION = 1.0

# Default setting
DXL1_ID = 1                 # Dynamixel#1 ID : 1
DXL2_ID = 2                 # Dynamixel#1 ID : 2
BAUDRATE = 57600             # Dynamixel default baudrate : 57600
DEVICENAME = '/dev/ttyUSB0'    # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE = 1                 # Value for enabling the torque
TORQUE_DISABLE = 0                 # Value for disabling the torque
# Dynamixel will rotate between this value
DXL_MINIMUM_POSITION_VALUE = 100
# and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MAXIMUM_POSITION_VALUE = 4000
DXL_MOVE_SPEED = 500  # 0-1023 ccw; 1024-2047 cw
DXL_STOP_SPEED = 0
# Dynamixel moving status threshold
DXL_MOVING_STATUS_THRESHOLD = 20
WHEEL_ENABLE = 0

index = 1


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)


# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MOVING_SPEED, LEN_MOVING_SPEED)

class DXL_SPEED:
    DXL_LEFT_SPEED = 0
    DXL_RIGHT_SPEED = 0
    QUIT = 0


#global DXL_MOVE_SPEED


def torque_enable_def():
    # Enable Dynamixel#1 Torque
    
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pass
###        print("Dynamixel#%d has been successfully connected" % DXL1_ID)

    # Enable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pass
###        print("Dynamixel#%d has been successfully connected" % DXL2_ID)


def read_Temperature():
   # Read Dynamixel#1 Temperature
    dxl1_present_temp, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(
        portHandler, DXL1_ID, ADDR_Present_Temperature)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print("ID %d Tem %d" % (DXL1_ID, dxl1_present_temp))

    # Read Dynamixel#2 Temperature
    dxl2_present_temp, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(
        portHandler, DXL2_ID, ADDR_Present_Temperature)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print("ID %d Tem %d" % (DXL2_ID, dxl2_present_temp))


def read_load():
    # Read Dynamixel#1 Load
    dxl1_present_load, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
        portHandler, DXL1_ID, ADDR_Present_Load)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    if dxl1_present_load > 1024:
        real_load = (dxl1_present_load - 1024) / 10
    elif dxl1_present_load <= 1024:
        real_load = dxl1_present_load / 10

    print("ID %d Load is %f  %%" % (DXL1_ID, real_load))

    # Read Dynamixel#2 Load
    dxl2_present_load, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
        portHandler, DXL2_ID, ADDR_Present_Load)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    if dxl2_present_load > 1024:
        real_load = (dxl2_present_load - 1024) / 10
    elif dxl2_present_load <= 1024:
        real_load = dxl2_present_load / 10

    print("ID %d Load is %f  %%" % (DXL2_ID, real_load))


def torque_disable():
    # Disable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()


def write_wheel_direction(WHEEL_SPEED):
    # Allocate goal position value into byte array
    param_goal_speed = [DXL_LOBYTE(DXL_LOWORD(WHEEL_SPEED.DXL_LEFT_SPEED)), DXL_HIBYTE(
        DXL_LOWORD(WHEEL_SPEED.DXL_LEFT_SPEED))]

    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_speed)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()

    param_goal_speed = [DXL_LOBYTE(DXL_LOWORD(WHEEL_SPEED.DXL_RIGHT_SPEED)), DXL_HIBYTE(
        DXL_LOWORD(WHEEL_SPEED.DXL_RIGHT_SPEED))]
    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage

    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_speed)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    # Read Dynamixel#1 present speed
    dxl1_present_speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
        portHandler, DXL1_ID, ADDR_PRESENT_SPEED)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read Dynamixel#2 present speed
    dxl2_present_speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
        portHandler, DXL2_ID, ADDR_PRESENT_SPEED)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    print("[ID:%03d] GoalSpeed:%03d  PresSpeed:%03d\t[ID:%03d] GoalSpeed:%03d  PresSpeed:%03d"
          % (DXL1_ID, WHEEL_SPEED.DXL_LEFT_SPEED, dxl1_present_speed, DXL2_ID, WHEEL_SPEED.DXL_RIGHT_SPEED, dxl2_present_speed))

def open_port():
    # Open port
    if portHandler.openPort():
        pass
        #print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        #print("Succeeded to change the baudrate")
        pass
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

def CommandCallback(message_t):
    N_linear = 2500.47 * message_t.linear.x 
    N_angular = 412.578 * message_t.angular.z  
    WHEEL_SPEED = DXL_SPEED()
    WHEEL_SPEED.DXL_LEFT_SPEED = int(N_linear - N_angular)
    WHEEL_SPEED.DXL_RIGHT_SPEED = int(N_linear + N_angular) + 1024
    if(WHEEL_SPEED.DXL_LEFT_SPEED < 0):
        WHEEL_SPEED.DXL_LEFT_SPEED = -WHEEL_SPEED.DXL_LEFT_SPEED + 1024
    if(WHEEL_SPEED.DXL_RIGHT_SPEED < 1024):
        WHEEL_SPEED.DXL_RIGHT_SPEED = -WHEEL_SPEED.DXL_RIGHT_SPEED + 1024
    if(WHEEL_SPEED.DXL_LEFT_SPEED > 2047):
        WHEEL_SPEED.DXL_LEFT_SPEED = 2047
    if(WHEEL_SPEED.DXL_RIGHT_SPEED > 2047):
        WHEEL_SPEED.DXL_RIGHT_SPEED = 2047
    write_wheel_direction(WHEEL_SPEED)








if __name__ == "__main__":
    try:
        open_port()
        torque_enable_def()
        rospy.init_node('wheel_sub', anonymous = True)
        rospy.Subscriber('/twist_mux/cmd_vel', Twist, CommandCallback)
        rospy.spin()
    except KeyboardInterrupt:
        print("Stop it")
        stop()
        torque_disable()
          
