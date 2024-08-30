import numpy as np
from dynamixel_sdk import *

# Define constants
ADDR_MX_TORQUE_ENABLE = 64
ADDR_MX_GOAL_POSITION = 116
ADDR_MX_PROFILE_VELOCITY = 112
ADDR_MX_PROFILE_ACCELERATION = 108
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9]
INVERTED_MOTORS = {2, 4}  # IDs of motors that are inverted

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


def open_port():
    if not portHandler.openPort():
        print("Failed to open the port")
        return False
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to change the baudrate")
        portHandler.closePort()
        return False
    return True


def enable_torque(enable=True):
    for motor_id in MOTOR_IDS:
        torque_status = TORQUE_ENABLE if enable else TORQUE_DISABLE
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, motor_id, ADDR_MX_TORQUE_ENABLE, torque_status)
        if dxl_comm_result != COMM_SUCCESS:
            print(
                f"Failed to {'enable' if enable else 'disable'} torque on motor {motor_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(
                f"Error on motor {motor_id}: {packetHandler.getRxPacketError(dxl_error)}")


def set_profile_velocity(velocity):
    for motor_id in MOTOR_IDS:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
            portHandler, motor_id, ADDR_MX_PROFILE_VELOCITY, velocity)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to write profile velocity on motor {motor_id}")
        elif dxl_error != 0:
            print(f"Error when setting profile velocity on motor {motor_id}")


def set_profile_acceleration(acceleration):
    for motor_id in MOTOR_IDS:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
            portHandler, motor_id, ADDR_MX_PROFILE_ACCELERATION, acceleration)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to write profile acceleration on motor {motor_id}")
        elif dxl_error != 0:
            print(
                f"Error when setting profile acceleration on motor {motor_id}")

def init_angles():
	initial = [0, 110, -92, 0, -40, 0, 0, 1]
	with open('/home/pi/Desktop/Abhishek/joint_angles.txt', 'w') as f:
		for angle in initial:
			f.write(f"{angle}\n")

init_angles()

if open_port():
    enable_torque(True)
    set_profile_velocity(40)
    set_profile_acceleration(10)

try:
    while True:
        with open('/home/pi/Desktop/Abhishek/joint_angles.txt', 'r') as f:
            lines = f.readlines()
        if not lines:
            continue

        motor_angles = [float(line.strip()) for line in lines[:-1]]
        control_signal = int(lines[-1].strip())

        if len(motor_angles) < 7:
            print("Not enough angle data for all joints.")
            continue

        motor_to_joint_map = {
            1: motor_angles[0],  # Joint 1
            2: motor_angles[1],  # Joint 2
            3: motor_angles[1],  # Joint 2
            4: motor_angles[2],  # Joint 3
            5: motor_angles[2],  # Joint 3
            6: motor_angles[3],  # Joint 4
            7: motor_angles[4],  # Joint 5
            8: motor_angles[5],  # Joint 6
            9: motor_angles[6]   # Joint 7
        }

        for motor_id in range(1, 10):
            angle = motor_to_joint_map[motor_id]

            # Invert the angle for motors 2 and 4
            if motor_id in {2, 4}:
                angle = -angle

            position = int(4096 / (2 * np.pi) * (angle * np.pi / 180 + np.pi))
            packetHandler.write4ByteTxRx(
                portHandler, motor_id, ADDR_MX_GOAL_POSITION, position)

        if control_signal == 0:
            print("Disabling motor torque")
            enable_torque(False)
            break

finally:
    portHandler.closePort()
