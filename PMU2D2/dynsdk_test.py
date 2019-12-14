import dynamixel_sdk as dynamixel
import serial
import time
import binascii

DEVICENAME = "COM3"
BAUDRATE = 9600

DXL_ID = 1

# Control table address
ADDR_PRO_TORQUE_ENABLE       = 24                          # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 30
ADDR_PRO_PRESENT_POSITION    = 37
ADDR_PRESENT_VOLTAGE = 45

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000                       # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000                        # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = dynamixel.port_handler.PortHandler(DEVICENAME)
port_num.setBaudRate(BAUDRATE)

# Initialize PacketHandler Structs
packetHandler = dynamixel.packet_handler.PacketHandler(PROTOCOL_VERSION)

print("Wait 4 arduino")
time.sleep(2) # Wait for arduino init

print("Test R/W")

# dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
print(packetHandler.broadcastPing(port_num))
#port_num.ser.flushInput()
#port_num.ser.flushOutput()

print(packetHandler.write1ByteTxRx(port_num, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE))
print(packetHandler.read1ByteTxRx(port_num, DXL_ID, ADDR_PRO_TORQUE_ENABLE))

print(packetHandler.write4ByteTxRx(port_num, DXL_ID, ADDR_PRO_GOAL_POSITION, 400))
print(packetHandler.read4ByteTxRx(port_num, DXL_ID, ADDR_PRO_PRESENT_POSITION))

print("Voltage: ", packetHandler.read1ByteTxRx(port_num, DXL_ID, ADDR_PRESENT_VOLTAGE)) # 10x larger than actual value
print("Moving: ", packetHandler.read1ByteTxRx(port_num, DXL_ID, 49))

'''
ser = serial.Serial(DEVICENAME, timeout=5, baudrate=BAUDRATE)

time.sleep(2)

print(ser.write(b'\xFF\xFF\xFD\x00\xFE\x03\x00\x01\x31\x42'))
print(binascii.hexlify(ser.read(100)))
'''