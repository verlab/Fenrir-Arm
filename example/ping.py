# Pings dynamixel

import os

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

from dynamixel_sdk.port_handler import *
from dynamixel_sdk.packet_handler import *

# Protocol version
PROTOCOL_VERSION        = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                  = 4                 # Dynamixel ID : 1
BAUDRATE                = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME              = '/dev/ttyUSB0'    # Check which port is being used on your controller

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print "Open port successful"
else:
    print "Failed to open port"
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print "Succeeded to change the baudrate"
else:
    print "Failed to change the baudrate"
    print "Press any key to terminate..."
    getch()
    quit()

# Ping dynamixel
dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID)
if dxl_comm_result != COMM_SUCCESS:
    print "Error: %s" % packetHandler.getTxRxResult(dxl_comm_result)

elif dxl_error != 0:
    print "Error: %s" % packetHandler.getRxPacketError(dxl_error)

else:
    print "[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID, dxl_model_number)

# Close port
portHandler.closePort()

