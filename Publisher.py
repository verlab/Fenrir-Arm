import os 
from Dynamixel import Dynamixel

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

BAUDRATE                = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME              = '/dev/ttyUSB0'    # Check which port is being used on your controller

portHandler = PortHandler(DEVICENAME)

# Open port
if portHandler.openPort():
    print "Open port successful"
else:
    print "Failed to open port"
    print "Press any key to terminate..."
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print "Succeeded to set the baudrate"
else:
    print "Failed to change the baudrate"
    print "Press any key to terminate..."
    getch()
    quit()

# TODO initialize dynamixels in a for loop, after reading info from file
mx64_1 = Dynamixel(portHandler, 7, 2.0)
mx64_2 = Dynamixel(portHandler, 2, 2.0)
ax12 = Dynamixel(portHandler, 4, 1.0)
dynamixels = {
    "shoulder": mx64_1,
    "shoulder_shadow": mx64_2,
    "actuator": ax12
}

# TODO for each dynamixel, set the flags read from file, update max position (important), etc...
for key in dynamixels:
    dxl = dynamixels[key]

    dxl.write("Return_Delay_Time", 250)
    dxl.updateMaxPositions()

dynamixels['shoulder_shadow'].write("Shadow_ID", 7)

# Enable torque AFTER setting flags
dynamixels['actuator'].write("Torque_Enabled", 1)
dynamixels['shoulder_shadow'].write("Torque_Enabled", 1)
dynamixels['shoulder'].write("Torque_Enabled", 1)

# TODO subscribe to topic and send position controls to dynamixels
isMoving = False
goal_angle = 300
thresh = 0.1
dynamixels['shoulder'].writeAngle(goal_angle)
dynamixels['actuator'].writeAngle(goal_angle)

while True:
    angle, result = dynamixels['shoulder'].readAngle()
    if not result:
        print "Error: could not read angle"
        break
    
    print "Current angle: %f" % (angle)
    
    # Test if has finished movement
    if dynamixels['actuator'].reachedGoal() and dynamixels['shoulder'].reachedGoal():
        break

# Disable Torque
dynamixels['actuator'].write("Torque_Enabled", 0)
dynamixels['shoulder_shadow'].write("Torque_Enabled", 0)
dynamixels['shoulder'].write("Torque_Enabled", 0)