from dynamixel_sdk.port_handler import *
from dynamixel_sdk.packet_handler import *


class ModelFlag:
    def __init__(self, address, data_length):
        self.address = address
        self.data_length = data_length


class Dynamixel:
    protocol_versions = [1.0, 2.0]
    models = {
        # MX series
        311: {
            "Return_Delay_Time": ModelFlag(9, 1),
            "Drive_Mode": ModelFlag(10, 1),
            "Operating_Mode": ModelFlag(11, 1),
            "Shadow_ID": ModelFlag(12, 1),
            "Velocity_Limit": ModelFlag(44, 4),
            "Max_Position": ModelFlag(48, 4),
            "Min_Position": ModelFlag(52, 4),
            "Shutdown": ModelFlag(63, 1),
            "Torque_Enabled": ModelFlag(64, 1),
            "Profile_Acceleration": ModelFlag(108, 4),
            "Profile_Velocity": ModelFlag(112, 4),
            "Goal_Position": ModelFlag(116, 4),
            "Moving": ModelFlag(122, 1),
            "Present_Current": ModelFlag(126, 2),
            "Present_Velocity": ModelFlag(128, 4),
            "Present_Position": ModelFlag(132, 4),
            "Velocity_Trajectory": ModelFlag(136, 4)
        },

        # AX series
        12: {
            "Return_Delay_Time": ModelFlag(5, 1),
            "Min_Position": ModelFlag(6, 2),
            "Max_Position": ModelFlag(8, 2),
            "Max_Torque": ModelFlag(14, 2),
            "Shutdown": ModelFlag(18, 1),
            "Torque_Enabled": ModelFlag(24, 1),
            "Goal_Position": ModelFlag(30, 2),
            "Present_Position": ModelFlag(36, 2),
            "Present_Velocity": ModelFlag(38, 2),
            "Present_Load": ModelFlag(40, 2),
            "Moving": ModelFlag(46, 1),
        }
    }

    angle_limits = {
        311: {
            "Max_Angle": 360.0,
            "Max_Position": 4095,
            "Min_Angle": 0.0,
            "Min_Position": 0
        },
        12: {
            "Max_Angle": 300.0,
            "Max_Position": 1023,
            "Min_Angle": 0.0,
            "Min_Position": 0
        }
    }

    def __init__(self, portHandler, id, protocol):
        self.portHandler = portHandler
        self.id = id
        if protocol in Dynamixel.protocol_versions:
            self.protocol = protocol
        else:
            print "Invalid protocol, using default"
            self.protocol = Dynamixel.protocol_versions[-1]

        self.packetHandler = PacketHandler(self.protocol)

        # Ping dynamixel
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(
            self.portHandler, self.id)
        if dxl_comm_result != COMM_SUCCESS:
            print "Error: %s" % self.packetHandler.getTxRxResult(
                dxl_comm_result)

        elif dxl_error != 0:
            print "Error: %s" % self.packetHandler.getRxPacketError(dxl_error)

        else:
            model = "MX-64" if dxl_model_number == 311 else "AX-12"
            print "Found dynamixel [ID:%03d] with model : %s" % (
                self.id, model)

        self.model = None
        if dxl_model_number not in Dynamixel.models:
            print 'Error: Model not configured. Using default model.'
            self.model = Dynamixel.models[311]
        else:
            self.model = Dynamixel.models[dxl_model_number]

        self.limits = None
        if dxl_model_number not in Dynamixel.angle_limits:
            print 'Error: Model not configured. Using default model.'
            self.limits = Dynamixel.angle_limits[311]
        else:
            self.limits = Dynamixel.angle_limits[dxl_model_number]

        self.goal_pos = 0
        self.max_pos = self.limits["Max_Position"]
        self.min_pos = self.limits["Min_Position"]

    def updateMaxPositions(self):
        pos, result = self.read("Max_Position")
        if result:
            self.max_pos = pos
            print "max pos %d" % self.max_pos
        else:
            print "Error: Could not read max position, using default one"

        pos, result = self.read("Min_Position")
        if result:
            self.min_pos = pos
            print "min pos %d" % self.min_pos
        else:
            print "Error: Could not read min position, using default one"

    def write(self, flag, value):
        if flag not in self.model:
            print "Error: Flag not specified. Options are:"
            for key in self.model:
                print "\t"+key
            return False

        port = self.portHandler
        id = self.id
        address = self.model[flag].address
        data_length = self.model[flag].data_length
        dxl_comm_result, dxl_error = (None, None)

        if data_length == 1:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                port, id, address, value)
        elif data_length == 2:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
                port, id, address, value)
        elif data_length == 4:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                port, id, address, value)
        else:
            print "Fatal: Undefined data length"
            quit()

        if dxl_comm_result != COMM_SUCCESS:
            print "COMM -> %s" % self.packetHandler.getTxRxResult(dxl_comm_result)
            return False
        elif dxl_error != 0:
            print "ERROR -> %s" % self.packetHandler.getRxPacketError(dxl_error)
            return False

        return True

    def read(self, flag):
        if flag not in self.model:
            print "Error: Flag not specified. Options are:"
            for key in self.model:
                print "\t"+key
            return 0, False

        port = self.portHandler
        id = self.id
        address = self.model[flag].address
        data_length = self.model[flag].data_length
        result, dxl_comm_result, dxl_error = (0, None, None)

        if data_length == 1:
            (result, dxl_comm_result, dxl_error) = self.packetHandler.read1ByteTxRx(
                port, id, address)
        elif data_length == 2:
            (result, dxl_comm_result, dxl_error) = self.packetHandler.read2ByteTxRx(
                port, id, address)
        elif data_length == 4:
            (result, dxl_comm_result, dxl_error) = self.packetHandler.read4ByteTxRx(
                port, id, address)
        else:
            print "Fatal: Unsupported data length"
            quit()

        if dxl_comm_result != COMM_SUCCESS:
            print "dxl_comm_result -> %s" % self.packetHandler.getTxRxResult(
                dxl_comm_result)
            return 0, False
        elif dxl_error != 0:
            print "dxl_error -> %s" % self.packetHandler.getRxPacketError(
                dxl_error)
            return 0, False

        return result, True

    # Receives the position in degrees and converts to actual value
    def writeAngle(self, angle):
        max_angle = self.limits['Max_Angle']
        min_angle = self.limits['Min_Angle']
        max_position = self.limits['Max_Position']
        min_position = self.limits['Min_Position']
        if angle < min_angle or angle > max_angle:
            print "Error: Angle out of range"
            return False

        # Convert angle to position
        ratio = angle/(abs(max_angle - min_angle))
        position = int(round(ratio*(abs(max_position-min_position))))

        # Cap position to maximum position set on the dynamixel
        if position > self.max_pos:
            position = self.max_pos
        elif position < self.min_pos:
            position = self.min_pos

        result = self.write("Goal_Position", position)
        self.goal_pos = position
        return result

    def readAngle(self):
        position, result = self.read("Present_Position")
        if not result:
            return 0.0, False

        max_angle = self.limits['Max_Angle']
        min_angle = self.limits['Min_Angle']
        max_position = self.limits['Max_Position']
        min_position = self.limits['Min_Position']

        ratio = (1.0*position)/abs(max_position-min_position)
        angle = ratio * max_angle

        return angle, True

    def readLoad(self):
        if self.protocol == 2:
            current, result = self.read("Present_Current")
            if (current > 32768):
                current = current - 65536
            load = (1.1489*current - 0.172340426)*0.001
            if not result:
                return 0.0, False
        elif self.protocol == 1:
            load, result = self.read("Present_Load")
            if not result:
                return 0.0, False
        return load, True

    def readVelocity(self):
        velocity, result = self.read("Present_Velocity")
        if not result:
            return 0.0, False
        return velocity, True

    def reachedGoal(self):
        position, result = self.read("Present_Position")
        if not result:
            return True

        return abs(position-self.goal_pos) <= 1

    def isMoving(self):
        return self.read("Moving")
