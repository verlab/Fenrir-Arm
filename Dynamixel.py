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
            "Shadow_ID": ModelFlag(12, 1),
            "Max_Position": ModelFlag(48, 4),
            "Min_Position": ModelFlag(52, 4),
            "Torque_Enabled": ModelFlag(64, 1),
            "Goal_Position": ModelFlag(116, 4),
            "Moving": ModelFlag(122, 1),
            "Present_Position": ModelFlag(132, 4)
        }, 

        # AX series
        12: {
            "Return_Delay_Time": ModelFlag(5, 1),
            "Min_Position": ModelFlag(6, 2),
            "Max_Position": ModelFlag(8, 2),
            "Max_Torque": ModelFlag(14, 2),
            "Torque_Enabled": ModelFlag(24, 1),
            "Goal_Position": ModelFlag(30, 2),
            "Present_Position": ModelFlag(36, 2),
            "Moving": ModelFlag(46, 1)
        }
    }

    angle_limits = {
        311 : {
            "max_angle": 360.0,
            "max_position": 4095,
            "min_angle": 0.0,
            "min_position": 0
        },
        12 : {
            "max_angle": 300.0,
            "max_position": 1023,
            "min_angle": 0.0,
            "min_position": 0
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
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, self.id)
        if dxl_comm_result != COMM_SUCCESS:
            print "Error: %s" % self.packetHandler.getTxRxResult(dxl_comm_result)

        elif dxl_error != 0:
            print "Error: %s" % self.packetHandler.getRxPacketError(dxl_error)

        else:
            print "Found dynamixel [ID:%03d] with model number : %d" % (self.id, dxl_model_number)

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
        self.max_pos = self.limits["max_position"]
        self.min_pos = self.limits["min_position"]

    def updateMaxPositions(self):
        pos, result = self.read("Max_Position")
        if result: 
            self.max_pos = pos
            print "max pos %d" %self.max_pos
        else:
            print "Error: Could not read max position, using default one"

        pos, result = self.read("Min_Position")
        if result: 
            self.min_pos = pos
            print "min pos %d" %self.min_pos
        else:
            print "Error: Could not read max position, using default one"


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
        dxl_comm_result, dxl_error = (None,None)

        if data_length == 1:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(port, id, address, value)
        elif data_length == 2: 
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(port, id, address, value)
        elif data_length == 4: 
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(port, id, address, value)
        else:
            print "Fatal: Undefined data length"
            quit()

        if dxl_comm_result != COMM_SUCCESS:
            print "%s" % self.packetHandler.getTxRxResult(dxl_comm_result)
            return False
        elif dxl_error != 0:
            print "%s" % self.packetHandler.getRxPacketError(dxl_error) 
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
        result, dxl_comm_result, dxl_error = (0, None,None)

        if data_length == 1:
            result, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(port, id, address)
        elif data_length == 2: 
            result, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(port, id, address)
        elif data_length == 4: 
            result, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(port, id, address)
        else:
            print "Fatal: Unsupported data length"
            quit()

        if dxl_comm_result != COMM_SUCCESS:
            print "%s" % self.packetHandler.getTxRxResult(dxl_comm_result)
            return 0, False
        elif dxl_error != 0:
            print "%s" % self.packetHandler.getRxPacketError(dxl_error) 
            return 0, False
        
        return result, True

    # Receives the position in degrees and converts to actual value
    def writeAngle(self, angle):
        max_angle = self.limits['max_angle']
        min_angle = self.limits['min_angle']
        max_position = self.limits['max_position']
        min_position = self.limits['min_position']
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

        max_angle = self.limits['max_angle']
        min_angle = self.limits['min_angle']
        max_position = self.limits['max_position']
        min_position = self.limits['min_position']
        
        ratio = (1.0*position)/abs(max_position-min_position)
        angle = ratio * max_angle

        return angle, True

    def reachedGoal(self):
        position, result = self.read("Present_Position")
        if not result:
            return True

        return abs( position-self.goal_pos ) <= 1
        
    def isMoving(self):
        return self.read("Moving")
        




        

        

    
