from dynamixel_sdk.port_handler import *
from dynamixel_sdk.packet_handler import *
from colors import *
import numpy as np

import rospy

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
            "Goal_Velocity": ModelFlag(104, 4),
            "Profile_Acceleration": ModelFlag(108, 4),
            "Profile_Velocity": ModelFlag(112, 4),
            "Goal_Position": ModelFlag(116, 4),
            "Moving": ModelFlag(122, 1),
            "Present_Current": ModelFlag(126, 2),
            "Present_Velocity": ModelFlag(128, 4),
            "Present_Position": ModelFlag(132, 4),
            "Present_Voltage": ModelFlag(144, 2),
            "Present_Temperature": ModelFlag(146, 1),
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
            "Velocity_Limit": ModelFlag(32, 2),
            "Goal_Velocity": ModelFlag(32, 2),
            "Torque_Limit": ModelFlag(34, 2),
            "Present_Position": ModelFlag(36, 2),
            "Present_Velocity": ModelFlag(38, 2),
            "Present_Load": ModelFlag(40, 2),
            "Present_Voltage": ModelFlag(42, 1),
            "Present_Temperature": ModelFlag(43, 1),
            "Moving": ModelFlag(46, 1)
        }
    }

    motors_limits = {
        311: {
            "Max_Angle": (2.0*np.pi),
            "Max_Position": 4095,
            "Max_Velocity": 24.5323,
            "Min_Angle": 0.0,
            "Min_Position": 0,
            "Min_Velocity": -24.5323
        },
        12: {
            "Max_Angle": (300.0*np.pi/180.0),
            "Max_Position": 1023,
            "Max_Velocity": 6.1784,
            "Min_Angle": 0.0,
            "Min_Position": 0,
            "Min_Velocity": -6.1784
        }
    }

    def __init__(self, portHandler, id, protocol):
        self.portHandler = portHandler
        self.id = id
        if protocol in Dynamixel.protocol_versions:
            self.protocol = protocol
        else:
            # print ("Invalid protocol, using default")
            rospy.logwarn("[ID:{:03}] Unable to use protocol version {}! Switching 2.0 version by default.".format(id, protocol))
            self.protocol = Dynamixel.protocol_versions[-1]

        self.packetHandler = PacketHandler(self.protocol)

        # Ping dynamixel
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(
            self.portHandler, self.id)
        if dxl_comm_result != COMM_SUCCESS:
            err = self.packetHandler.getTxRxResult(dxl_comm_result)
            rospy.logwarn("[ID:{:03}] Unable to ping the motor. Getting dxl_comm_result error {}.".format(id, err))
            # print ("Error: {}".format(self.packetHandler.getTxRxResult(dxl_comm_result)))

        elif dxl_error != 0:
            err = self.packetHandler.getRxPacketError(dxl_error)
            rospy.logwarn("[ID:{:03}] Unable to ping the motor. Getting dxl_error error {}.".format(id, err))
            # print ("Error: {}}".format(self.packetHandler.getRxPacketError(dxl_error)))

        else:
            model = "MX-64" if dxl_model_number == 311 else "AX-12"
            if model == "MX-64":
                rospy.loginfo("[ID:{:03}] Founded a MX-64 motor.".format(self.id))
                # printC(MX64)
            elif model == "AX-12":
                # printC(AX12)
                rospy.loginfo("[ID:{:03}] Founded a AX-12 motor.".format(self.id))
            # print "-> [ID:%03d]" % (self.id)

        self.model = None
        if dxl_model_number not in Dynamixel.models:
            # print ('Error: Model not configured. Using default model.')
            rospy.logwarn("[ID:{:03}] Model was not configured! Using default model.".format(self.id))
            self.model = Dynamixel.models[311]
        else:
            self.model = Dynamixel.models[dxl_model_number]

        self.limits = None
        if dxl_model_number not in Dynamixel.motors_limits:
            # print ('Error: Model not configured. Using default model.')
            rospy.logwarn("[ID:{:03}] Model was not configured! Using default model.".format(self.id))
            self.limits = Dynamixel.motors_limits[311]
        else:
            self.limits = Dynamixel.motors_limits[dxl_model_number]

        self.goal_pos = 0
        self.max_pos = self.limits["Max_Position"]
        self.min_pos = self.limits["Min_Position"]
        self.max_vel = self.limits["Max_Velocity"]
        self.min_vel = self.limits["Min_Velocity"]
        self.mode = 3

    def setControlMode(self):
        # print("ID %i -> " % self.id)
        # rospy.loginfo("[ID:{:03}] Setting the control mode.".format(self.id))
        if self.protocol == 2:
            mode, result = self.read("Operating_Mode")
            if result:
                if mode in [1, 3]:
                    self.mode = mode
                else:
                    rospy.logwarn("[ID:{:03}] Operation mode was not specified! Using position control as default.".format(self.id))
                    # printC(WARNING, "Error: Not specified operation mode,"
                    #        "using default position control")
                    
            else:
                rospy.logwarn("[ID:{:03}] Unable to read the operation mode! Using position control as default.".format(self.id))
                # printC(WARNING, "Error: Could not read operation mode, using"
                #        "default position control")
        elif self.protocol == 1:
            self.mode = 1 if (self.max_pos == 0 and self.min_pos == 0) else 3
        if (self.mode == 1):
            rospy.loginfo("[ID:{:03}] Setting the control mode = {}, using velocity control.".format(self.id, self.mode))
            # printC("   |")
            # printC("   `---> Operation mode = 1, using velocity control.")
        elif (self.mode == 3):
            rospy.loginfo("[ID:{:03}] Setting the control mode = {}, using position control.".format(self.id, self.mode))
            # printC("   |")
            # printC("   `---> Operation mode = 3, using position control.")
        


    def updateLimits(self):
        # print("ID %i -> " % self.id),
        pos, result = self.read("Max_Position")
        if result:
            self.max_pos = pos
        else:
            # printC(WARNING, "Could not read max position, using default one")
            rospy.logwarn("[ID:{:03}] Unable to read max position! Using default Max_Position = {}.".format(self.id, self.max_pos))

        pos, result = self.read("Min_Position")
        if result:
            self.min_pos = pos
        else:
            rospy.logwarn("[ID:{:03}] Unable to read min position! Using default Min_Position = {}.".format(self.id, self.min_pos))
            # printC(WARNING, "Error: Could not read min position, using default one")

        if self.protocol == 2:
            vel, result = self.read("Velocity_Limit")
            if result:
                self.max_vel = vel
                self.min_vel = -vel
            else:
                rospy.logwarn("[ID:{:03}] Unable to read velocity limit! Using default Velocity_Limit = +/-{}.".format(self.id, self.max_vel))
                # printC(
                #     WARNING, "Error: Could not read velocity limit, using default one")
        rospy.loginfo("[ID:{:03}] Setting angle limits to ({},{}) and velocity limits to ({},{}).".format(self.id, self.min_pos, self.max_pos, self.min_vel, self.max_vel))
        # print("ANGLE:")
        # printC(RANGE, "%s %s" % (self.min_vel, self.max_vel))
        # print("VELOCITY:")
        # printC(RANGE, "%s %s" % (self.min_pos, self.max_pos))

    def write(self, flag, value):
        if flag not in self.model:
            rospy.logwarn("[ID:{:03}] Unable to write value {}. Flag was not specified. ".format(self.id, value))
            # printC(ERROR, "Flag not specified. Options are:")
            for key in self.model:
                print ("\33[92m\t- {}\33[0m".format(key))
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
            # printC(ERROR, "Fatal: Undefined data length")
            rospy.logerr("[ID:{:03}] Fatal error: Undefined data length. Check models.".format(self.id))
            quit()

        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn("[ID:{:03}] Unable to get feedback of getTxRxResult. Error {}.".format(self.id, self.packetHandler.getTxRxResult(dxl_comm_result)))
            # printC(WARNING, self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            rospy.logwarn("[ID:{:03}] Unable to get feedback of getRxPacketError. Error {}.".format(self.id, self.packetHandler.getTxRxResult(dxl_comm_result)))
            # printC(WARNING,  self.packetHandler.getRxPacketError(dxl_error))
            return False

        return True

    def read(self, flag):
        if flag not in self.model:
            rospy.logwarn("[ID:{:03}] Unable to read a value. Flag was not specified. ".format(self.id))
            for key in self.model:
                print ("\33[92m\t- {}\33[0m".format(key))
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
            rospy.logerr("[ID:{:03}] Fatal error: Undefined data length. Check models.".format(self.id))
            # print "Fatal: Unsupported data length"
            quit()

        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn("[ID:{:03}] Unable to get feedback of getTxRxResult. Error {}.".format(self.id, self.packetHandler.getTxRxResult(dxl_comm_result)))
            # printC(WARNING, "%s" % self.packetHandler.getTxRxResult(
            #     dxl_comm_result))
            return 0, False
        elif dxl_error != 0:
            rospy.logwarn("[ID:{:03}] Unable to get feedback of getRxPacketError. Error {}.".format(self.id, self.packetHandler.getTxRxResult(dxl_comm_result)))
            # printC(WARNING, "%s" % self.packetHandler.getRxPacketError(
            #     dxl_error))
            return 0, False

        return result, True

    # Receives the position in degrees and converts to actual value
    def writeAngle(self, angle):
        max_angle = self.limits['Max_Angle']
        min_angle = self.limits['Min_Angle']
        max_position = self.limits['Max_Position']
        min_position = self.limits['Min_Position']
        if angle < min_angle or angle > max_angle:
            # print "Error: Angle out of range"
            rospy.logwarn("[ID:{:03}] Unable to set angle {}. Out of range value ({},{})".format(self.id, angle, min_angle, max_angle))
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

    def writeVelocity(self, vel):
        min_vel = self.limits['Min_Velocity']
        max_vel = self.limits['Max_Velocity']
        if vel < min_vel or vel > max_vel:
            # print "Error: Velocity out of range"
            rospy.logwarn("[ID:{:03}] Unable to set velocity {}. Out of range value ({},{})".format(self.id, vel, min_vel, max_vel))
            return False

        # Convert vel to value
        if self.protocol == 2:
            value = int(np.ceil(vel * (60.0/(2.0*np.pi*0.229))))

            # Cap vel to maximum vel set on the dynamixel
            value = min(value, self.max_vel) if value > 0 else max(
                value, self.min_vel)
        elif self.protocol == 1:
            value = int(np.ceil((vel/max_vel)/0.0009775)) if vel > 0 else 1024 + \
                int(np.ceil((-vel/max_vel)/0.0009775))

        result = self.write("Goal_Velocity", value)
        return result

    # Read funcions

    def readAngle(self):
        position, result = self.read("Present_Position")
        if not result:
            return 0.0, False
        max_angle = self.limits['Max_Angle']
        min_angle = self.limits['Min_Angle']
        max_position = self.limits['Max_Position']
        min_position = self.limits['Min_Position']
        ratio = (1.0*position)/abs(max_position-min_position)
        angle = ratio * (max_angle - min_angle)
        return round(angle, 6), True

    def readLoad(self):
        if self.protocol == 2:
            current, result = self.readCurrent()
            current /= 1000.0
            if not result:
                return 0.0, False
            load = (current - 0.10949)/0.83885
        elif self.protocol == 1:
            load, result = self.read("Present_Load")
            if not result:
                return 0.0, False
        return round(load, 6), True

    def readVelocity(self):
        velocity, result = self.read("Present_Velocity")
        if not result:
            return 0.0, False
        if self.protocol == 2:
            if (velocity > 2147483648):
                velocity = velocity - 4294967296
            velocity = (velocity * 0.229)*2*np.pi/60.0  # rad/s
        else:
            if (velocity > 1024):
                velocity = 1024 - velocity
            velocity = (velocity * 0.111)*2*np.pi/60.0  # rad/s
        return round(velocity, 6), True

    def readCurrent(self):
        if self.protocol == 2:
            current, result = self.read("Present_Current")
            if not result:
                return 0.0, False
            if (current > 32768):
                current = current - 65536
            current = current*3.36  # mA
        else:
            current = 0.0
        return round(current, 2), True

    def readVoltage(self):
        voltage, result = self.read("Present_Voltage")
        if not result:
            return 0.0, False
        return float(voltage)/10, True

    def readTemp(self):
        temperature, result = self.read("Present_Temperature")
        if not result:
            return 0.0, False
        return temperature, True

    def reachedGoal(self):
        position, result = self.read("Present_Position")
        if not result:
            return True

        return abs(position-self.goal_pos) <= 1

    def isMoving(self):
        return self.read("Moving")
