#!/usr/bin/python3
import threading
import math
import re
import rclpy
import moteus
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from moteus_drive.MoteusDrive import MoteusDrive
from sensor_msgs.msg import Imu
from moteus_msgs.msg import MoteusState, MoteusStateStamped, MoteusCommandStamped

class MoteusNode(Node):
    def __init__(self):
        super().__init__('moteus_drive')
        # declare parameters
        self.declare_param()
        
        # get parameters values
        self.frame_id = self.get_parameter("frame_id").value
        self.connector_device = self.get_parameter("connector_device").value
        self.rezero_on_startup = self.get_parameter("rezero_on_startup").value
        self.devices = self.get_parameter("moteus_ids").value
        
        # initialize variables and objects
        self.command = None
        self.time_command = self.get_clock().now().to_msg()
        self.time_command_seconds = self.get_clock().now().to_msg()
        self.recv_command = {}
        
        # initialize moteus drive cycle
        self.moteusDrive = MoteusDrive(
            self.connector_device, 
            self.devices
        )
        
        # initialize start state drive
        self.drive_rezero()
        
        # create thread for moteus drive cycle
        thread = threading.Thread(target=self.moteusDrive.run_start, args=())
        thread.daemon = True
        
        #start thread
        thread.start()
        self.get_logger().info('MoteusNode started')
        
        # create publisher for moteus state
        self.publisher_ = self.create_publisher(MoteusStateStamped, 'moteus_feedback', 10)
        if self.connector_device == "pi3hat":
            self.publisher_imu_ = self.create_publisher(Imu, 'moteus_imu', 100)
        self.subscriber_ = self.create_subscription(MoteusCommandStamped, 'moteus_command', self.callback_command, 10)
    
        # create timer interval
        self.timer_state = self.create_timer(0.1, self.state_check)
        self.timer_update = self.create_timer(0.01, self.interval_update)
        
    def declare_param(self):
        self.declare_parameter("frame_id", "moteus_drive", ParameterDescriptor(description="Frame ID"))
        self.declare_parameter("connector_device", "pi3hat", ParameterDescriptor(description="Conector device"))
        self.declare_parameter("rezero_on_startup", False, ParameterDescriptor(description="Rezero on startup"))
        self.declare_parameter("moteus_ids", [1], ParameterDescriptor(description="Moteus IDs"))

    def callback_command(self, msg):
        self.time_command = msg.header.stamp
        self.commands = msg.commands
        self.recv_command = {}
        for idx, command in enumerate(self.commands):
            self.recv_command[command.device_id] = {"velocity": command.velocity, "maximum_torque": command.maximum_torque, "position": None}
        # send commands
        self.moteusDrive.set_state_command(self.recv_command)
        
    def state_check(self):
        now_seconds = self.get_clock().now().to_msg()
        now_seconds = float(str(now_seconds.sec) + '.' + str(now_seconds.nanosec))
        self.time_command_seconds = float(str(self.time_command.sec) + '.' + str(self.time_command.nanosec))
        timeout = now_seconds - self.time_command_seconds
        if timeout >= 1: #timeout 1 second emergency stop
            self.moteusDrive.set_state_brake()

    def interval_update(self):
        drive_feedback = self.moteusDrive.get_feedback()
        if drive_feedback is not None:

            stamp = self.get_clock().now().to_msg()
            moteusStateStamped = MoteusStateStamped()
            moteusStateStamped.header.frame_id = self.frame_id
            moteusStateStamped.header.stamp = stamp
            for index, device in enumerate(self.devices):
                moteusStateMsg = MoteusState()
                moteusStateMsg.device_id = device
                moteusStateMsg.mode = drive_feedback[index].values[moteus.Register(device).MODE]
                moteusStateMsg.position = drive_feedback[index].values[moteus.Register(device).POSITION]
                moteusStateMsg.velocity = drive_feedback[index].values[moteus.Register(device).VELOCITY]
                moteusStateMsg.torque = drive_feedback[index].values[moteus.Register(device).TORQUE]
                moteusStateMsg.voltage = drive_feedback[index].values[moteus.Register(device).VOLTAGE]
                moteusStateMsg.temperature = drive_feedback[index].values[moteus.Register(device).TEMPERATURE]
                moteusStateMsg.fault = drive_feedback[index].values[moteus.Register(device).FAULT]
                moteusStateStamped.state.append(moteusStateMsg)
                try:
                    self.recv_command[device]["position"] = drive_feedback[index].values[moteus.Register(device).POSITION]
                except:
                    pass

            # data_string = drive_feedback[1:-1]

            # # split the string into a list of strings, one for each data point
            # data_points = re.split(r'}, ', data_string)

            # # create an empty dictionary to store the parsed data
            # parsed_data = {}

            # # iterate over the data points
            # for data_point in data_points:
            #     # add a comma at the end of each data point
            #     data_point += '}'
            #     # extract the device id
            #     device_id = re.match(r'(\d+)/', data_point).groups()[0]
            #     # extract the key-value pairs
            #     kvpairs = re.findall(r'(\w+)\((0x[0-9a-f]+)\): ([\w.-]+)', data_point)
            #     #create a dictionary for the device and add the key-value pairs
            #     device_data = {}
            #     for kv in kvpairs:
            #         device_data[kv[0]] = kv[2]
            #     parsed_data[device_id] = device_data
    
            # stamp = self.get_clock().now().to_msg()
            # moteusStateStamped = MoteusStateStamped()
            # moteusStateStamped.header.frame_id = self.frame_id
            # moteusStateStamped.header.stamp = stamp
            
            # for index, device in enumerate(self.devices):
            #     drive_id = str(device)
            #     moteusStateMsg = MoteusState()
                
            #     moteusStateMsg.device_id = device
            #     moteusStateMsg.mode = parsed_data[drive_id]["MODE"]
            #     moteusStateMsg.position = parsed_data[drive_id]["POSITION"]
            #     moteusStateMsg.velocity = parsed_data[drive_id]["VELOCITY"]
            #     moteusStateMsg.torque = parsed_data[drive_id]["TORQUE"]
            #     moteusStateMsg.voltage = parsed_data[drive_id]["VOLTAGE"]
            #     moteusStateMsg.temperature = parsed_data[drive_id]["TEMPERATURE"]
            #     moteusStateMsg.fault = parsed_data[drive_id]["FAULT"]
            #     moteusStateStamped.state.append(moteusStateMsg)
            #     try:
            #         self.recv_command[device]["position"] = parsed_data[drive_id]["POSITION"]
            #     except:
            #         pass
                
                # moteusStateMsg.device_id = device
                # moteusStateMsg.mode = drive_feedback[index].values[moteus.Register(device).MODE]
                # moteusStateMsg.position = drive_feedback[index].values[moteus.Register(device).POSITION]
                # moteusStateMsg.velocity = drive_feedback[index].values[moteus.Register(device).VELOCITY]
                # moteusStateMsg.torque = drive_feedback[index].values[moteus.Register(device).TORQUE]
                # moteusStateMsg.voltage = drive_feedback[index].values[moteus.Register(device).VOLTAGE]
                # moteusStateMsg.temperature = drive_feedback[index].values[moteus.Register(device).TEMPERATURE]
                # moteusStateMsg.fault = drive_feedback[index].values[moteus.Register(device).FAULT]
                # moteusStateStamped.state.append(moteusStateMsg)
                # try:
                #     self.recv_command[device]["position"] = drive_feedback[index].values[moteus.Register(device).POSITION]
                # except:
                #     pass
                
            self.publisher_.publish(moteusStateStamped)

            # if imu_feedback is not None:
            #     imu_data = Imu()
            #     imu_data.header.stamp = stamp
            #     imu_data.header.frame_id = "moteus_imu"
                
            #     # Quaternion
            #     att = imu_feedback.attitude
            #     imu_data.orientation.w = att.w
            #     imu_data.orientation.x = att.x
            #     imu_data.orientation.y = att.y
            #     imu_data.orientation.z = att.z
                
            #     # Angular velocity
            #     rate_dps = imu_feedback.rate_dps
            #     omega_x = rate_dps.x * math.pi / 180
            #     omega_y = rate_dps.y * math.pi / 180
            #     omega_z = rate_dps.z * math.pi / 180
            #     imu_data.angular_velocity.x = omega_x
            #     imu_data.angular_velocity.y = omega_y
            #     imu_data.angular_velocity.z = omega_z
                
            #     # Linear acceleration
            #     accel_mps2 = imu_feedback.accel_mps2
            #     imu_data.linear_acceleration.x = accel_mps2.x
            #     imu_data.linear_acceleration.y = accel_mps2.y
            #     imu_data.linear_acceleration.z = accel_mps2.z
            #     self.publisher_imu_.publish(imu_data)

    def drive_rezero(self):
        if self.rezero_on_startup:
            self.moteusDrive.set_state_rezero()
            self.get_logger().info('MoteusNode rezero')
        
    def drive_stop(self):
        self.moteusDrive.set_state_stop()
        
    def drive_terminate(self):
        self.get_logger().info('MoteusNode terminated')
        self.moteusDrive.set_state_terminated()
    
def main(args=None):
    try:
        rclpy.init(args=args)
        moteusNode = MoteusNode()
        rclpy.spin(moteusNode)
    except KeyboardInterrupt:
        moteusNode.drive_terminate()
    finally:
        # destroy the node explicitly
        moteusNode.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()